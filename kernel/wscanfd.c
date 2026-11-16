// SPDX-License-Identifier: GPL-2.0
/*
 * SocketCAN driver for the Waveshare USB-CAN-FD (non-B, USB 04d8:0053).
 *
 * The adapter is an NXP LPC54616 (2x Bosch M_CAN) running the stock Waveshare
 * firmware. Protocol reference: ../docs/index.html.
 *
 *   EP1 OUT  commands (0x80xx) and every TX frame (cmd 0x8004, no reply)
 *   EP1 IN   command replies
 *   EP2 IN   CAN1 RX records    EP3 IN  CAN2 RX records
 *
 * Firmware quirks this driver works around:
 *  - A second SET_CONFIGURATION wedges EP1 IN until power-cycle: never reset
 *    the configuration.
 *  - RX endpoints never NAK: an idle poll completes at once with 8 zero bytes,
 *    so idle URBs are re-armed through an hrtimer instead of resubmitted
 *    immediately.
 *  - There is no TX confirmation or error/bus-state reporting. The echo skb is
 *    released when the adapter has accepted the frame over USB; the firmware
 *    retries an unacknowledged frame for ~0.5 s and then drops it silently.
 */
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/ethtool.h>
#include <linux/random.h>
#include <linux/timekeeping.h>
#include <linux/unaligned.h>
#include <linux/usb.h>
#include <linux/workqueue.h>

#include <linux/can.h>
#include <linux/can/dev.h>
#include <linux/can/error.h>
#include <linux/can/length.h>
#include <linux/can/skb.h>

#define WS_VID			0x04d8
#define WS_PID			0x0053

#define WS_EP_CMD_OUT		0x01
#define WS_EP_CMD_IN		0x81
#define WS_NUM_CHANNELS		2

#define WS_HEAD			0xbeef
#define WS_TAIL			0xdead
#define WS_CHECK_BIAS		0xbead
#define WS_MAX_PAYLOAD		500
#define WS_HDR_LEN		8
#define WS_PKT_OVERHEAD		12
#define WS_BUF_LEN		512

#define WS_CMD_AUTH		0x8001
#define WS_CMD_INIT_CAN		0x8002
#define WS_CMD_START_CAN	0x8003
#define WS_CMD_TRANSMIT		0x8004
#define WS_CMD_DEVICE_INFO	0x8005
/* 0x8006 starts a firmware update: never send it. */
#define WS_CMD_RESET_CAN	0x8009
#define WS_CMD_CLOSE		0x800a
#define WS_CMD_FILTER		0x800b

#define WS_DEVICE_TYPE		41	/* device type byte in reset/status payloads */
#define WS_CAN_CLOCK		60000000
#define WS_DEFAULT_DBIT		0x03030a02	/* default data timing, 1 Mbit/s */

#define WS_CMD_TIMEOUT_MS	3000
#define WS_TX_SLOTS		16
#define WS_RX_URBS		2
#define WS_RX_BUF_LEN		4096	/* as tested from userspace at bus saturation */
#define WS_ERR_BACKOFF_NS	(10 * NSEC_PER_MSEC)
#define WS_DEVQ_MAX		8	/* ring size for the device-queue model */
#define WS_IDLE_LEN		8	/* idle poll: 8 zero bytes */

#define WS_REC_CLASSIC		21	/* R0 R1 data[8] 0x55 ts32 */
#define WS_REC_FD		76	/* R0 R1 data[64] ts32 */
#define WS_TX_CLASSIC_LEN	0x1a
#define WS_TX_FD_LEN		0x56
#define WS_TX_PKT_MAX		(WS_PKT_OVERHEAD + WS_TX_FD_LEN)

/* M_CAN RX element header bits */
#define WS_R0_ESI		BIT(31)
#define WS_R0_XTD		BIT(30)
#define WS_R0_RTR		BIT(29)
#define WS_R1_FDF		BIT(21)
#define WS_R1_BRS		BIT(20)

static unsigned int idle_poll_us = 1000;
module_param(idle_poll_us, uint, 0644);
MODULE_PARM_DESC(idle_poll_us,
		 "RX poll interval while the bus is idle, us (default 1000; lower = less latency, more USB interrupts)");

/*
 * The firmware buffers only ~10 TX frames, keeps accepting USB writes when
 * full and silently drops the excess. We model the device queue from the bit
 * rate and keep at most this many frames in it; the margin covers bus time lost
 * to other nodes.
 */
static unsigned int tx_dev_queue = 4;
module_param(tx_dev_queue, uint, 0644);
MODULE_PARM_DESC(tx_dev_queue, "max frames queued inside the adapter (1-8, default 4)");

/*
 * Floor on the per-frame time: the firmware cannot move frames faster than
 * about (300 + 2 * data_len) us each, whatever the bit rate (measured on fw 2.12:
 * 0-16 B ok at 300 us, 32 B at 350 us, 64 B at 400 us).
 */
static unsigned int tx_min_frame_us = 300;
module_param(tx_min_frame_us, uint, 0644);
MODULE_PARM_DESC(tx_min_frame_us,
		 "minimum time budgeted per TX frame, us, plus 2 us per data byte (default 300)");

static const u8 ws_auth_template[32] = {
	0xde, 0xff, 0x00, 0x3f, 0xaf, 0x67, 0x43, 0x01,
	0xf2, 0x89, 0x82, 0xee, 0xce, 0x2f, 0x22, 0x16,
	0xd0, 0x1f, 0x07, 0x11, 0x5f, 0x68, 0xb3, 0xb1,
	0x2c, 0xe9, 0xc2, 0x3e, 0xc8, 0x26, 0x52, 0x36,
};

struct ws_usb;
struct ws_chan;

struct ws_tx_ctx {
	struct ws_chan *ch;
	unsigned int idx;
	unsigned int len;		/* data bytes, for tx_bytes */
};

struct ws_rx {
	struct ws_chan *ch;
	struct urb *urb;
	struct hrtimer timer;
};

struct ws_chan {
	struct can_priv can;		/* must be first */
	struct ws_usb *ws;
	struct net_device *netdev;
	unsigned int index;

	bool running;			/* RX URBs may be (re)submitted */
	bool tx_running;		/* xmit may submit, completions may wake */
	struct usb_anchor rx_anchor;
	struct ws_rx rx[WS_RX_URBS];
	struct work_struct clear_halt_work;

	struct usb_anchor tx_anchor;
	spinlock_t tx_lock;		/* protects tx_used and the devq_* model */
	unsigned long tx_used;		/* bitmap of busy echo slots */
	ktime_t devq_finish[WS_DEVQ_MAX];	/* estimated wire-finish time per queued frame */
	unsigned int devq_head, devq_count;
	struct hrtimer tx_timer;	/* wakes the queue as the device queue drains */
	u32 nom_bit_ns, data_bit_ns;
	struct ws_tx_ctx tx_ctx[WS_TX_SLOTS];
};

struct ws_usb {
	struct usb_device *udev;
	struct usb_interface *intf;
	struct mutex cmd_lock;		/* one command/reply exchange at a time */
	u8 *cmd_buf;
	u8 *rsp_buf;
	struct ws_chan *chan[WS_NUM_CHANNELS];
	char serial[21];
	u16 hw_version, fw_version;
};

static const struct can_bittiming_const ws_bittiming_const = {
	.name = KBUILD_MODNAME,
	.tseg1_min = 2,
	.tseg1_max = 256,
	.tseg2_min = 2,
	.tseg2_max = 128,
	.sjw_max = 128,
	.brp_min = 1,
	.brp_max = 256,		/* 8-bit field in the firmware's timing word */
	.brp_inc = 1,
};

static const struct can_bittiming_const ws_data_bittiming_const = {
	.name = KBUILD_MODNAME,
	.tseg1_min = 1,
	.tseg1_max = 32,
	.tseg2_min = 1,
	.tseg2_max = 16,
	.sjw_max = 16,
	.brp_min = 1,
	.brp_max = 32,
	.brp_inc = 1,
};

/* ------------------------------------------------------------------ framing */

static u16 ws_checksum(u16 cmd, const u8 *payload, u16 len)
{
	u32 c = 0x100 + 0x100 + len + 2 * cmd;	/* pkgcur = pkgall = 1 */
	u16 i;

	for (i = 0; i + 1 < len; i += 2)
		c += (payload[i] << 8) | payload[i + 1];
	return (c + WS_CHECK_BIAS) & 0xffff;
}

/* Builds a framed packet in @buf; @payload may already live at buf + 8. */
static int ws_pack(u8 *buf, u16 cmd, const u8 *payload, u16 len)
{
	if (len > WS_MAX_PAYLOAD)
		return -EINVAL;
	put_unaligned_be16(WS_HEAD, buf);
	put_unaligned_be16(len, buf + 2);
	buf[4] = 1;
	buf[5] = 1;
	put_unaligned_be16(cmd, buf + 6);
	if (payload && payload != buf + WS_HDR_LEN)
		memcpy(buf + WS_HDR_LEN, payload, len);
	put_unaligned_be16(ws_checksum(cmd, buf + WS_HDR_LEN, len), buf + WS_HDR_LEN + len);
	put_unaligned_be16(WS_TAIL, buf + WS_HDR_LEN + len + 2);
	return len + WS_PKT_OVERHEAD;
}

static int ws_unpack(const u8 *buf, int actual, u16 *cmd, const u8 **payload, u16 *len)
{
	u16 n;

	if (actual < WS_PKT_OVERHEAD || get_unaligned_be16(buf) != WS_HEAD)
		return -EPROTO;
	n = get_unaligned_be16(buf + 2);
	if (n > WS_MAX_PAYLOAD || actual < n + WS_PKT_OVERHEAD)
		return -EPROTO;
	if (get_unaligned_be16(buf + WS_HDR_LEN + n + 2) != WS_TAIL)
		return -EPROTO;
	*cmd = get_unaligned_be16(buf + 6);
	if (get_unaligned_be16(buf + WS_HDR_LEN + n) != ws_checksum(*cmd, buf + WS_HDR_LEN, n))
		return -EBADMSG;
	*payload = buf + WS_HDR_LEN;
	*len = n;
	return 0;
}

/* --------------------------------------------------------------- commands */

/* Discard anything queued on the reply endpoint. Caller holds cmd_lock (or probe). */
static void ws_drain_cmd_in_locked(struct ws_usb *ws)
{
	int i, actual;

	for (i = 0; i < 8; i++)
		if (usb_bulk_msg(ws->udev, usb_rcvbulkpipe(ws->udev, WS_EP_CMD_IN),
				 ws->rsp_buf, WS_BUF_LEN, &actual, 20))
			return;
}

/*
 * Send a command and wait for its reply. The reply payload is copied to
 * @reply (up to @reply_max bytes); returns the reply payload length.
 */
static int ws_cmd(struct ws_usb *ws, u16 cmd, const u8 *payload, u16 len,
		  u8 *reply, u16 reply_max)
{
	const u8 *rp;
	u16 rcmd, rlen;
	int n, actual, err, tries;

	mutex_lock(&ws->cmd_lock);
	n = ws_pack(ws->cmd_buf, cmd, payload, len);
	if (n < 0) {
		err = n;
		goto out;
	}
	err = usb_bulk_msg(ws->udev, usb_sndbulkpipe(ws->udev, WS_EP_CMD_OUT),
			   ws->cmd_buf, n, &actual, WS_CMD_TIMEOUT_MS);
	if (!err && actual != n)
		err = -EIO;
	if (err)
		goto out;
	/* A late reply to an earlier, timed-out command may be queued first: skip it. */
	for (tries = 0; tries < 4; tries++) {
		err = usb_bulk_msg(ws->udev, usb_rcvbulkpipe(ws->udev, WS_EP_CMD_IN),
				   ws->rsp_buf, WS_BUF_LEN, &actual, WS_CMD_TIMEOUT_MS);
		if (err)
			goto out;
		err = ws_unpack(ws->rsp_buf, actual, &rcmd, &rp, &rlen);
		if (!err && rcmd == cmd)
			break;
		if (!err)
			err = -EPROTO;
	}
	if (err)
		goto out;
	if (reply)
		memcpy(reply, rp, min(rlen, reply_max));
	err = rlen;
out:
	if (err < 0 && err != -ENODEV) {
		ws_drain_cmd_in_locked(ws);
		dev_warn_ratelimited(&ws->intf->dev, "cmd %#06x failed: %d\n", cmd, err);
	}
	mutex_unlock(&ws->cmd_lock);
	return err;
}

static int ws_auth(struct ws_usb *ws)
{
	u8 c[32];
	u32 addr = 0, now = (u32)ktime_get_real_seconds(), rnd = get_random_u32();

	memcpy(c, ws_auth_template, sizeof(c));
	put_unaligned_be32(addr, c + 2);
	put_unaligned_be32(now, c + 12);
	put_unaligned_be32(rnd, c + 22);
	/* The 32-byte reply is not validated. */
	return ws_cmd(ws, WS_CMD_AUTH, c, sizeof(c), NULL, 0);
}

static int ws_read_info(struct ws_usb *ws)
{
	u8 info[76];
	int n = ws_cmd(ws, WS_CMD_DEVICE_INFO, NULL, 0, info, sizeof(info));

	if (n < 28)
		return n < 0 ? n : -EPROTO;
	ws->hw_version = get_unaligned_be16(info);
	ws->fw_version = get_unaligned_be16(info + 2);
	memcpy(ws->serial, info + 8, 20);
	ws->serial[20] = 0;
	return 0;
}

static u32 ws_timing_word(const struct can_bittiming *bt)
{
	u32 tseg1 = bt->prop_seg + bt->phase_seg1;

	return ((bt->brp - 1) << 24) | ((bt->sjw - 1) << 16) |
	       ((tseg1 - 1) << 8) | (bt->phase_seg2 - 1);
}

/* Throw away RX records buffered from before this start (frames from a previous session). */
static void ws_drain_rx(struct ws_chan *ch)
{
	struct usb_device *udev = ch->ws->udev;
	int i, actual;
	u8 *buf = kmalloc(WS_RX_BUF_LEN, GFP_KERNEL);

	if (!buf)
		return;
	for (i = 0; i < 32; i++)
		if (usb_bulk_msg(udev, usb_rcvbulkpipe(udev, 0x82 + ch->index), buf,
				 WS_RX_BUF_LEN, &actual, 20) || actual <= WS_IDLE_LEN)
			break;
	kfree(buf);
}

static int ws_chan_init(struct ws_chan *ch)
{
	struct can_priv *priv = &ch->can;
	u8 b[32] = {};
	u32 dbit = WS_DEFAULT_DBIT;
	int err;

	if ((priv->ctrlmode & CAN_CTRLMODE_FD) && priv->fd.data_bittiming.brp)
		dbit = ws_timing_word(&priv->fd.data_bittiming);

	b[0] = 0x55;
	b[1] = 0x02;
	b[2] = ch->index;
	b[3] = 1;					/* can_type: CAN FD controller */
	b[4] = (priv->ctrlmode & CAN_CTRLMODE_LISTENONLY) ? 1 : 0;
	b[5] = 1;					/* single filter */
	put_unaligned_be32(0, b + 6);			/* acc_code */
	put_unaligned_be32(0xffffffff, b + 10);		/* acc_mask */
	put_unaligned_be32(ws_timing_word(&priv->bittiming), b + 18);
	put_unaligned_be32(dbit, b + 22);
	b[30] = (priv->ctrlmode & CAN_CTRLMODE_FD_NON_ISO) ? 0x80 : 0;

	netdev_dbg(ch->netdev, "init abit %#010x dbit %#010x mode %u\n",
		   get_unaligned_be32(b + 18), dbit, b[4]);
	err = ws_cmd(ch->ws, WS_CMD_INIT_CAN, b, sizeof(b), NULL, 0);
	if (err < 0)
		return err;

	b[0] = 0x7e;					/* clear filter: accept all */
	b[1] = ch->index;
	b[2] = 0x0f;
	b[3] = 0xff;
	err = ws_cmd(ch->ws, WS_CMD_FILTER, b, 4, NULL, 0);
	if (err < 0)
		return err;

	ws_drain_rx(ch);

	b[0] = 0x55;
	b[1] = 0x80;
	b[2] = 0x03;
	b[3] = ch->index;
	err = ws_cmd(ch->ws, WS_CMD_START_CAN, b, 4, NULL, 0);
	return err < 0 ? err : 0;
}

static int ws_chan_reset(struct ws_chan *ch)
{
	u8 b[8] = { 0x55, 0x09, WS_DEVICE_TYPE, 0, ch->index, 0, 0, 0 };
	int err = ws_cmd(ch->ws, WS_CMD_RESET_CAN, b, sizeof(b), NULL, 0);

	return err < 0 ? err : 0;
}

/* --------------------------------------------------------------------- RX */

static void ws_rx_frame(struct ws_chan *ch, const u8 *rec, u32 r0, u32 r1)
{
	struct net_device *netdev = ch->netdev;
	struct net_device_stats *stats = &netdev->stats;
	bool xtd = r0 & WS_R0_XTD, rtr = r0 & WS_R0_RTR;
	canid_t id = xtd ? (r0 & CAN_EFF_MASK) | CAN_EFF_FLAG : (r0 >> 18) & CAN_SFF_MASK;
	u8 dlc = (r1 >> 16) & 0xf;
	struct sk_buff *skb;

	if (r1 & WS_R1_FDF) {
		struct canfd_frame *cfd;

		/* The controller always runs FD-capable; hide FD frames from a classic netdev. */
		if (!(ch->can.ctrlmode & CAN_CTRLMODE_FD))
			goto drop;
		skb = alloc_canfd_skb(netdev, &cfd);
		if (!skb)
			goto drop;
		cfd->can_id = id;
		cfd->len = can_fd_dlc2len(dlc);
		if (r1 & WS_R1_BRS)
			cfd->flags |= CANFD_BRS;
		if (r0 & WS_R0_ESI)
			cfd->flags |= CANFD_ESI;
		memcpy(cfd->data, rec + 8, cfd->len);
		stats->rx_bytes += cfd->len;
	} else {
		struct can_frame *cf;

		skb = alloc_can_skb(netdev, &cf);
		if (!skb)
			goto drop;
		cf->can_id = id;
		cf->len = can_cc_dlc2len(dlc);
		if (rtr) {
			cf->can_id |= CAN_RTR_FLAG;
		} else {
			memcpy(cf->data, rec + 8, cf->len);
			stats->rx_bytes += cf->len;
		}
	}
	stats->rx_packets++;
	netif_rx(skb);
	return;
drop:
	stats->rx_dropped++;
}

static void ws_devq_charge_rx(struct ws_usb *ws, u64 ns);

/* Returns the number of records parsed (0 for an idle poll). */
static int ws_rx_parse(struct ws_chan *ch, const u8 *buf, int len)
{
	u64 fw_ns = 0;
	int pos = 0, n = 0;

	while (pos + 8 <= len) {
		u32 r0 = get_unaligned_le32(buf + pos);
		u32 r1 = get_unaligned_le32(buf + pos + 4);
		int size = (r1 & WS_R1_FDF) ? WS_REC_FD : WS_REC_CLASSIC;

		if (pos + size > len)
			break;
		if (!(r1 & WS_R1_FDF) && buf[pos + 16] != 0x55)
			break;		/* idle poll (8 zero bytes) or garbage */
		ws_rx_frame(ch, buf + pos, r0, r1);
		fw_ns += ((u64)READ_ONCE(tx_min_frame_us) +
			  2 * ((r1 & WS_R1_FDF) ? can_fd_dlc2len((r1 >> 16) & 0xf) : 8)) * NSEC_PER_USEC;
		pos += size;
		n++;
	}
	if (fw_ns)
		ws_devq_charge_rx(ch->ws, fw_ns);
	if (pos < len && len != 8)
		netdev_dbg(ch->netdev, "rx: %d unparsed bytes of %d\n", len - pos, len);
	return n;
}

static int ws_rx_submit(struct ws_rx *rx, gfp_t gfp)
{
	int err;

	usb_anchor_urb(rx->urb, &rx->ch->rx_anchor);
	err = usb_submit_urb(rx->urb, gfp);
	if (err)
		usb_unanchor_urb(rx->urb);
	return err;
}

/* Submit, or on failure re-arm the timer with a back-off so RX never silently dies. */
static void ws_rx_resubmit(struct ws_rx *rx)
{
	struct ws_chan *ch = rx->ch;
	int err;

	if (!READ_ONCE(ch->running))
		return;
	err = ws_rx_submit(rx, GFP_ATOMIC);
	if (!err)
		return;
	if (err == -ENODEV) {
		netif_device_detach(ch->netdev);
		return;
	}
	net_warn_ratelimited("%s: rx resubmit failed: %d, retrying\n", netdev_name(ch->netdev), err);
	if (READ_ONCE(ch->running))
		hrtimer_start(&rx->timer, ns_to_ktime(WS_ERR_BACKOFF_NS), HRTIMER_MODE_REL_SOFT);
}

static enum hrtimer_restart ws_rx_timer(struct hrtimer *t)
{
	ws_rx_resubmit(container_of(t, struct ws_rx, timer));
	return HRTIMER_NORESTART;
}

static void ws_clear_halt_work(struct work_struct *work)
{
	struct ws_chan *ch = container_of(work, struct ws_chan, clear_halt_work);
	struct usb_device *udev = ch->ws->udev;
	int i, err;

	err = usb_clear_halt(udev, usb_rcvbulkpipe(udev, 0x82 + ch->index));
	if (err)
		netdev_warn(ch->netdev, "clear halt failed: %d\n", err);
	for (i = 0; i < WS_RX_URBS; i++)
		if (READ_ONCE(ch->running))
			hrtimer_start(&ch->rx[i].timer, ns_to_ktime(WS_ERR_BACKOFF_NS),
				      HRTIMER_MODE_REL_SOFT);
}

static void ws_rx_complete(struct urb *urb)
{
	struct ws_rx *rx = urb->context;
	struct ws_chan *ch = rx->ch;
	int frames;

	switch (urb->status) {
	case 0:
		break;
	case -ENOENT:
	case -ECONNRESET:
	case -ESHUTDOWN:
		return;		/* unlinked or device gone */
	case -EPIPE:
		/* Both URBs share the endpoint; the work item re-arms them. */
		if (READ_ONCE(ch->running))
			schedule_work(&ch->clear_halt_work);
		return;
	default:
		/* Transient (e.g. -EPROTO): retry after a back-off. */
		netdev_dbg(ch->netdev, "rx urb status %d\n", urb->status);
		if (READ_ONCE(ch->running))
			hrtimer_start(&rx->timer, ns_to_ktime(WS_ERR_BACKOFF_NS),
				      HRTIMER_MODE_REL_SOFT);
		return;
	}

	if (!READ_ONCE(ch->running))
		return;
	frames = ws_rx_parse(ch, urb->transfer_buffer, urb->actual_length);
	if (!frames) {
		/* Idle: the firmware never NAKs, so pace the polling. */
		hrtimer_start(&rx->timer, us_to_ktime(clamp(idle_poll_us, 50U, 100000U)),
			      HRTIMER_MODE_REL_SOFT);
		return;
	}
	ws_rx_resubmit(rx);
}

static void ws_rx_free(struct ws_chan *ch)
{
	int i;

	for (i = 0; i < WS_RX_URBS; i++) {
		struct urb *urb = ch->rx[i].urb;

		if (!urb)
			continue;
		usb_free_coherent(ch->ws->udev, WS_RX_BUF_LEN, urb->transfer_buffer,
				  urb->transfer_dma);
		usb_free_urb(urb);
		ch->rx[i].urb = NULL;
	}
}

static int ws_rx_start(struct ws_chan *ch)
{
	struct usb_device *udev = ch->ws->udev;
	u8 ep = 0x82 + ch->index;
	int i, err;

	for (i = 0; i < WS_RX_URBS; i++) {
		struct ws_rx *rx = &ch->rx[i];
		void *buf;

		rx->urb = usb_alloc_urb(0, GFP_KERNEL);
		if (!rx->urb)
			return -ENOMEM;
		buf = usb_alloc_coherent(udev, WS_RX_BUF_LEN, GFP_KERNEL, &rx->urb->transfer_dma);
		if (!buf) {
			usb_free_urb(rx->urb);
			rx->urb = NULL;
			return -ENOMEM;
		}
		usb_fill_bulk_urb(rx->urb, udev, usb_rcvbulkpipe(udev, ep), buf,
				  WS_RX_BUF_LEN, ws_rx_complete, rx);
		rx->urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
	}
	WRITE_ONCE(ch->running, true);
	for (i = 0; i < WS_RX_URBS; i++) {
		err = ws_rx_submit(&ch->rx[i], GFP_KERNEL);
		if (err)
			return err;
	}
	return 0;
}

static void ws_rx_stop(struct ws_chan *ch)
{
	int i;

	WRITE_ONCE(ch->running, false);
	usb_kill_anchored_urbs(&ch->rx_anchor);
	cancel_work_sync(&ch->clear_halt_work);
	/* A completion or the work item may have armed a timer before seeing !running. */
	for (i = 0; i < WS_RX_URBS; i++)
		hrtimer_cancel(&ch->rx[i].timer);
	usb_kill_anchored_urbs(&ch->rx_anchor);
	ws_rx_free(ch);
}

/* --------------------------------------------------------------------- TX */

static unsigned int ws_devq_limit(void)
{
	return clamp(tx_dev_queue, 1U, (unsigned int)WS_DEVQ_MAX);
}

/* Estimated time on the wire, worst-case stuffing, +10%. */
static u64 ws_frame_ns(const struct ws_chan *ch, const struct sk_buff *skb)
{
	const struct canfd_frame *cfd = (const struct canfd_frame *)skb->data;
	bool fd = can_is_canfd_skb(skb), eff = cfd->can_id & CAN_EFF_FLAG;
	unsigned int len = can_skb_get_data_len(skb);
	u64 ns;

	if (fd && (cfd->flags & CANFD_BRS) && ch->data_bit_ns)
		ns = (u64)can_frame_bits(true, eff, true, true, 0) * ch->nom_bit_ns +
		     (u64)DIV_ROUND_UP(len * 8 * 5, 4) * ch->data_bit_ns;
	else
		ns = (u64)can_frame_bits(fd, eff, true, true, len) * ch->nom_bit_ns;
	ns += ns / 10;
	return max_t(u64, ns, ((u64)READ_ONCE(tx_min_frame_us) + 2 * len) * NSEC_PER_USEC);
}

/* tx_lock held: forget frames the model says have left the adapter. */
static void ws_devq_expire(struct ws_chan *ch, ktime_t now)
{
	while (ch->devq_count && ktime_compare(ch->devq_finish[ch->devq_head], now) <= 0) {
		ch->devq_head = (ch->devq_head + 1) % WS_DEVQ_MAX;
		ch->devq_count--;
	}
}

/* tx_lock held: wake the queue if there is room, else arm the drain timer. */
static void ws_tx_update_queue(struct ws_chan *ch)
{
	struct net_device *netdev = ch->netdev;

	ws_devq_expire(ch, ktime_get());
	if (!READ_ONCE(ch->tx_running) || !netif_device_present(netdev))
		return;
	if (ch->devq_count >= ws_devq_limit()) {
		hrtimer_start(&ch->tx_timer, ch->devq_finish[ch->devq_head], HRTIMER_MODE_ABS_SOFT);
		return;
	}
	if (find_first_zero_bit(&ch->tx_used, WS_TX_SLOTS) < WS_TX_SLOTS)
		netif_wake_queue(netdev);
}

/*
 * Received frames use the same firmware time as transmitted ones (measured: with
 * a 1:1 echo load, TX needs about twice the per-frame floor), so they delay every
 * frame still queued in the adapter, on both channels.
 */
static void ws_devq_charge_rx(struct ws_usb *ws, u64 ns)
{
	int c;

	for (c = 0; c < WS_NUM_CHANNELS; c++) {
		struct ws_chan *ch = ws->chan[c];
		unsigned long flags;
		unsigned int i;

		if (!ch)
			continue;
		spin_lock_irqsave(&ch->tx_lock, flags);
		for (i = 0; i < ch->devq_count; i++) {
			unsigned int k = (ch->devq_head + i) % WS_DEVQ_MAX;

			ch->devq_finish[k] = ktime_add_ns(ch->devq_finish[k], ns);
		}
		if (ch->devq_count && hrtimer_active(&ch->tx_timer))
			hrtimer_start(&ch->tx_timer, ch->devq_finish[ch->devq_head],
				      HRTIMER_MODE_ABS_SOFT);
		spin_unlock_irqrestore(&ch->tx_lock, flags);
	}
}

static enum hrtimer_restart ws_tx_timer(struct hrtimer *t)
{
	struct ws_chan *ch = container_of(t, struct ws_chan, tx_timer);
	unsigned long flags;

	spin_lock_irqsave(&ch->tx_lock, flags);
	ws_tx_update_queue(ch);
	spin_unlock_irqrestore(&ch->tx_lock, flags);
	return HRTIMER_NORESTART;
}

static void ws_tx_complete(struct urb *urb)
{
	struct ws_tx_ctx *ctx = urb->context;
	struct ws_chan *ch = ctx->ch;
	struct net_device *netdev = ch->netdev;
	unsigned long flags;

	if (!urb->status) {
		/* Accepted by the adapter -- the closest thing to a TX-done we get. */
		/* Returned 0 here in testing on 7.0, so count the length ourselves. */
		unsigned int echoed = can_get_echo_skb(netdev, ctx->idx, NULL);

		(void)echoed;
		netdev->stats.tx_bytes += ctx->len;
		netdev->stats.tx_packets++;
	} else {
		can_free_echo_skb(netdev, ctx->idx, NULL);
		if (urb->status != -ENOENT && urb->status != -ECONNRESET &&
		    urb->status != -ESHUTDOWN)
			netdev->stats.tx_errors++;
		else
			netdev->stats.tx_dropped++;
	}

	spin_lock_irqsave(&ch->tx_lock, flags);
	clear_bit(ctx->idx, &ch->tx_used);
	ws_tx_update_queue(ch);
	spin_unlock_irqrestore(&ch->tx_lock, flags);
}

static int ws_build_tx(struct ws_chan *ch, const struct sk_buff *skb, u8 *pkt)
{
	u8 *b = pkt + WS_HDR_LEN;

	if (can_is_canfd_skb(skb)) {
		const struct canfd_frame *cfd = (const struct canfd_frame *)skb->data;
		u8 len = can_fd_dlc2len(can_fd_len2dlc(cfd->len));

		memset(b, 0, WS_TX_FD_LEN);
		b[0] = 0x55;
		b[1] = 0xf2;
		b[4] = !!(cfd->can_id & CAN_EFF_FLAG);
		put_unaligned_be32(cfd->can_id & CAN_EFF_MASK, b + 6);
		b[10] = len;		/* data beyond cfd->len is zero (memset) */
		b[11] = (cfd->flags & CANFD_BRS) ? 1 : 0;
		b[12] = 1;
		b[13] = ch->index;
		memcpy(b + 14, cfd->data, cfd->len);
		return ws_pack(pkt, WS_CMD_TRANSMIT, b, WS_TX_FD_LEN);
	} else if (can_is_can_skb(skb)) {
		const struct can_frame *cf = (const struct can_frame *)skb->data;

		memset(b, 0, WS_TX_CLASSIC_LEN);
		b[0] = 0x55;
		b[1] = 0xf1;
		b[4] = !!(cf->can_id & CAN_EFF_FLAG);
		b[5] = !!(cf->can_id & CAN_RTR_FLAG);
		put_unaligned_be32(cf->can_id & CAN_EFF_MASK, b + 6);
		b[10] = cf->len;
		if (!(cf->can_id & CAN_RTR_FLAG))
			memcpy(b + 11, cf->data, cf->len);
		b[0x14] = ch->index;
		return ws_pack(pkt, WS_CMD_TRANSMIT, b, WS_TX_CLASSIC_LEN);
	}
	return -EINVAL;		/* e.g. CAN XL: not supported */
}

static netdev_tx_t ws_start_xmit(struct sk_buff *skb, struct net_device *netdev)
{
	struct ws_chan *ch = netdev_priv(netdev);
	struct usb_device *udev = ch->ws->udev;
	unsigned long flags;
	unsigned int idx;
	struct urb *urb;
	ktime_t now;
	u64 frame_ns;
	u8 *buf;
	int len, err;

	if (can_dev_dropped_skb(netdev, skb))
		return NETDEV_TX_OK;
	if (!can_is_can_skb(skb) && !can_is_canfd_skb(skb)) {
		netdev->stats.tx_dropped++;
		kfree_skb(skb);
		return NETDEV_TX_OK;
	}

	frame_ns = ws_frame_ns(ch, skb);

	spin_lock_irqsave(&ch->tx_lock, flags);
	idx = find_first_zero_bit(&ch->tx_used, WS_TX_SLOTS);
	if (WARN_ON_ONCE(idx >= WS_TX_SLOTS)) {
		netif_stop_queue(netdev);
		spin_unlock_irqrestore(&ch->tx_lock, flags);
		return NETDEV_TX_BUSY;
	}
	set_bit(idx, &ch->tx_used);
	now = ktime_get();
	ws_devq_expire(ch, now);
	if (ch->devq_count < WS_DEVQ_MAX) {
		ktime_t start = now;

		if (ch->devq_count) {
			ktime_t last = ch->devq_finish[(ch->devq_head + ch->devq_count - 1) % WS_DEVQ_MAX];

			if (ktime_after(last, start))
				start = last;
		}
		ch->devq_finish[(ch->devq_head + ch->devq_count) % WS_DEVQ_MAX] =
			ktime_add_ns(start, frame_ns);
		ch->devq_count++;
	}
	if (ch->devq_count >= ws_devq_limit()) {
		netif_stop_queue(netdev);
		hrtimer_start(&ch->tx_timer, ch->devq_finish[ch->devq_head], HRTIMER_MODE_ABS_SOFT);
	} else if (find_first_zero_bit(&ch->tx_used, WS_TX_SLOTS) >= WS_TX_SLOTS) {
		netif_stop_queue(netdev);
	}
	spin_unlock_irqrestore(&ch->tx_lock, flags);

	urb = usb_alloc_urb(0, GFP_ATOMIC);
	buf = kmalloc(WS_TX_PKT_MAX, GFP_ATOMIC);
	if (!urb || !buf)
		goto nomem;

	len = ws_build_tx(ch, skb, buf);
	usb_fill_bulk_urb(urb, udev, usb_sndbulkpipe(udev, WS_EP_CMD_OUT), buf, len,
			  ws_tx_complete, &ch->tx_ctx[idx]);
	urb->transfer_flags |= URB_FREE_BUFFER;

	ch->tx_ctx[idx].len = can_skb_get_data_len(skb);
	can_put_echo_skb(skb, netdev, idx, 0);
	usb_anchor_urb(urb, &ch->tx_anchor);
	err = usb_submit_urb(urb, GFP_ATOMIC);
	if (err) {
		usb_unanchor_urb(urb);
		can_free_echo_skb(netdev, idx, NULL);
		netdev->stats.tx_dropped++;
		if (err == -ENODEV)
			netif_device_detach(netdev);
		else
			netdev_warn(netdev, "tx submit failed: %d\n", err);
		goto release_slot;
	}
	usb_free_urb(urb);	/* the anchor holds a reference until completion */
	return NETDEV_TX_OK;

nomem:
	kfree(buf);
	dev_kfree_skb_any(skb);
	netdev->stats.tx_dropped++;
release_slot:
	usb_free_urb(urb);
	spin_lock_irqsave(&ch->tx_lock, flags);
	clear_bit(idx, &ch->tx_used);
	ws_tx_update_queue(ch);
	spin_unlock_irqrestore(&ch->tx_lock, flags);
	return NETDEV_TX_OK;
}

/* ---------------------------------------------------------------- netdev */

static int ws_chan_start(struct ws_chan *ch)
{
	int err = ws_chan_init(ch);

	if (err) {
		netdev_err(ch->netdev, "channel init failed: %d\n", err);
		return err;
	}
	err = ws_rx_start(ch);
	if (err) {
		ws_rx_stop(ch);
		ws_chan_reset(ch);
		return err;
	}
	ch->nom_bit_ns = DIV_ROUND_UP(NSEC_PER_SEC, ch->can.bittiming.bitrate);
	ch->data_bit_ns = (ch->can.ctrlmode & CAN_CTRLMODE_FD) && ch->can.fd.data_bittiming.bitrate ?
		DIV_ROUND_UP(NSEC_PER_SEC, ch->can.fd.data_bittiming.bitrate) : 0;
	ch->devq_head = 0;
	ch->devq_count = 0;
	ch->can.state = CAN_STATE_ERROR_ACTIVE;
	WRITE_ONCE(ch->tx_running, true);
	return 0;
}

static void ws_chan_stop(struct ws_chan *ch)
{
	WRITE_ONCE(ch->tx_running, false);
	hrtimer_cancel(&ch->tx_timer);
	ws_rx_stop(ch);
	usb_kill_anchored_urbs(&ch->tx_anchor);
	hrtimer_cancel(&ch->tx_timer);	/* a completion may have re-armed it */
	ws_chan_reset(ch);
	ch->can.state = CAN_STATE_STOPPED;
}

static int ws_open(struct net_device *netdev)
{
	struct ws_chan *ch = netdev_priv(netdev);
	int err;

	err = open_candev(netdev);
	if (err)
		return err;
	err = ws_chan_start(ch);
	if (err) {
		close_candev(netdev);
		return err;
	}
	netif_start_queue(netdev);
	return 0;
}

static int ws_stop(struct net_device *netdev)
{
	struct ws_chan *ch = netdev_priv(netdev);

	netif_stop_queue(netdev);
	ws_chan_stop(ch);
	close_candev(netdev);
	return 0;
}

static const struct net_device_ops ws_netdev_ops = {
	.ndo_open = ws_open,
	.ndo_stop = ws_stop,
	.ndo_start_xmit = ws_start_xmit,
};

static const struct ethtool_ops ws_ethtool_ops = {
	.get_ts_info = ethtool_op_get_ts_info,
};

/* ------------------------------------------------------------------ USB */

static int ws_create_chan(struct ws_usb *ws, unsigned int index)
{
	struct net_device *netdev;
	struct ws_chan *ch;
	int i, err;

	netdev = alloc_candev(sizeof(*ch), WS_TX_SLOTS);
	if (!netdev)
		return -ENOMEM;
	ch = netdev_priv(netdev);
	ch->ws = ws;
	ch->netdev = netdev;
	ch->index = index;
	spin_lock_init(&ch->tx_lock);
	init_usb_anchor(&ch->rx_anchor);
	init_usb_anchor(&ch->tx_anchor);
	INIT_WORK(&ch->clear_halt_work, ws_clear_halt_work);
	hrtimer_setup(&ch->tx_timer, ws_tx_timer, CLOCK_MONOTONIC, HRTIMER_MODE_ABS_SOFT);
	for (i = 0; i < WS_TX_SLOTS; i++) {
		ch->tx_ctx[i].ch = ch;
		ch->tx_ctx[i].idx = i;
	}
	for (i = 0; i < WS_RX_URBS; i++) {
		ch->rx[i].ch = ch;
		hrtimer_setup(&ch->rx[i].timer, ws_rx_timer, CLOCK_MONOTONIC,
			      HRTIMER_MODE_REL_SOFT);
	}

	ch->can.clock.freq = WS_CAN_CLOCK;
	ch->can.bittiming_const = &ws_bittiming_const;
	ch->can.fd.data_bittiming_const = &ws_data_bittiming_const;
	ch->can.ctrlmode_supported = CAN_CTRLMODE_LISTENONLY | CAN_CTRLMODE_FD |
				     CAN_CTRLMODE_FD_NON_ISO;

	netdev->netdev_ops = &ws_netdev_ops;
	netdev->ethtool_ops = &ws_ethtool_ops;
	netdev->dev_port = index;
	SET_NETDEV_DEV(netdev, &ws->intf->dev);

	err = register_candev(netdev);
	if (err) {
		free_candev(netdev);
		return err;
	}
	ws->chan[index] = ch;
	netdev_info(netdev, "Waveshare USB-CAN-FD channel CAN%u\n", index + 1);
	return 0;
}

static void ws_destroy_chans(struct ws_usb *ws)
{
	int i;

	/* Stop every channel before freeing any: RX on one channel touches the other's TX model. */
	for (i = 0; i < WS_NUM_CHANNELS; i++)
		if (ws->chan[i])
			unregister_candev(ws->chan[i]->netdev);
	for (i = 0; i < WS_NUM_CHANNELS; i++) {
		if (!ws->chan[i])
			continue;
		free_candev(ws->chan[i]->netdev);
		ws->chan[i] = NULL;
	}
}

static bool ws_has_endpoints(struct usb_interface *intf)
{
	static const u8 want[] = { 0x01, 0x81, 0x82, 0x83 };
	struct usb_host_interface *alt = intf->cur_altsetting;
	int i, j, found = 0;

	for (i = 0; i < ARRAY_SIZE(want); i++)
		for (j = 0; j < alt->desc.bNumEndpoints; j++)
			if (alt->endpoint[j].desc.bEndpointAddress == want[i] &&
			    usb_endpoint_xfer_bulk(&alt->endpoint[j].desc)) {
				found++;
				break;
			}
	return found == ARRAY_SIZE(want);
}

static int ws_probe(struct usb_interface *intf, const struct usb_device_id *id)
{
	struct ws_usb *ws;
	int i, err;

	if (intf->cur_altsetting->desc.bInterfaceNumber != 0 || !ws_has_endpoints(intf))
		return -ENODEV;

	ws = kzalloc(sizeof(*ws), GFP_KERNEL);
	if (!ws)
		return -ENOMEM;
	ws->udev = interface_to_usbdev(intf);
	ws->intf = intf;
	mutex_init(&ws->cmd_lock);
	ws->cmd_buf = kmalloc(WS_BUF_LEN, GFP_KERNEL);
	ws->rsp_buf = kmalloc(WS_BUF_LEN, GFP_KERNEL);
	if (!ws->cmd_buf || !ws->rsp_buf) {
		err = -ENOMEM;
		goto err_free;
	}
	usb_set_intfdata(intf, ws);

	ws_drain_cmd_in_locked(ws);	/* no other users yet */
	err = ws_auth(ws);
	if (err < 0) {
		dev_err(&intf->dev,
			"no reply to open command (%d); if the device was reconfigured it needs a power cycle\n",
			err);
		goto err_free;
	}
	err = ws_read_info(ws);
	if (err) {
		dev_err(&intf->dev, "device info failed: %d\n", err);
		goto err_free;
	}
	dev_info(&intf->dev, "Waveshare USB-CAN-FD serial %s hw %x.%02x fw %x.%02x\n",
		 ws->serial, ws->hw_version >> 8, ws->hw_version & 0xff,
		 ws->fw_version >> 8, ws->fw_version & 0xff);

	for (i = 0; i < WS_NUM_CHANNELS; i++) {
		err = ws_create_chan(ws, i);
		if (err)
			goto err_chans;
	}
	return 0;

err_chans:
	ws_destroy_chans(ws);
err_free:
	usb_set_intfdata(intf, NULL);
	kfree(ws->cmd_buf);
	kfree(ws->rsp_buf);
	kfree(ws);
	return err;
}

static void ws_disconnect(struct usb_interface *intf)
{
	struct ws_usb *ws = usb_get_intfdata(intf);
	u8 close[2] = { 0x55, 0x0a };

	if (!ws)
		return;
	ws_destroy_chans(ws);
	/* Best effort (fails fast if the device is already gone). */
	if (ws->udev->state != USB_STATE_NOTATTACHED)
		ws_cmd(ws, WS_CMD_CLOSE, close, sizeof(close), NULL, 0);
	usb_set_intfdata(intf, NULL);
	kfree(ws->cmd_buf);
	kfree(ws->rsp_buf);
	kfree(ws);
}

static const struct usb_device_id ws_table[] = {
	{ USB_DEVICE_INTERFACE_NUMBER(WS_VID, WS_PID, 0) },
	{ }
};
MODULE_DEVICE_TABLE(usb, ws_table);

static struct usb_driver ws_driver = {
	.name = KBUILD_MODNAME,
	.probe = ws_probe,
	.disconnect = ws_disconnect,
	.id_table = ws_table,
	.soft_unbind = 1,	/* lets disconnect() still talk to the device on rmmod */
};
module_usb_driver(ws_driver);

MODULE_DESCRIPTION("SocketCAN driver for Waveshare USB-CAN-FD (non-B, 04d8:0053)");
MODULE_LICENSE("GPL");
