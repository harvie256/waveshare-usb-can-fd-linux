// Minimal MCP2518FD driver (polling) for the LilyGo T-2CAN-FD "CAN A" port.
// Register map: Microchip DS20006027 (MCP2518FD).
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "mcp2518fd.h"

#define PIN_SCLK 12
#define PIN_MOSI 11
#define PIN_MISO 13
#define PIN_CS   10

// SFRs
#define C1CON     0x000
#define C1NBTCFG  0x004
#define C1DBTCFG  0x008
#define C1TDC     0x00C
#define C1TBC     0x010
#define C1TSCON   0x014
#define C1INT     0x01C
#define C1TREC    0x034
#define C1BDIAG0  0x038
#define C1BDIAG1  0x03C
#define C1TXQCON  0x050
#define FIFOCON(n) (0x050 + 12 * (n))
#define FIFOSTA(n) (0x054 + 12 * (n))
#define FIFOUA(n)  (0x058 + 12 * (n))
#define C1FLTCON0 0x1D0
#define C1FLTOBJ0 0x1F0
#define C1MASK0   0x1F4
#define OSC       0xE00
#define IOCON     0xE04
#define RAM_START 0x400

#define TXFIFO 1
#define RXFIFO 2

static const char *TAG = "mcp2518fd";
static spi_device_handle_t s_spi;
static uint32_t s_sysclk = 40000000;   // T-2CAN-FD has a 40 MHz crystal (verified)

// ------------------------------------------------------------------ SPI

static void xfer(const uint8_t *tx, uint8_t *rx, size_t n)
{
    spi_transaction_t t = { .length = n * 8, .tx_buffer = tx, .rx_buffer = rx };
    ESP_ERROR_CHECK(spi_device_polling_transmit(s_spi, &t));
}

static void spi_reset(void)
{
    uint8_t c[2] = { 0, 0 };
    xfer(c, NULL, 2);
}

static void rd(uint16_t addr, uint8_t *buf, size_t n)
{
    uint8_t tx[2 + 80] = { 0x30 | (addr >> 8), addr & 0xFF }, rx[2 + 80];
    xfer(tx, rx, 2 + n);
    memcpy(buf, rx + 2, n);
}

static void wr(uint16_t addr, const uint8_t *buf, size_t n)
{
    uint8_t tx[2 + 80] = { 0x20 | (addr >> 8), addr & 0xFF };
    memcpy(tx + 2, buf, n);
    xfer(tx, NULL, 2 + n);
}

static uint32_t rd32(uint16_t a)
{
    uint8_t b[4];
    rd(a, b, 4);
    return b[0] | b[1] << 8 | b[2] << 16 | (uint32_t)b[3] << 24;
}

static void wr32(uint16_t a, uint32_t v)
{
    uint8_t b[4] = { v, v >> 8, v >> 16, v >> 24 };
    wr(a, b, 4);
}

static void wr8(uint16_t a, uint8_t v) { wr(a, &v, 1); }

// ------------------------------------------------------------------ timing

static int calc(uint32_t clk, uint32_t rate, int max_brp, int max_t1, int max_t2,
                int *brp, int *t1, int *t2)
{
    for (int b = 1; b <= max_brp; b++) {
        if (clk % (b * rate)) continue;
        int ntq = clk / (b * rate);
        if (ntq < 4 || ntq > 1 + max_t1 + max_t2) continue;
        int s1 = (ntq * 8 + 5) / 10 - 1;        // ~80% sample point
        int s2 = ntq - 1 - s1;
        if (s2 > max_t2) { s2 = max_t2; s1 = ntq - 1 - s2; }
        if (s1 < 1 || s1 > max_t1 || s2 < 1) continue;
        *brp = b; *t1 = s1; *t2 = s2;
        return 0;
    }
    return -1;
}

static int set_mode(uint32_t reqop)
{
    uint32_t con = rd32(C1CON);
    con = (con & ~(7u << 24)) | (reqop << 24);
    wr32(C1CON, con);
    for (int i = 0; i < 100; i++) {
        if (((rd32(C1CON) >> 21) & 7) == reqop) return 0;
        vTaskDelay(1);
    }
    ESP_LOGE(TAG, "mode %lu not reached (C1CON=%08lx)", reqop, rd32(C1CON));
    return -1;
}

// ------------------------------------------------------------------ API

int mcp_init(uint32_t nominal, uint32_t data, bool listen_only)
{
    if (!s_spi) {
        spi_bus_config_t bus = { .sclk_io_num = PIN_SCLK, .mosi_io_num = PIN_MOSI,
                                 .miso_io_num = PIN_MISO, .quadwp_io_num = -1, .quadhd_io_num = -1,
                                 .max_transfer_sz = 128 };
        ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_CH_AUTO));
        spi_device_interface_config_t dev = { .clock_speed_hz = 4000000, .mode = 0,
                                              .spics_io_num = PIN_CS, .queue_size = 1 };
        ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &dev, &s_spi));
    }
    spi_reset();
    vTaskDelay(pdMS_TO_TICKS(5));
    uint32_t osc = rd32(OSC);
    for (int i = 0; i < 50 && !(osc & (1 << 10)); i++) { vTaskDelay(1); osc = rd32(OSC); }
    uint32_t con = rd32(C1CON);
    ESP_LOGI(TAG, "OSC=%08lx C1CON=%08lx (expect opmode 4)", osc, con);
    if (((con >> 21) & 7) != 4) { ESP_LOGE(TAG, "not in config mode -- SPI/chip problem"); return -1; }

    int nb, n1, n2, db, d1, d2;
    if (calc(s_sysclk, nominal, 256, 256, 128, &nb, &n1, &n2)) { ESP_LOGE(TAG, "no nominal timing"); return -2; }
    if (calc(s_sysclk, data, 256, 32, 16, &db, &d1, &d2)) { ESP_LOGE(TAG, "no data timing"); return -3; }
    wr32(C1NBTCFG, (nb - 1) << 24 | (n1 - 1) << 16 | (n2 - 1) << 8 | (n2 - 1));
    wr32(C1DBTCFG, (db - 1) << 24 | (d1 - 1) << 16 | (d2 - 1) << 8 | (d2 - 1));
    // Transmitter delay compensation: auto, offset = sample point in SYSCLKs
    uint32_t tdco = db * (d1 + 1);
    if (tdco > 63) tdco = 63;
    wr32(C1TDC, (2u << 16) | (tdco << 8));
    ESP_LOGI(TAG, "sysclk %lu: nominal %lu brp=%d t1=%d t2=%d; data %lu brp=%d t1=%d t2=%d tdco=%lu",
             s_sysclk, nominal, nb, n1, n2, data, db, d1, d2, tdco);

    // Time base for RX timestamps (1 us), enabled
    wr32(C1TSCON, (1u << 16) | (s_sysclk / 1000000 - 1));

    // C1CON: no TXQ, no TEF, ISO CRC, protocol exception -> bus integration, keep REQOP=config
    con = rd32(C1CON);
    con &= ~((1u << 20) | (1u << 19));   // TXQEN=0, STEF=0
    con |= (1u << 5);                    // ISOCRCEN
    con |= (1u << 16);                   // RTXAT: honour FIFO TXAT
    wr32(C1CON, con);

    // FIFO1 = TX, 8 x 64 B, unlimited retransmissions; FIFO2 = RX, 16 x 64 B, timestamps
    wr32(FIFOCON(TXFIFO), (7u << 29) | (7u << 24) | (3u << 21) | (1u << 7));
    wr32(FIFOCON(RXFIFO), (7u << 29) | (15u << 24) | (1u << 5));

    // Filter 0: accept everything into FIFO2
    wr32(C1FLTOBJ0, 0);
    wr32(C1MASK0, 0);
    wr8(C1FLTCON0, 0x80 | RXFIFO);

    return set_mode(listen_only ? 3 : 0);    // 0 = normal CAN FD mode, 3 = listen only
}

void mcp_set_sysclk(uint32_t hz) { s_sysclk = hz; }
uint32_t mcp_get_sysclk(void) { return s_sysclk; }

static const uint8_t DLC2LEN[16] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64 };

static uint8_t len2dlc(uint8_t n)
{
    for (uint8_t d = 0; d < 16; d++) if (DLC2LEN[d] >= n) return d;
    return 15;
}

int mcp_send(const mcp_frame_t *f)
{
    if (!(rd32(FIFOSTA(TXFIFO)) & 1)) return -1;       // TFNRFNIF: not full
    uint16_t ua = rd32(FIFOUA(TXFIFO)) & 0xFFF;
    uint8_t obj[8 + 64] = { 0 };
    uint32_t t0 = f->ext ? ((f->id >> 18) & 0x7FF) | ((f->id & 0x3FFFF) << 11) : (f->id & 0x7FF);
    uint8_t dlc = f->fd ? len2dlc(f->len) : (f->len > 8 ? 8 : f->len);
    uint32_t t1 = dlc | (f->ext << 4) | (f->rtr << 5) | (f->brs << 6) | (f->fd << 7);
    memcpy(obj, &t0, 4);
    memcpy(obj + 4, &t1, 4);
    uint8_t n = f->fd ? DLC2LEN[dlc] : dlc;
    memcpy(obj + 8, f->data, f->len < n ? f->len : n);
    wr(RAM_START + ua, obj, 8 + ((n + 3) & ~3));
    wr8(FIFOCON(TXFIFO) + 1, 0x03);                      // UINC | TXREQ
    return 0;
}

int mcp_recv(mcp_frame_t *f)
{
    if (!(rd32(FIFOSTA(RXFIFO)) & 1)) return 0;         // TFNRFNIF: not empty
    uint16_t ua = rd32(FIFOUA(RXFIFO)) & 0xFFF;
    uint8_t obj[12 + 64];
    rd(RAM_START + ua, obj, 12);
    uint32_t r0, r1;
    memcpy(&r0, obj, 4);
    memcpy(&r1, obj + 4, 4);
    memcpy(&f->ts, obj + 8, 4);
    f->ext = (r1 >> 4) & 1;
    f->rtr = (r1 >> 5) & 1;
    f->brs = (r1 >> 6) & 1;
    f->fd = (r1 >> 7) & 1;
    f->esi = (r1 >> 8) & 1;
    f->id = f->ext ? ((r0 & 0x7FF) << 18) | ((r0 >> 11) & 0x3FFFF) : (r0 & 0x7FF);
    uint8_t dlc = r1 & 0xF;
    f->len = f->fd ? DLC2LEN[dlc] : (dlc > 8 ? 8 : dlc);
    if (f->len && !f->rtr) rd(RAM_START + ua + 12, f->data, (f->len + 3) & ~3);
    wr8(FIFOCON(RXFIFO) + 1, 0x01);                      // UINC
    return 1;
}

void mcp_status(uint32_t *trec, uint32_t *bdiag1, uint32_t *intf)
{
    *trec = rd32(C1TREC);
    *bdiag1 = rd32(C1BDIAG1);
    *intf = rd32(C1INT);
}
