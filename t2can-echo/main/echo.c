// T-2CAN-FD bench peer for USB-CAN-FD driver work.
//
// CAN A (MCP2518FD, CAN FD): echoes every frame with ID+1 and data XOR 0xFF,
//   keeping length/FD/BRS/IDE. Optional 1 Hz heartbeat (ID 0x7E0).
// CAN B (ESP32 TWAI, classic, 250k): echoes the same way (no heartbeat).
//
// Serial commands (newline terminated) on the USB-Serial-JTAG console:
//   b <nominal> <data>   re-init CAN A bit rates      e.g. "b 500000 2000000"
//   c <hz>               MCP2518FD SYSCLK used for timing calc (default 40000000)
//   l <0|1>              CAN A listen-only
//   h <0|1>              CAN A heartbeat
//   e <0|1>              echo on/off (both ports)
//   q <0|1>              quiet: don't print frames (throughput tests)
//   s                    print status now
// Replies start with "OK" or "ERR".
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/twai.h"
#include "driver/usb_serial_jtag.h"
#include "driver/usb_serial_jtag_vfs.h"
#include "esp_timer.h"
#include "mcp2518fd.h"

static volatile bool s_echo = true, s_quiet = false, s_hb = true, s_listen = false;
static volatile uint32_t s_nominal = 250000, s_data = 2000000;
static volatile uint32_t s_a_rx, s_a_tx, s_a_txfull, s_b_rx, s_b_tx;
static SemaphoreHandle_t s_mcp_lock;

static void print_mcp(const char *dir, const mcp_frame_t *f)
{
    if (s_quiet) return;
    char line[300];
    int n = snprintf(line, sizeof line, "A%s %lld id=%s%08lX len=%u%s%s%s data=", dir, esp_timer_get_time(),
                     f->ext ? "X" : "S", (unsigned long)f->id, f->len, f->fd ? " FD" : "",
                     f->brs ? " BRS" : "", f->rtr ? " RTR" : "");
    for (int i = 0; i < f->len && !f->rtr && n < (int)sizeof line - 4; i++)
        n += snprintf(line + n, sizeof line - n, "%02X", f->data[i]);
    puts(line);
}

static void print_twai(const char *dir, const twai_message_t *m)
{
    if (s_quiet) return;
    printf("B%s %lld id=%s%08lX len=%u%s data=", dir, esp_timer_get_time(),
           m->extd ? "X" : "S", (unsigned long)m->identifier, m->data_length_code, m->rtr ? " RTR" : "");
    for (int i = 0; i < m->data_length_code && !m->rtr; i++) printf("%02X", m->data[i]);
    printf("\n");
}

static void mcp_task(void *arg)
{
    mcp_frame_t f;
    int64_t next_hb = esp_timer_get_time() + 1000000;
    uint32_t hb = 0;
    for (;;) {
        xSemaphoreTake(s_mcp_lock, portMAX_DELAY);
        int got = mcp_recv(&f);
        if (got) {
            s_a_rx++;
            print_mcp("RX", &f);
            if (s_echo && !f.rtr && !s_listen) {
                f.id = (f.id + 1) & (f.ext ? 0x1FFFFFFF : 0x7FF);
                for (int i = 0; i < f.len; i++) f.data[i] ^= 0xFF;
                int tries = 0;
                while (mcp_send(&f) != 0 && ++tries < 200) esp_rom_delay_us(50);
                if (tries < 200) { s_a_tx++; print_mcp("TX", &f); } else s_a_txfull++;
            }
        }
        if (s_hb && !s_listen && esp_timer_get_time() >= next_hb) {
            next_hb += 1000000;
            mcp_frame_t h = { .id = 0x7E0, .len = 4 };
            h.data[0] = hb >> 24; h.data[1] = hb >> 16; h.data[2] = hb >> 8; h.data[3] = hb;
            hb++;
            if (mcp_send(&h) == 0) print_mcp("HB", &h);
        }
        xSemaphoreGive(s_mcp_lock);
        if (!got) vTaskDelay(1);
    }
}

static void twai_task(void *arg)
{
    for (;;) {
        twai_message_t m;
        if (twai_receive(&m, portMAX_DELAY) != ESP_OK) continue;
        s_b_rx++;
        print_twai("RX", &m);
        if (!s_echo || m.rtr) continue;
        m.identifier = (m.identifier + 1) & (m.extd ? 0x1FFFFFFF : 0x7FF);
        for (int i = 0; i < m.data_length_code; i++) m.data[i] ^= 0xFF;
        if (twai_transmit(&m, pdMS_TO_TICKS(50)) == ESP_OK) { s_b_tx++; print_twai("TX", &m); }
    }
}

static void status(void)
{
    uint32_t trec, bdiag1, intf;
    xSemaphoreTake(s_mcp_lock, portMAX_DELAY);
    mcp_status(&trec, &bdiag1, &intf);
    xSemaphoreGive(s_mcp_lock);
    twai_status_info_t s;
    twai_get_status_info(&s);
    printf("STATUS A: %lu/%lu sysclk=%lu listen=%d rx=%lu tx=%lu txfull=%lu tec=%lu rec=%lu trec=%08lx bdiag1=%08lx int=%08lx"
           " | B: rx=%lu tx=%lu state=%d tec=%lu rec=%lu missed=%lu\n",
           s_nominal, s_data, mcp_get_sysclk(), s_listen, s_a_rx, s_a_tx, s_a_txfull,
           (trec >> 8) & 0xFF, trec & 0xFF, trec, bdiag1, intf,
           s_b_rx, s_b_tx, s.state, s.tx_error_counter, s.rx_error_counter, s.rx_missed_count);
}

static void status_task(void *arg)
{
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(5000));
        if (!s_quiet) status();
    }
}

static void reinit(void)
{
    xSemaphoreTake(s_mcp_lock, portMAX_DELAY);
    int r = mcp_init(s_nominal, s_data, s_listen);
    xSemaphoreGive(s_mcp_lock);
    printf("%s mcp_init(%lu, %lu, listen=%d) = %d\n", r ? "ERR" : "OK", s_nominal, s_data, s_listen, r);
}

static void console_task(void *arg)
{
    char line[96];
    for (;;) {
        if (!fgets(line, sizeof line, stdin)) { vTaskDelay(10); continue; }
        char c = line[0];
        char *p = line + 1;
        unsigned long a = strtoul(p, &p, 10), b = strtoul(p, &p, 10);
        switch (c) {
        case 'b': s_nominal = a; if (b) s_data = b; reinit(); break;
        case 'c': mcp_set_sysclk(a); printf("OK sysclk %lu\n", a); break;
        case 'l': s_listen = a; reinit(); break;
        case 'h': s_hb = a; printf("OK hb %lu\n", a); break;
        case 'e': s_echo = a; printf("OK echo %lu\n", a); break;
        case 'q': s_quiet = a; printf("OK quiet %lu\n", a); break;
        case 's': status(); break;
        case 'z': s_a_rx = s_a_tx = s_a_txfull = s_b_rx = s_b_tx = 0; printf("OK zeroed\n"); break;
        case '\n': case '\r': break;
        default: printf("ERR unknown '%c'\n", c);
        }
        fflush(stdout);
    }
}

void app_main(void)
{
    usb_serial_jtag_driver_config_t ucfg = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
    ucfg.rx_buffer_size = 1024; ucfg.tx_buffer_size = 4096;
    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&ucfg));
    usb_serial_jtag_vfs_use_driver();
    setvbuf(stdin, NULL, _IONBF, 0);

    s_mcp_lock = xSemaphoreCreateMutex();

    twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(GPIO_NUM_7, GPIO_NUM_6, TWAI_MODE_NORMAL);
    g.tx_queue_len = 32;
    g.rx_queue_len = 64;
    twai_timing_config_t t = TWAI_TIMING_CONFIG_250KBITS();
    twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    ESP_ERROR_CHECK(twai_driver_install(&g, &t, &f));
    ESP_ERROR_CHECK(twai_start());

    reinit();
    printf("ECHO READY: A=MCP2518FD %lu/%lu, B=TWAI 250000\n", s_nominal, s_data);
    xTaskCreatePinnedToCore(mcp_task, "mcp", 4096, NULL, 10, NULL, 1);
    xTaskCreatePinnedToCore(twai_task, "twai", 4096, NULL, 9, NULL, 0);
    xTaskCreate(status_task, "status", 4096, NULL, 3, NULL);
    xTaskCreate(console_task, "console", 4096, NULL, 4, NULL);
}
