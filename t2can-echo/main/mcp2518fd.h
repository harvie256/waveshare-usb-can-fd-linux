#pragma once
#include <stdint.h>
#include <stdbool.h>

typedef struct {
    uint32_t id;
    uint8_t len;            // payload bytes (0..8 classic, 0..64 FD)
    bool ext, rtr, fd, brs, esi;
    uint32_t ts;            // RX timestamp, us
    uint8_t data[64];
} mcp_frame_t;

int mcp_init(uint32_t nominal, uint32_t data, bool listen_only);
void mcp_set_sysclk(uint32_t hz);
uint32_t mcp_get_sysclk(void);
int mcp_send(const mcp_frame_t *f);     // 0 ok, -1 TX FIFO full
int mcp_recv(mcp_frame_t *f);           // 1 = frame, 0 = none
void mcp_status(uint32_t *trec, uint32_t *bdiag1, uint32_t *intf);
