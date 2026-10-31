#ifndef WIRELESS_TX_H
#define WIRELESS_TX_H
#include <stdint.h>
typedef struct {
    uint32_t sequence,last_ack_ms,sent_count,failed_count;
    uint8_t link_ok;
} WirelessTxDebug_t;
extern volatile WirelessTxDebug_t g_wireless_tx;
void WirelessTx_Init(void);
void WirelessTx_Process(uint32_t now);
#endif