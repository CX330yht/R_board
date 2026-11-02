#include "wireless_tx.h"
#include "nrf24.h"
#include "radio_protocol.h"
#include "remote_command.h"
volatile WirelessTxDebug_t g_wireless_tx;
static uint32_t s_last_send;
void WirelessTx_Init(void)
{
    g_wireless_tx=(WirelessTxDebug_t){0};
    s_last_send=g_wireless_tx.last_ack_ms=HAL_GetTick();
}
void WirelessTx_Process(uint32_t now)
{
    int tx=NRF24_PollTx();
    if(tx==1) {
        g_wireless_tx.last_ack_ms=now;
        g_wireless_tx.link_ok=1;
    } else if(tx<0) g_wireless_tx.failed_count++;
    if((uint32_t)(now-g_wireless_tx.last_ack_ms)>RADIO_TIMEOUT_MS ||
       !g_nrf24_debug.ready) {
        g_wireless_tx.link_ok=0;
        /* Never automatically restore enabled on the first successful ACK. */
        g_remote_command.enabled=0;
       }
    if((uint32_t)(now-s_last_send)<RADIO_PERIOD_MS || tx==0) return;
    s_last_send=now;
    RadioPacket_t p={0};
    p.sequence=++g_wireless_tx.sequence;
    p.arm_token=g_remote_command.arm_token;
    p.flags=(g_remote_command.enabled?RADIO_FLAG_ENABLED:0U)|
            (g_remote_command.estop_latched?RADIO_FLAG_ESTOP:0U)|
            (g_remote_command.input_fresh?RADIO_FLAG_FRESH:0U)|
            (g_remote_command.arm_auto_active?RADIO_FLAG_ARM_AUTO:0U)|
            (g_remote_command.arm_preset_64?RADIO_FLAG_ARM_64:0U)|
            (g_remote_input.sw12_down?RADIO_FLAG_SW12_DOWN:0U);
    p.keys=g_remote_input.keys_down;
    p.gear=g_remote_command.gear;
    for(unsigned i=0;i<4;i++) {
        p.axis[i]=g_remote_input.axis[i];
        p.servo_ddeg[i]=(uint16_t)(g_remote_command.servo_target_deg[i]*10.0f+0.5f);
    }
    /* A corrupt local sample is transmitted as disabled, never as motion. */
    if(g_remote_input.estop_raw || !g_remote_input.valid) {
        p.flags&=(uint8_t)~RADIO_FLAG_ENABLED;
        if(g_remote_input.estop_raw) p.flags|=RADIO_FLAG_ESTOP;
    }
    uint8_t payload[32]; RadioEncode(payload,&p);
    if(NRF24_StartTx(payload)==HAL_OK) g_wireless_tx.sent_count++;
    else g_wireless_tx.failed_count++;
}