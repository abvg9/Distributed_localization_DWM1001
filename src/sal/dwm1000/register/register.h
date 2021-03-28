#ifndef _REGISTER_H_
#define _REGISTER_H_

#include <stdint.h>
#include <stdlib.h>

/* Enumerate of the dwm1000 registers. */
typedef enum {
    DEV_ID     = 0x00,
    EUI        = 0x01,
    PANADR     = 0x3,
    SYS_CFG    = 0x04,
    SYS_TIME   = 0x06,
    TX_FCTRL   = 0x08,
    TX_BUFFER  = 0x09,
    DX_TIME    = 0x0A,
    RX_FWTO    = 0X0C,
    SYS_CTRL   = 0X0D,
    SYS_MASK   = 0X0E,
    SYS_STATUS = 0X0F,
    RX_FINFO   = 0x10,
    RX_BUFFER  = 0X11,
    RX_FQUAL   = 0X12,
    RX_TTCKI   = 0X13,
    RX_TTCKO   = 0X14,
    RX_TIME    = 0X15,
    TX_TIME    = 0X17,
    TX_ANTD    = 0X18,
    SYS_STATE  = 0X19,
    ACK_RESP_T = 0X1A,
    RX_SNIFF   = 0X1D,
    TX_POWER   = 0X1E,
    CHAN_CTRL  = 0X1F,
    USR_SFD    = 0X21,
    AGC_CTRL   = 0X23,
    EXT_SYNC   = 0X24,
    ACC_MEM    = 0X25,
    GPIO_CTRL  = 0X26,
    DRX_CONF   = 0X28,
    TX_CAL     = 0X2A,
    FS_CTRL    = 0X2B,
    AON        = 0X2C,
    OTP_IF     = 0X2D,
    LDE_CTRL   = 0X2E,
    DIG_DIAG   = 0X2F,
    PMSC       = 0X36
} register_id;

typedef struct {
    const register_id id;
    const size_t tx_buf_size;
    const size_t rx_buf_size;
} command;

/* Constant ID value of the dwm1000. */
extern const uint8_t DEVICE_ID[4];

/* COMMANDS */
extern const command DEV_ID_COMMAND;

#endif // _REGISTER_H_
