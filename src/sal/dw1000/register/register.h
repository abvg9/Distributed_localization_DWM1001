#ifndef _REGISTER_H_
#define _REGISTER_H_

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>

#define MAX_REG_ID_VALUE 0x3F
#define REG_WRITE_FLAG   0X80

/* Enumerate of the dwm1000 registers. */
typedef enum {
    DEV_ID      = 0x00,
    EUI         = 0x01,
    RESERVED_1  = 0X02,
    PANADR      = 0x3,
    SYS_CFG     = 0x04,
    RESERVED_2  = 0X05,
    SYS_TIME    = 0x06,
    RESERVED_3  = 0X07,
    TX_FCTRL    = 0x08,
    TX_BUFFER   = 0x09,
    DX_TIME     = 0x0A,
    RESERVED_4  = 0X0B,
    RX_FWTO     = 0X0C,
    SYS_CTRL    = 0X0D,
    SYS_MASK    = 0X0E,
    SYS_STATUS  = 0X0F,
    RX_FINFO    = 0x10,
    RX_BUFFER   = 0X11,
    RX_FQUAL    = 0X12,
    RX_TTCKI    = 0X13,
    RX_TTCKO    = 0X14,
    RX_TIME     = 0X15,
    RESERVED_5  = 0X06,
    TX_TIME     = 0X17,
    TX_ANTD     = 0X18,
    SYS_STATE   = 0X19,
    ACK_RESP_T  = 0X1A,
    RESERVED_6  = 0X1B,
    RESERVED_7  = 0X1C,
    RX_SNIFF    = 0X1D,
    TX_POWER    = 0X1E,
    CHAN_CTRL   = 0X1F,
    USR_SFD     = 0X21,
    RESERVED_8  = 0X22,
    AGC_CTRL    = 0X23,
    EXT_SYNC    = 0X24,
    ACC_MEM     = 0X25,
    GPIO_CTRL   = 0X26,
    RESERVED_9  = 0X27,
    DRX_CONF    = 0X28,
    TX_CAL      = 0X2A,
    FS_CTRL     = 0X2B,
    AON         = 0X2C,
    OTP_IF      = 0X2D,
    LDE_CTRL    = 0X2E,
    DIG_DIAG    = 0X2F,
    RESERVED_10 = 0X30,
    RESERVED_11 = 0X31,
    RESERVED_12 = 0X32,
    RESERVED_13 = 0X33,
    RESERVED_14 = 0X34,
    RESERVED_15 = 0X35,
    PMSC        = 0X36,
    RESERVED_16 = 0X37,
    RESERVED_17 = 0X38,
    RESERVED_18 = 0X39,
    RESERVED_19 = 0X3A,
    RESERVED_20 = 0X3B,
    RESERVED_21 = 0X3C,
    RESERVED_22 = 0X3D,
    RESERVED_23 = 0X3E,
    RESERVED_24 = 0X3F
} register_id;

/* Access permissions of the dwm1000 registers. */
typedef enum {
    RO, // Read only.
    WO, // Write only.
    RW, // Read and write.
    RE  // Reserved.
} register_access;

/* General command structure to interact with dwm1000 registers. */
typedef struct {
    const size_t tx_buf_size;
    const size_t rx_buf_size;
    const register_access ra : 2;
    const int offset;
} command;

/* Constant ID value of the dwm1000. */
extern const uint8_t DEVICE_ID[4];

bool is_access_permission_valid(const register_id id, const bool read);

bool is_command_valid(const register_id id, const bool read);

bool is_id_valid(const register_id id);

#endif // _REGISTER_H_
