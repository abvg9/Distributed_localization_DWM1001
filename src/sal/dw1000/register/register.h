/*
 * This file is part of the UCM-237 distribution (https://github.com/UCM-237/Distributed_localization_DWM1001).
 * Copyright (c) 2021 Complutense university of Madrid, Madrid, Spain.
 *
 * Author: Alvaro Velasco Garcia. <https://github.com/abvg9>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 2.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 */

#ifndef _REGISTER_H_
#define _REGISTER_H_

#include <stdlib.h>
#include <stdbool.h>
#include "format.h"
#include "hal.h"
#include <string.h>

// Dwm1000 registers.
// The values of the enumerates correspond to the memory addresses of the registers.
typedef enum {
    DEV_ID      = 0x00,
    EUI         = 0x01,
    RESERVED_1  = 0X02,
    PAN_ADR     = 0x03,
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

// Factory value of the dev_id register.
extern const dev_id_format DEFINED_DEV_ID;

/**
 * @brief Reads the value of a dw1000 register and loads it inside parsed_reg parameter.
 *
 * @param[in] reg_id: Register identifier to read.
 * @param[in] offset: Indicates which sub-register will be read.
 * @param[out] parsed_reg: Container in which will be loaded the value of the register.
 *
 * @return bool: If the read operation can be performed returns true, otherwise false.
 *
 */
bool dw_read_reg(const register_id reg_id, const size_t offset, void* parsed_reg);

/**
 * @brief Writes a value into a dw1000 register.
 *
 * @param[in] reg_id: Register identifier to write.
 * @param[in] offset: Indicates which sub-register will be write.
 * @param[in] parsed_reg: Container which contains the value to be write.
 *
 * @return bool: If the write operation can be performed returns true, otherwise false.
 *
 */
bool dw_write_reg(const register_id reg_id, const size_t offset, void* parsed_reg);

#endif // _REGISTER_H_
