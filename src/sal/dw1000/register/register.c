/**
 * This file is part of the abvg9 distribution (https://github.com/abvg9/Distributed_localization_DWM1001).
 * Copyright (c) 2021 Álvaro Velsco García, Madrid, Spain.
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

#include "register.h"
#include "format.h"

#define MAX_REG_ID_VALUE 0x3F
#define REG_WRITE_FLAG   0X80
const dev_id_format DEFINED_DEV_ID = { .ridtag = 0XDECA, .model = 0X01, .ver = 0X03, .rev = 0x00};

// Access permissions of the dwm1000 registers.
typedef enum {
    RO, // Read only.
    WO, // Write only.
    RW, // Read and write.
    RE  // Reserved.
} register_access;

// General command structure to interact with dwm1000 registers.
typedef struct {
    const size_t rx_buf_size;                                                      // Size of the received buffer.
    const register_access ra;                                                      // Register access permission.
    void (*formatter)(spi_frame f, void* structure, const size_t sub_register);     // Pointer to the function that formats the spi_frame.
    size_t (*unformatter)(void* structure, spi_frame f, const size_t sub_register); // Pointer to the function that unformats the spi_frame.
} command;

#define RESERVED_REGISTER {0U, RE, NULL, NULL}

// Structure which contains all the registers information of the dw1000.
const command COMMAND_MAP[] = {
        {4U, RO, dev_id_formatter, NULL},                         // DEV_ID
        {8U, RW, eui_formatter, eui_unformatter},                 // EUI
        RESERVED_REGISTER,                                        // RESERVED_1
        {4U, RW, pan_addr_formatter, pan_addr_unformatter},       // PAN_ADR
        {4U, RW, sys_cfg_formatter, sys_cfg_unformatter},         // SYS_CFG
        RESERVED_REGISTER,                                        // RESERVED_2
        {5U, RO, sys_time_formatter, NULL},                       // SYS_TIME
        RESERVED_REGISTER,                                        // RESERVED_3
        {4U, RW, tx_fctrl_formatter, tx_fctrl_unformatter},       // TX_FCTRL
        {TX_RX_BUFFER_MAX_SIZE, WO, NULL, tx_buffer_unformatter}, // TX_BUFFER
        {5U, RW, dx_time_formatter, dx_time_unformatter},         // DX_TIME
        RESERVED_REGISTER,                                        // RESERVED_4
        {2U, RW, rx_fwto_formatter, rx_fwto_unformatter},         // RX_FWTO
        {4U, RW, tx_ctrl_formatter, tx_ctrl_unformatter},         // SYS_CTRL
        {4U, RW, sys_evt_msk_formatter, sys_evt_msk_unformatter}, // SYS_MASK
        {4U, RW, sys_evt_sts_formatter, sys_evt_sts_unformatter}, // SYS_STATUS
        {4U, RO, rx_finfo_formatter, NULL},                       // RX_FINFO
        {TX_RX_BUFFER_MAX_SIZE, RO, rx_buffer_formatter, NULL},   // RX_BUFFER
        {4U, RO, rx_fqual_formatter, NULL},                       // RX_FQUAL
        {4U, RO, rx_ttcki_formatter, NULL},                       // RX_TTCKI
        {4U, RO, rx_ttcko_formatter, NULL},                       // RX_TTCKO
        {4U, RO, rx_time_formatter, NULL},                        // RX_TIME
        RESERVED_REGISTER,                                        // RESERVED_5
        {4U, RO, tx_time_formatter, NULL},                        // TX_TIME
        {2U, RW, tx_antd_formatter, tx_antd_unformatter},         // TX_ANTD
        {4U, RO, sys_state_formatter, NULL},                      // SYS_STATE
        {4U, RW, ack_resp_t_formatter, ack_resp_t_unformatter},   // ACK_RESP_T
        RESERVED_REGISTER,                                        // RESERVED_6
        RESERVED_REGISTER,                                        // RESERVED_7
        {2U, RW, rx_sniff_formatter, rx_sniff_unformatter},       // RX_SNIFF
        {4U, RW, tx_power_formatter, tx_power_unformatter},       // TX_POWER
        {4U, RW, chan_ctrl_formatter, chan_ctrl_unformatter},     // CHAN_CTRL
        RESERVED_REGISTER,                                        // RESERVED_8
        {4U, RW, usr_sfd_formatter, usr_sfd_unformatter},         // USR_SFD
        RESERVED_REGISTER,                                        // RESERVED_9
        {4U, RW, agc_ctrl_formatter, agc_ctrl_unformatter},       // AGC_CTRL
        {4U, RW, ext_sync_formatter, ext_sync_unformatter},       // EXT_SYNC
        {4U, RW, acc_mem_formatter, NULL},                        // ACC_MEM
        {4U, RW, gpio_ctrl_formatter, gpio_ctrl_unformatter},     // GPIO_CTRL
        {4U, RW, drx_conf_formatter, drx_conf_unformatter},       // DRX_CONF
        {5U, RW, rf_conf_formatter, rf_conf_unformatter},         // RF_CONF
        RESERVED_REGISTER,                                        // RESERVED_10
        {3U, RW, tx_cal_formatter, tx_cal_unformatter},           // TX_CAL
        {4U, RW, fs_ctrl_formatter, fs_ctrl_unformatter},         // FS_CTRL
        {4U, RW, aon_formatter, aon_unformatter},                 // AON
        {4U, RW, otp_if_formatter, otp_if_unformatter},           // OTP_IF
        {2U, RW, lde_if_formatter, lde_if_unformatter},           // LDE_IF
        {4U, RW, dig_diag_formatter, dig_diag_unformatter},       // DIG_DIAG
        RESERVED_REGISTER,                                        // RESERVED_11
        RESERVED_REGISTER,                                        // RESERVED_12
        RESERVED_REGISTER,                                        // RESERVED_13
        RESERVED_REGISTER,                                        // RESERVED_14
        RESERVED_REGISTER,                                        // RESERVED_15
        RESERVED_REGISTER,                                        // RESERVED_16
        {4U, RW, pmsc_formatter, pmsc_unformatter},               // PMSC
        RESERVED_REGISTER,                                        // RESERVED_17
        RESERVED_REGISTER,                                        // RESERVED_18
        RESERVED_REGISTER,                                        // RESERVED_19
        RESERVED_REGISTER,                                        // RESERVED_20
        RESERVED_REGISTER,                                        // RESERVED_21
        RESERVED_REGISTER,                                        // RESERVED_22
        RESERVED_REGISTER,                                        // RESERVED_23
        RESERVED_REGISTER,                                        // RESERVED_24
        RESERVED_REGISTER,                                        // RESERVED_25
};

/**
 * @brief Checks if the read/write operation can be done in the register.
 *
 * @param[in] reg_id: Register identifier.
 * @param[in] read: Type of operation. True -> read, False -> write.
 *
 * @return bool: If operation is valid, returns true, otherwise false.
 *
 */
bool _is_access_permission_valid(const register_id id, const bool read) {

    switch(COMMAND_MAP[id].ra) {
        case RO:
            return read;
        case WO:
            return !read;
        case RW:
            return true;
        default:
            return false;
    }
}

/**
 * @brief Checks if the register identification provided is valid.
 *
 * @param[in] reg_id: Register identifier.
 *
 * @return bool: If register_id is valid returns true, otherwise false.
 *
 */
bool _is_id_valid(const register_id id) {
    return id <= MAX_REG_ID_VALUE;
}

/**
 * @brief Checks if command is valid.
 *
 * @param[in] reg_id: Register identifier.
 * @param[in] read: Type of operation. True -> read, False -> write.
 *
 * @return bool: If command is valid, returns true, otherwise false.
 *
 */
bool _is_command_valid(const register_id id, const bool read) {
    return _is_id_valid(id) && _is_access_permission_valid(id, read);
}

/**
 * @brief Prepare SPI header frame to be send.
 *
 * @param[in] reg_id: Register identifier.
 * @param[in] offset: Sub-register index.
 * @param[in] read: If the operation will be a read operation.
 * @param[out] header: SPI header frame formed.
 * @param[in] disable_permission_sec: Disables the mechanism to protect the invalid registers access.
 *
 * @return size_t: Size of the SPI header frame. If this function returns
 *                 zero that means that the given parameters was invalid.
 *
 */
size_t _compose_spi_header(const register_id reg_id, size_t offset, const bool read, spi_frame header, const bool disable_permission_sec) {

    size_t spi_header_size = 0;

    if(disable_permission_sec || _is_command_valid(reg_id, read)) {

        spi_header_size = 1;

        if(offset == 0) {

            if(!read) {
                header[0] = REG_WRITE_FLAG | (uint8_t) reg_id;
            } else {
                header[0] =  (uint8_t) reg_id;
            }

        } else {

            if(!read) {
                header[0] = (uint8_t)(0XC0 | reg_id);
            } else {
                header[0] = (uint8_t)(0x40 | reg_id);
            }

            if(offset <= 127) {
                spi_header_size = 2;
                header[1] = (uint8_t)offset;
            } else {
                spi_header_size = 3;
                header[1] = 0x80 | (uint8_t)(offset);
                header[2] =  (uint8_t) (offset >> 7);
            }

        }

    }

    return spi_header_size;
}

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
bool _dw_read_reg(const register_id reg_id, const size_t offset, void* parsed_reg) {

    uint8_t header[3] = {0X00, 0X00, 0X00};
    size_t spi_header_size = _compose_spi_header(reg_id, offset, true, header, false);

    if(spi_header_size > 0) {

        command c = COMMAND_MAP[reg_id];

        uint8_t rx[c.rx_buf_size];

        spiAcquireBus(&SPID1);
        spiSelect(&SPID1);

        spiSend(&SPID1, spi_header_size, header);
        spiReceive(&SPID1, c.rx_buf_size, rx);

        spiUnselect(&SPID1);
        spiReleaseBus(&SPID1);

        c.formatter(rx, parsed_reg, offset);

        return true;
    }

    return false;
}

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
bool _dw_write_reg(const register_id reg_id, const size_t offset, void* parsed_reg) {

    uint8_t header[3] = {0X00, 0X00, 0X00};
    size_t spi_header_size = _compose_spi_header(reg_id, offset, false, header, false);

    if(spi_header_size > 0) {

        command c = COMMAND_MAP[reg_id];

        uint8_t data[c.rx_buf_size];
        size_t data_size = c.unformatter(parsed_reg, data, offset);

        uint8_t message[spi_header_size+data_size];
        memcpy(message, header, spi_header_size);
        memcpy(message+spi_header_size, data, data_size);

        spiAcquireBus(&SPID1);
        spiSelect(&SPID1);

        spiSend(&SPID1, spi_header_size+data_size, message);

        spiUnselect(&SPID1);
        spiReleaseBus(&SPID1);

        return true;

    }

    return false;
}

/******* REGISTERS GETTERS AND SETTERS *******/

bool get_dev_id(dev_id_format *dev_id_f) {
    return _dw_read_reg(DEV_ID, 0, (void*) dev_id_f);
}

bool get_eui(eui_format* eui_f) {
    return _dw_read_reg(EUI, 0, (void*) eui_f);
}

bool set_eui(eui_format* eui_f) {
    return _dw_write_reg(EUI, 0, (void*) eui_f);
}

bool get_pan_adr(pan_adr_format *pan_adr_f) {
    return _dw_read_reg(PAN_ADR, 0, (void*) pan_adr_f);
}

bool set_pan_adr(pan_adr_format* pan_adr_f, const pan_adr_subregister subregister) {

    switch(subregister) {

        case PAN_ID:
            return _dw_write_reg(PAN_ADR, PAN_ID, (void*) pan_adr_f);

        case SHORT_ADR:
            return _dw_write_reg(PAN_ADR, SHORT_ADR, (void*) pan_adr_f);

        default:
            return _dw_write_reg(PAN_ADR, PAN_ID, (void*) pan_adr_f) &&
                   _dw_write_reg(PAN_ADR, SHORT_ADR, (void*) pan_adr_f);

    }

}

bool get_sys_cfg(sys_cfg_format* sys_cfg_f) {
    return _dw_read_reg(SYS_CFG, 0, (void*) sys_cfg_f);
}

bool set_sys_cfg(sys_cfg_format* sys_cfg_f) {
    return _dw_write_reg(SYS_CFG, 0, (void*) sys_cfg_f);
}

bool get_sys_time(double* seconds) {
    return _dw_read_reg(SYS_TIME, 0, (void*) seconds);
}

bool get_tx_fctrl(tx_fctrl_format* tx_fctrl_f, const tx_fctrl_subregister subregister) {

    switch(subregister) {

        case REST:
            return _dw_read_reg(TX_FCTRL, REST, (void*) tx_fctrl_f);

        case IFSDELAY:
            return _dw_read_reg(TX_FCTRL, IFSDELAY, (void*) tx_fctrl_f);

        default:
            return _dw_read_reg(TX_FCTRL, REST, (void*) tx_fctrl_f) &&
                   _dw_read_reg(TX_FCTRL, IFSDELAY, (void*) tx_fctrl_f);

    }

}

bool set_tx_fctrl(tx_fctrl_format* tx_fctrl_f, const tx_fctrl_subregister subregister) {

    switch(subregister) {

        case REST:
            return _dw_write_reg(TX_FCTRL, REST, (void*) tx_fctrl_f);

        case IFSDELAY:
            return _dw_write_reg(TX_FCTRL, IFSDELAY, (void*) tx_fctrl_f);

        default:
            return _dw_write_reg(TX_FCTRL, REST, (void*) tx_fctrl_f) &&
                   _dw_write_reg(TX_FCTRL, IFSDELAY, (void*) tx_fctrl_f);

    }
}

bool set_tx_buffer(uwb_frame_format* frame) {
    return _dw_write_reg(TX_BUFFER, 0, (void*) frame);
}

bool get_dx_time(double* seconds) {
    return _dw_read_reg(DX_TIME, 0, (void*) seconds);
}

bool set_dx_time(double* seconds) {
    return (*seconds <= DTR_COUNTER_WRAP_PERIOD) &&
           (*seconds >= 0.0) && _dw_write_reg(DX_TIME, 0, (void*) seconds);
}

bool get_rx_fwto(double* seconds) {
    return _dw_read_reg(RX_FWTO, 0, (void*) seconds);
}

bool set_rx_fwto(double* seconds) {
    return (*seconds <= RFR_COUNTER_WRAP_PERIOD) &&
           (*seconds >= 0.0) && _dw_write_reg(RX_FWTO, 0, (void*) seconds);
}

bool get_sys_ctrl(sys_ctrl_format* sys_ctrl_f) {
    return _dw_read_reg(SYS_CTRL, 0, (void*) sys_ctrl_f);
}

bool set_sys_ctrl(sys_ctrl_format* sys_ctrl_f) {
    return _dw_write_reg(SYS_CTRL, 0, (void*) sys_ctrl_f);
}

bool get_sys_event_msk(sys_evt_msk_format* sys_evt_msk_f) {
    return _dw_read_reg(SYS_MASK, 0, (void*) sys_evt_msk_f);
}

bool set_sys_event_msk(sys_evt_msk_format* sys_evt_msk_f) {
    return _dw_write_reg(SYS_MASK, 0, (void*) sys_evt_msk_f);
}

bool get_sys_event_sts(sys_evt_sts_format* sys_evt_sts_f, const sys_evt_sts_subregister subregister) {

    switch(subregister) {

        case SES_OCT_0_TO_3:
            return _dw_read_reg(SYS_STATUS, SES_OCT_0_TO_3, (void*) sys_evt_sts_f);

        case SES_OCT_4:
            return _dw_read_reg(SYS_STATUS, SES_OCT_4, (void*) sys_evt_sts_f);

        default:
            return _dw_read_reg(SYS_STATUS, SES_OCT_0_TO_3, (void*) sys_evt_sts_f) &&
                   _dw_read_reg(SYS_STATUS, SES_OCT_4, (void*) sys_evt_sts_f);

    }

}

bool set_sys_event_sts(sys_evt_sts_format* sys_evt_sts_f, const sys_evt_sts_subregister subregister) {

    switch(subregister) {

        case SES_OCT_0_TO_3:
            return _dw_write_reg(SYS_STATUS, SES_OCT_0_TO_3, (void*) sys_evt_sts_f);

        case SES_OCT_4:
            return _dw_write_reg(SYS_STATUS, SES_OCT_4, (void*) sys_evt_sts_f);

        default:
            return _dw_write_reg(SYS_STATUS, SES_OCT_0_TO_3, (void*) sys_evt_sts_f) &&
                   _dw_write_reg(SYS_STATUS, SES_OCT_4, (void*) sys_evt_sts_f);

    }

}

bool get_rx_finfo(rx_finfo_format* rx_finfo_f) {
    return _dw_read_reg(RX_FINFO, 0, (void*) rx_finfo_f);
}

bool get_rx_buffer(uwb_frame_format* frame) {
    return _dw_read_reg(RX_BUFFER, 0, (void*) frame);
}

bool get_rx_fqual(rx_fqual_format* rx_fqual_f, const rx_finfo_subregister subregister) {

    switch(subregister) {

        case FP_AMPL2_STD_NOISE:
            return _dw_read_reg(RX_FQUAL, FP_AMPL2_STD_NOISE, (void*) rx_fqual_f);

        case CIR_PWR_PP_AMPL3:
            return _dw_read_reg(RX_FQUAL, CIR_PWR_PP_AMPL3, (void*) rx_fqual_f);

        default:
            return _dw_read_reg(RX_FQUAL, FP_AMPL2_STD_NOISE, (void*) rx_fqual_f) &&
                   _dw_read_reg(RX_FQUAL, CIR_PWR_PP_AMPL3, (void*) rx_fqual_f);

    }

}

bool get_rx_ttcki(rx_ttcki_value* rx_ttcki) {
    return _dw_read_reg(RX_TTCKI, 0, (void*) rx_ttcki);
}

bool get_rx_ttcko(rx_ttcko_format* rx_ttcko_f, const rx_ttcko_subregister subregister) {

    switch(subregister) {

        case RX_TTCKO_OCT_0_TO_3:
            return _dw_read_reg(RX_TTCKO, RX_TTCKO_OCT_0_TO_3, (void*) rx_ttcko_f);

        case RX_TTCKO_OCT_4:
            return _dw_read_reg(RX_TTCKO, RX_TTCKO_OCT_4, (void*) rx_ttcko_f);

        default:
            return _dw_read_reg(RX_TTCKO, RX_TTCKO_OCT_0_TO_3, (void*) rx_ttcko_f) &&
                   _dw_read_reg(RX_TTCKO, RX_TTCKO_OCT_4, (void*) rx_ttcko_f);
    }

}

bool get_rx_time(rx_time_format* rx_time_f, const rx_time_subregister subregister) {

    switch(subregister) {

        case RX_TIME_OCT_0_TO_3:
            return _dw_read_reg(RX_TIME, RX_TIME_OCT_0_TO_3, (void*) rx_time_f);

        case RX_TIME_OCT_4_TO_7:
            return _dw_read_reg(RX_TIME, RX_TIME_OCT_4_TO_7, (void*) rx_time_f);

        case RX_TIME_OCT_8_TO_11:
            return _dw_read_reg(RX_TIME, RX_TIME_OCT_8_TO_11, (void*) rx_time_f);

        case RX_TIME_OCT_12_TO_13:
            return _dw_read_reg(RX_TIME, RX_TIME_OCT_12_TO_13, (void*) rx_time_f);

        default:
            return _dw_read_reg(RX_TIME, RX_TIME_OCT_0_TO_3, (void*) rx_time_f) &&
                   _dw_read_reg(RX_TIME, RX_TIME_OCT_4_TO_7, (void*) rx_time_f) &&
                   _dw_read_reg(RX_TIME, RX_TIME_OCT_8_TO_11, (void*) rx_time_f) &&
                   _dw_read_reg(RX_TIME, RX_TIME_OCT_12_TO_13, (void*) rx_time_f);

    }
}

bool get_tx_time(tx_time_format* tx_time_f, const tx_time_subregister subregister) {

    switch(subregister) {

        case TX_TIME_OCT_0_TO_3:
            return _dw_read_reg(TX_TIME, TX_TIME_OCT_0_TO_3, (void*) tx_time_f);

        case TX_TIME_OCT_4_TO_7:
            return _dw_read_reg(TX_TIME, TX_TIME_OCT_4_TO_7, (void*) tx_time_f);

        case TX_TIME_OCT_8_TO_9:
            return _dw_read_reg(TX_TIME, TX_TIME_OCT_8_TO_9, (void*) tx_time_f);

        default:
            return _dw_read_reg(TX_TIME, TX_TIME_OCT_0_TO_3, (void*) tx_time_f) &&
                   _dw_read_reg(TX_TIME, TX_TIME_OCT_4_TO_7, (void*) tx_time_f) &&
                   _dw_read_reg(TX_TIME, TX_TIME_OCT_8_TO_9, (void*) tx_time_f);
    }

}

bool get_tx_antd(double* seconds) {
    return _dw_read_reg(TX_ANTD, 0, (void*) seconds);
}

bool set_tx_antd(double* seconds) {
    return (*seconds <= TX_ANTD_WRAP_PERIOD) &&
           (*seconds >= 0.0) && _dw_write_reg(TX_ANTD, 0, (void*) seconds);
}

bool get_sys_status(sys_status_format* sys_status_f) {
    return _dw_read_reg(SYS_STATUS, 0, (void*) sys_status_f);
}

bool get_ack_resp_t(ack_resp_t_format* ack_resp_t_f) {
    return _dw_read_reg(ACK_RESP_T, 0, (void*) ack_resp_t_f);
}

bool set_ack_resp_t(ack_resp_t_format* ack_resp_t_f) {
    return (ack_resp_t_f->w4r_tim <= W4RTIM_WRAP_PERIOD) &&
           (ack_resp_t_f->w4r_tim >= 0.0) &&
           _dw_write_reg(ACK_RESP_T, 0, (void*) ack_resp_t_f);
}

bool get_rx_sniff(rx_sniff_format* rx_sniff_f) {
    return _dw_read_reg(RX_SNIFF, 0, (void*) rx_sniff_f);
}

bool set_rx_sniff(rx_sniff_format* rx_sniff_f) {
    return (rx_sniff_f->sniff_offt <= SNIFF_OFFT_WRAP_PERIOD) &&
           (rx_sniff_f->sniff_offt >= 0.0) &&
           _dw_write_reg(RX_SNIFF, 0, (void*) rx_sniff_f);
}

bool get_tx_power(tx_power_format* tx_power_f) {
    return _dw_read_reg(TX_POWER, 0, (void*) tx_power_f);
}

bool set_tx_power(tx_power_format* tx_power_f) {
    return _dw_write_reg(TX_POWER, 0, (void*) tx_power_f);
}

bool get_chan_ctrl(chan_ctrl_format* chan_ctrl_f) {
    return _dw_read_reg(CHAN_CTRL, 0, (void*) chan_ctrl_f);
}

bool set_chan_ctrl(chan_ctrl_format* chan_ctrl_f) {
    return _dw_write_reg(CHAN_CTRL, 0, (void*) chan_ctrl_f);
}

bool get_usr_sfd(usr_sfd_format* usr_sfd_f, const usr_sfd_subregister subregister) {

    switch(subregister) {

        case USR_SFD_OCT_0_TO_3:
            return _dw_read_reg(USR_SFD, USR_SFD_OCT_0_TO_3, (void*) usr_sfd_f);

        case USR_SFD_OCT_4_TO_7:
            return _dw_read_reg(USR_SFD, USR_SFD_OCT_4_TO_7, (void*) usr_sfd_f);

        case USR_SFD_OCT_8_TO_11:
            return _dw_read_reg(USR_SFD, USR_SFD_OCT_8_TO_11, (void*) usr_sfd_f);

        case USR_SFD_OCT_12_TO_15:
            return _dw_read_reg(USR_SFD, USR_SFD_OCT_12_TO_15, (void*) usr_sfd_f);

        case USR_SFD_OCT_16_TO_19:
            return _dw_read_reg(USR_SFD, USR_SFD_OCT_16_TO_19, (void*) usr_sfd_f);

        case USR_SFD_OCT_20_TO_23:
            return _dw_read_reg(USR_SFD, USR_SFD_OCT_20_TO_23, (void*) usr_sfd_f);

        case USR_SFD_OCT_24_TO_27:
            return _dw_read_reg(USR_SFD, USR_SFD_OCT_24_TO_27, (void*) usr_sfd_f);

        case USR_SFD_OCT_28_TO_31:
            return _dw_read_reg(USR_SFD, USR_SFD_OCT_28_TO_31, (void*) usr_sfd_f);

        case USR_SFD_OCT_32_TO_35:
            return _dw_read_reg(USR_SFD, USR_SFD_OCT_32_TO_35, (void*) usr_sfd_f);

        case USR_SFD_OCT_36_TO_39:
            return _dw_read_reg(USR_SFD, USR_SFD_OCT_36_TO_39, (void*) usr_sfd_f);

        case USR_SFD_OCT_40_TO_41:
            return _dw_read_reg(USR_SFD, USR_SFD_OCT_40_TO_41, (void*) usr_sfd_f);

        default:
            return _dw_read_reg(USR_SFD, USR_SFD_OCT_0_TO_3, (void*) usr_sfd_f) &&
                   _dw_read_reg(USR_SFD, USR_SFD_OCT_4_TO_7, (void*) usr_sfd_f) &&
                   _dw_read_reg(USR_SFD, USR_SFD_OCT_8_TO_11, (void*) usr_sfd_f) &&
                   _dw_read_reg(USR_SFD, USR_SFD_OCT_12_TO_15, (void*) usr_sfd_f) &&
                   _dw_read_reg(USR_SFD, USR_SFD_OCT_16_TO_19, (void*) usr_sfd_f) &&
                   _dw_read_reg(USR_SFD, USR_SFD_OCT_20_TO_23, (void*) usr_sfd_f) &&
                   _dw_read_reg(USR_SFD, USR_SFD_OCT_24_TO_27, (void*) usr_sfd_f) &&
                   _dw_read_reg(USR_SFD, USR_SFD_OCT_28_TO_31, (void*) usr_sfd_f) &&
                   _dw_read_reg(USR_SFD, USR_SFD_OCT_32_TO_35, (void*) usr_sfd_f) &&
                   _dw_read_reg(USR_SFD, USR_SFD_OCT_36_TO_39, (void*) usr_sfd_f) &&
                   _dw_read_reg(USR_SFD, USR_SFD_OCT_40_TO_41, (void*) usr_sfd_f);
    }

}

bool set_usr_sfd(usr_sfd_format* usr_sfd_f, const usr_sfd_subregister subregister) {

    switch(subregister) {

        case USR_SFD_OCT_0_TO_3:
            return _dw_write_reg(USR_SFD, USR_SFD_OCT_0_TO_3, (void*) usr_sfd_f);

        case USR_SFD_OCT_4_TO_7:
            return _dw_write_reg(USR_SFD, USR_SFD_OCT_4_TO_7, (void*) usr_sfd_f);

        case USR_SFD_OCT_8_TO_11:
            return _dw_write_reg(USR_SFD, USR_SFD_OCT_8_TO_11, (void*) usr_sfd_f);

        case USR_SFD_OCT_12_TO_15:
            return _dw_write_reg(USR_SFD, USR_SFD_OCT_12_TO_15, (void*) usr_sfd_f);

        case USR_SFD_OCT_16_TO_19:
            return _dw_write_reg(USR_SFD, USR_SFD_OCT_16_TO_19, (void*) usr_sfd_f);

        case USR_SFD_OCT_20_TO_23:
            return _dw_write_reg(USR_SFD, USR_SFD_OCT_20_TO_23, (void*) usr_sfd_f);

        case USR_SFD_OCT_24_TO_27:
            return _dw_write_reg(USR_SFD, USR_SFD_OCT_24_TO_27, (void*) usr_sfd_f);

        case USR_SFD_OCT_28_TO_31:
            return _dw_write_reg(USR_SFD, USR_SFD_OCT_28_TO_31, (void*) usr_sfd_f);

        case USR_SFD_OCT_32_TO_35:
            return _dw_write_reg(USR_SFD, USR_SFD_OCT_32_TO_35, (void*) usr_sfd_f);

        case USR_SFD_OCT_36_TO_39:
            return _dw_write_reg(USR_SFD, USR_SFD_OCT_36_TO_39, (void*) usr_sfd_f);

        case USR_SFD_OCT_40_TO_41:
            return _dw_write_reg(USR_SFD, USR_SFD_OCT_40_TO_41, (void*) usr_sfd_f);

        default:
            return _dw_write_reg(USR_SFD, USR_SFD_OCT_0_TO_3, (void*) usr_sfd_f) &&
                   _dw_write_reg(USR_SFD, USR_SFD_OCT_4_TO_7, (void*) usr_sfd_f) &&
                   _dw_write_reg(USR_SFD, USR_SFD_OCT_8_TO_11, (void*) usr_sfd_f) &&
                   _dw_write_reg(USR_SFD, USR_SFD_OCT_12_TO_15, (void*) usr_sfd_f) &&
                   _dw_write_reg(USR_SFD, USR_SFD_OCT_16_TO_19, (void*) usr_sfd_f) &&
                   _dw_write_reg(USR_SFD, USR_SFD_OCT_20_TO_23, (void*) usr_sfd_f) &&
                   _dw_write_reg(USR_SFD, USR_SFD_OCT_24_TO_27, (void*) usr_sfd_f) &&
                   _dw_write_reg(USR_SFD, USR_SFD_OCT_28_TO_31, (void*) usr_sfd_f) &&
                   _dw_write_reg(USR_SFD, USR_SFD_OCT_32_TO_35, (void*) usr_sfd_f) &&
                   _dw_write_reg(USR_SFD, USR_SFD_OCT_36_TO_39, (void*) usr_sfd_f) &&
                   _dw_write_reg(USR_SFD, USR_SFD_OCT_40_TO_41, (void*) usr_sfd_f);
    }
}

bool get_agc_ctrl(agc_ctrl_format* agc_ctrl_f, const agc_ctrl_subregister subregister) {

    switch(subregister) {
        case AGC_CTRL1:
            return _dw_read_reg(AGC_CTRL, AGC_CTRL1, (void*) agc_ctrl_f);

        case AGC_TUNE1:
            return _dw_read_reg(AGC_CTRL, AGC_TUNE1, (void*) agc_ctrl_f);

        case AGC_TUNE2:
            return _dw_read_reg(AGC_CTRL, AGC_TUNE2, (void*) agc_ctrl_f);

        case AGC_TUNE3:
            return _dw_read_reg(AGC_CTRL, AGC_TUNE3, (void*) agc_ctrl_f);

        case AGC_STAT1:
            return _dw_read_reg(AGC_CTRL, AGC_STAT1, (void*) agc_ctrl_f);

        default:
            return _dw_read_reg(AGC_CTRL, AGC_CTRL1, (void*) agc_ctrl_f) &&
                   _dw_read_reg(AGC_CTRL, AGC_TUNE1, (void*) agc_ctrl_f) &&
                   _dw_read_reg(AGC_CTRL, AGC_TUNE2, (void*) agc_ctrl_f) &&
                   _dw_read_reg(AGC_CTRL, AGC_TUNE3, (void*) agc_ctrl_f) &&
                   _dw_read_reg(AGC_CTRL, AGC_STAT1, (void*) agc_ctrl_f);
    }

}

bool set_agc_ctrl(agc_ctrl_format* agc_ctrl_f, const agc_ctrl_subregister subregister) {

    switch(subregister) {
        case AGC_CTRL1:
            return _dw_write_reg(AGC_CTRL, AGC_CTRL1, (void*) agc_ctrl_f);

        case AGC_TUNE1:
            return _dw_write_reg(AGC_CTRL, AGC_TUNE1, (void*) agc_ctrl_f);

        case AGC_TUNE2:
            return _dw_write_reg(AGC_CTRL, AGC_TUNE2, (void*) agc_ctrl_f);

        case AGC_TUNE3:
            return _dw_write_reg(AGC_CTRL, AGC_TUNE3, (void*) agc_ctrl_f);

        case AGC_STAT1:
            return false;

        default:
            return _dw_write_reg(AGC_CTRL, AGC_CTRL1, (void*) agc_ctrl_f) &&
                   _dw_write_reg(AGC_CTRL, AGC_TUNE1, (void*) agc_ctrl_f) &&
                   _dw_write_reg(AGC_CTRL, AGC_TUNE2, (void*) agc_ctrl_f) &&
                   _dw_write_reg(AGC_CTRL, AGC_TUNE3, (void*) agc_ctrl_f);
    }

}

bool get_ext_sync(ext_sync_format* ext_sync_f, const ext_sync_subregister subregister) {

    switch(subregister) {

        case EC_CTRL:
            return _dw_read_reg(EXT_SYNC, EC_CTRL, (void*) ext_sync_f);

        case EC_RXTC:
            return _dw_read_reg(EXT_SYNC, EC_RXTC, (void*) ext_sync_f);

        case EC_GOLP:
            return _dw_read_reg(EXT_SYNC, EC_GOLP, (void*) ext_sync_f);

        default:
            return _dw_read_reg(EXT_SYNC, EC_CTRL, (void*) ext_sync_f) &&
                   _dw_read_reg(EXT_SYNC, EC_RXTC, (void*) ext_sync_f) &&
                   _dw_read_reg(EXT_SYNC, EC_GOLP, (void*) ext_sync_f);

    }

}

bool set_ext_sync(ext_sync_format* ext_sync_f, const ext_sync_subregister subregister) {

    switch(subregister) {

        case EC_CTRL:
            return _dw_write_reg(EXT_SYNC, EC_CTRL, (void*) ext_sync_f);

        case EC_RXTC:
            return false;

        case EC_GOLP:
            return false;

        default:
            return _dw_write_reg(EXT_SYNC, EC_CTRL, (void*) ext_sync_f);

    }
    return _dw_write_reg(EXT_SYNC, EC_CTRL, (void*) ext_sync_f);
}

bool get_acc_mem(acc_mem_field* acc_mem_f, const size_t offset) {
    return _dw_read_reg(ACC_MEM, offset, (void*) acc_mem_f);
}

bool get_gpio_ctrl(gpio_ctrl_format* gpio_ctrl_format, const gpio_ctrl_subregister subregister) {

    switch(subregister) {

        case GPIO_MODE:
            return _dw_read_reg(GPIO_CTRL, GPIO_MODE, (void*) &gpio_ctrl_format->gpio_mode_ctrl_f);

        case GPIO_DIR:
            return _dw_read_reg(GPIO_CTRL, GPIO_DIR, (void*) &gpio_ctrl_format->gpio_direction_ctrl_f);

        case GPIO_DOUT:
            return _dw_read_reg(GPIO_CTRL, GPIO_DOUT, (void*) &gpio_ctrl_format->gpio_data_output_ctrl_f);

        case GPIO_IRQE:
            return _dw_read_reg(GPIO_CTRL, GPIO_IRQE, (void*) &gpio_ctrl_format->gpio_irq_ctrl_f);

        case GPIO_ISEN:
            return _dw_read_reg(GPIO_CTRL, GPIO_ISEN, (void*) &gpio_ctrl_format->gpio_irq_sense_ctrl_f);

        case GPIO_IMODE:
            return _dw_read_reg(GPIO_CTRL, GPIO_IMODE, (void*) &gpio_ctrl_format->gpio_irq_mode_ctrl_f);

        case GPIO_IBES:
            return _dw_read_reg(GPIO_CTRL, GPIO_IBES, (void*) &gpio_ctrl_format->gpio_irq_both_edges_mode_f);

        case GPIO_ICLR:
            return _dw_read_reg(GPIO_CTRL, GPIO_ICLR, (void*) &gpio_ctrl_format->gpio_irq_latch_clear_mode_f);

        case GPIO_IDBE:
            return _dw_read_reg(GPIO_CTRL, GPIO_IDBE, (void*) &gpio_ctrl_format->gpio_irq_de_bounce_mode_f);

        case GPIO_RAW:
            return _dw_read_reg(GPIO_CTRL, GPIO_RAW, (void*) &gpio_ctrl_format->gpio_raw_state_f);

        default:
            return _dw_read_reg(GPIO_CTRL, GPIO_MODE, (void*) &gpio_ctrl_format->gpio_mode_ctrl_f) &&
                   _dw_read_reg(GPIO_CTRL, GPIO_DIR, (void*) &gpio_ctrl_format->gpio_direction_ctrl_f) &&
                   _dw_read_reg(GPIO_CTRL, GPIO_DOUT, (void*) &gpio_ctrl_format->gpio_data_output_ctrl_f) &&
                   _dw_read_reg(GPIO_CTRL, GPIO_IRQE, (void*) &gpio_ctrl_format->gpio_irq_ctrl_f) &&
                   _dw_read_reg(GPIO_CTRL, GPIO_ISEN, (void*) &gpio_ctrl_format->gpio_irq_sense_ctrl_f) &&
                   _dw_read_reg(GPIO_CTRL, GPIO_IMODE, (void*) &gpio_ctrl_format->gpio_irq_mode_ctrl_f) &&
                   _dw_read_reg(GPIO_CTRL, GPIO_IBES, (void*) &gpio_ctrl_format->gpio_irq_both_edges_mode_f) &&
                   _dw_read_reg(GPIO_CTRL, GPIO_ICLR, (void*) &gpio_ctrl_format->gpio_irq_latch_clear_mode_f) &&
                   _dw_read_reg(GPIO_CTRL, GPIO_IDBE, (void*) &gpio_ctrl_format->gpio_irq_de_bounce_mode_f) &&
                   _dw_read_reg(GPIO_CTRL, GPIO_RAW, (void*) &gpio_ctrl_format->gpio_raw_state_f);
    }

}

bool set_gpio_ctrl(gpio_ctrl_format* gpio_ctrl_format, const gpio_ctrl_subregister subregister) {

    switch(subregister) {

        case GPIO_MODE:
            return _dw_write_reg(GPIO_CTRL, GPIO_MODE, (void*) &gpio_ctrl_format->gpio_mode_ctrl_f);

        case GPIO_DIR:
            return _dw_write_reg(GPIO_CTRL, GPIO_DIR, (void*) &gpio_ctrl_format->gpio_direction_ctrl_f);

        case GPIO_DOUT:
            return _dw_write_reg(GPIO_CTRL, GPIO_DOUT, (void*) &gpio_ctrl_format->gpio_data_output_ctrl_f);

        case GPIO_IRQE:
            return _dw_write_reg(GPIO_CTRL, GPIO_IRQE, (void*) &gpio_ctrl_format->gpio_irq_ctrl_f);

        case GPIO_ISEN:
            return _dw_write_reg(GPIO_CTRL, GPIO_ISEN, (void*) &gpio_ctrl_format->gpio_irq_sense_ctrl_f);

        case GPIO_IMODE:
            return _dw_write_reg(GPIO_CTRL, GPIO_IMODE, (void*) &gpio_ctrl_format->gpio_irq_mode_ctrl_f);

        case GPIO_IBES:
            return _dw_write_reg(GPIO_CTRL, GPIO_IBES, (void*) &gpio_ctrl_format->gpio_irq_both_edges_mode_f);

        case GPIO_ICLR:
            return _dw_write_reg(GPIO_CTRL, GPIO_ICLR, (void*) &gpio_ctrl_format->gpio_irq_latch_clear_mode_f);

        case GPIO_IDBE:
            return _dw_write_reg(GPIO_CTRL, GPIO_IDBE, (void*) &gpio_ctrl_format->gpio_irq_de_bounce_mode_f);

        case GPIO_RAW:
            return false;

        default:
            return _dw_write_reg(GPIO_CTRL, GPIO_MODE, (void*) &gpio_ctrl_format->gpio_mode_ctrl_f) &&
                   _dw_write_reg(GPIO_CTRL, GPIO_DIR, (void*) &gpio_ctrl_format->gpio_direction_ctrl_f) &&
                   _dw_write_reg(GPIO_CTRL, GPIO_DOUT, (void*) &gpio_ctrl_format->gpio_data_output_ctrl_f) &&
                   _dw_write_reg(GPIO_CTRL, GPIO_IRQE, (void*) &gpio_ctrl_format->gpio_irq_ctrl_f) &&
                   _dw_write_reg(GPIO_CTRL, GPIO_ISEN, (void*) &gpio_ctrl_format->gpio_irq_sense_ctrl_f) &&
                   _dw_write_reg(GPIO_CTRL, GPIO_IMODE, (void*) &gpio_ctrl_format->gpio_irq_mode_ctrl_f) &&
                   _dw_write_reg(GPIO_CTRL, GPIO_IBES, (void*) &gpio_ctrl_format->gpio_irq_both_edges_mode_f) &&
                   _dw_write_reg(GPIO_CTRL, GPIO_ICLR, (void*) &gpio_ctrl_format->gpio_irq_latch_clear_mode_f) &&
                   _dw_write_reg(GPIO_CTRL, GPIO_IDBE, (void*) &gpio_ctrl_format->gpio_irq_de_bounce_mode_f);
    }

}

bool get_drx_conf(drx_conf_format* drx_conf_f, const drx_conf_subregister subregister) {

    switch(subregister) {

        case DRX_TUNE0B:
            return _dw_read_reg(DRX_CONF, DRX_TUNE0B, (void*) drx_conf_f);

        case DRX_TUNE1A:
            return _dw_read_reg(DRX_CONF, DRX_TUNE1A, (void*) drx_conf_f);

        case DRX_TUNE1B:
            return _dw_read_reg(DRX_CONF, DRX_TUNE1B, (void*) drx_conf_f);

        case DRX_TUNE2:
            return _dw_read_reg(DRX_CONF, DRX_TUNE2, (void*) drx_conf_f);

        case DRX_SFDTOC:
            return _dw_read_reg(DRX_CONF, DRX_SFDTOC, (void*) drx_conf_f);

        case DRX_PRETOC:
            return _dw_read_reg(DRX_CONF, DRX_PRETOC, (void*) drx_conf_f);

        case DRX_TUNE4H:
            return _dw_read_reg(DRX_CONF, DRX_TUNE4H, (void*) drx_conf_f);

        case DRX_CAR_INT:
            return _dw_read_reg(DRX_CONF, DRX_CAR_INT, (void*) drx_conf_f);

        case RXPACC_NOSAT:
            return _dw_read_reg(DRX_CONF, RXPACC_NOSAT, (void*) drx_conf_f);

        default:
            return _dw_read_reg(DRX_CONF, DRX_TUNE0B, (void*) drx_conf_f) &&
                   _dw_read_reg(DRX_CONF, DRX_TUNE1A, (void*) drx_conf_f) &&
                   _dw_read_reg(DRX_CONF, DRX_TUNE1B, (void*) drx_conf_f) &&
                   _dw_read_reg(DRX_CONF, DRX_TUNE2, (void*) drx_conf_f) &&
                   _dw_read_reg(DRX_CONF, DRX_SFDTOC, (void*) drx_conf_f) &&
                   _dw_read_reg(DRX_CONF, DRX_PRETOC, (void*) drx_conf_f) &&
                   _dw_read_reg(DRX_CONF, DRX_TUNE4H, (void*) drx_conf_f) &&
                   _dw_read_reg(DRX_CONF, DRX_CAR_INT, (void*) drx_conf_f) &&
                   _dw_read_reg(DRX_CONF, RXPACC_NOSAT, (void*) drx_conf_f);
    }

}

bool set_drx_conf(drx_conf_format* drx_conf_f, const drx_conf_subregister subregister) {

    switch(subregister) {

        case DRX_TUNE0B:
            return _dw_write_reg(DRX_CONF, DRX_TUNE0B, (void*) drx_conf_f);

        case DRX_TUNE1A:
            return _dw_write_reg(DRX_CONF, DRX_TUNE1A, (void*) drx_conf_f);

        case DRX_TUNE1B:
            return _dw_write_reg(DRX_CONF, DRX_TUNE1B, (void*) drx_conf_f);

        case DRX_TUNE2:
            return _dw_write_reg(DRX_CONF, DRX_TUNE2, (void*) drx_conf_f);

        case DRX_SFDTOC:
            return _dw_write_reg(DRX_CONF, DRX_SFDTOC, (void*) drx_conf_f);

        case DRX_PRETOC:
            return _dw_write_reg(DRX_CONF, DRX_PRETOC, (void*) drx_conf_f);

        case DRX_TUNE4H:
            return _dw_write_reg(DRX_CONF, DRX_TUNE4H, (void*) drx_conf_f);

        case DRX_CAR_INT:
            return false;

        case RXPACC_NOSAT:
            return false;

        default:
            return _dw_write_reg(DRX_CONF, DRX_TUNE0B, (void*) drx_conf_f) &&
                   _dw_write_reg(DRX_CONF, DRX_TUNE1A, (void*) drx_conf_f) &&
                   _dw_write_reg(DRX_CONF, DRX_TUNE1B, (void*) drx_conf_f) &&
                   _dw_write_reg(DRX_CONF, DRX_TUNE2, (void*) drx_conf_f) &&
                   _dw_write_reg(DRX_CONF, DRX_SFDTOC, (void*) drx_conf_f) &&
                   _dw_write_reg(DRX_CONF, DRX_PRETOC, (void*) drx_conf_f) &&
                   _dw_write_reg(DRX_CONF, DRX_TUNE4H, (void*) drx_conf_f);

    }

}

bool get_rf_conf(rf_conf_format* rf_conf_f, const rf_conf_subregister subregister) {

    switch(subregister) {

        case SRF_CONF:
            return _dw_read_reg(RF_CONF, SRF_CONF, (void*) rf_conf_f);

        case RF_RXCTRLH:
            return _dw_read_reg(RF_CONF, RF_RXCTRLH, (void*) rf_conf_f);

        case RF_TXCTRL:
            return _dw_read_reg(RF_CONF, RF_TXCTRL, (void*) rf_conf_f);

        case RF_STATUS:
            return _dw_read_reg(RF_CONF, RF_STATUS, (void*) rf_conf_f);

        case LDOTUNE:
            return _dw_read_reg(RF_CONF, LDOTUNE, (void*) rf_conf_f);

        default:
            return _dw_read_reg(RF_CONF, SRF_CONF, (void*) rf_conf_f) &&
                   _dw_read_reg(RF_CONF, RF_RXCTRLH, (void*) rf_conf_f) &&
                   _dw_read_reg(RF_CONF, RF_TXCTRL, (void*) rf_conf_f) &&
                   _dw_read_reg(RF_CONF, RF_STATUS, (void*) rf_conf_f) &&
                   _dw_read_reg(RF_CONF, LDOTUNE, (void*) rf_conf_f);
    }

}

bool set_rf_conf(rf_conf_format* rf_conf_f, const rf_conf_subregister subregister) {

    switch(subregister) {

        case SRF_CONF:
            return _dw_write_reg(RF_CONF, SRF_CONF, (void*) rf_conf_f);

        case RF_RXCTRLH:
            return _dw_write_reg(RF_CONF, RF_RXCTRLH, (void*) rf_conf_f);

        case RF_TXCTRL:
            return _dw_write_reg(RF_CONF, RF_TXCTRL, (void*) rf_conf_f);

        case RF_STATUS:
            return false;

        case LDOTUNE:
            return _dw_write_reg(RF_CONF, LDOTUNE, (void*) rf_conf_f);

        case RF_TLD_BIAS:
            return _dw_write_reg(RF_CONF, RF_TLD_BIAS, (void*) rf_conf_f);

        case RF_TLD_ADC_BIAS:
            return _dw_write_reg(RF_CONF, RF_TLD_ADC_BIAS, (void*) rf_conf_f);

        default:
            return _dw_write_reg(RF_CONF, SRF_CONF, (void*) rf_conf_f) &&
                   _dw_write_reg(RF_CONF, RF_RXCTRLH, (void*) rf_conf_f) &&
                   _dw_write_reg(RF_CONF, RF_TXCTRL, (void*) rf_conf_f) &&
                   _dw_write_reg(RF_CONF, LDOTUNE, (void*) rf_conf_f) &&
                   _dw_write_reg(RF_CONF, RF_TLD_BIAS, (void*) rf_conf_f) &&
                   _dw_write_reg(RF_CONF, RF_TLD_ADC_BIAS, (void*) rf_conf_f);
    }

}

bool get_tx_cal(tx_cal_format* tx_cal_f, const tx_cal_subregister subregister) {

    switch(subregister) {

        case TC_SARC:
            return _dw_read_reg(TX_CAL, TC_SARC, (void*) tx_cal_f);

        case TC_SARL:
            return _dw_read_reg(TX_CAL, TC_SARL, (void*) tx_cal_f);

        case TC_SARW:
            return _dw_read_reg(TX_CAL, TC_SARW, (void*) tx_cal_f);

        case TC_PG_CTRL:
            return _dw_read_reg(TX_CAL, TC_PG_CTRL, (void*) tx_cal_f);

        case TC_PG_STATUS:
            return _dw_read_reg(TX_CAL, TC_PG_STATUS, (void*) tx_cal_f);

        case TC_PG_DELAY:
            return _dw_read_reg(TX_CAL, TC_PG_DELAY, (void*) tx_cal_f);

        case TC_PG_TEST:
            return _dw_read_reg(TX_CAL, TC_PG_TEST, (void*) tx_cal_f);

        default:
            return _dw_read_reg(TX_CAL, TC_SARC, (void*) tx_cal_f) &&
                   _dw_read_reg(TX_CAL, TC_SARL, (void*) tx_cal_f) &&
                   _dw_read_reg(TX_CAL, TC_SARW, (void*) tx_cal_f) &&
                   _dw_read_reg(TX_CAL, TC_PG_CTRL, (void*) tx_cal_f) &&
                   _dw_read_reg(TX_CAL, TC_PG_STATUS, (void*) tx_cal_f) &&
                   _dw_read_reg(TX_CAL, TC_PG_DELAY, (void*) tx_cal_f) &&
                   _dw_read_reg(TX_CAL, TC_PG_TEST, (void*) tx_cal_f);
    }

}

bool set_tx_cal(tx_cal_format* tx_cal_f, const tx_cal_subregister subregister) {

    switch(subregister) {

        case TC_SARC:
            return _dw_write_reg(TX_CAL, TC_SARC, (void*) tx_cal_f);

        case TC_SARL:
            return false;

        case TC_SARW:
            return false;

        case TC_PG_CTRL:
            return _dw_write_reg(TX_CAL, TC_PG_CTRL, (void*) tx_cal_f);

        case TC_PG_STATUS:
            return false;

        case TC_PG_DELAY:
            return _dw_write_reg(TX_CAL, TC_PG_DELAY, (void*) tx_cal_f);

        case TC_PG_TEST:
            return _dw_write_reg(TX_CAL, TC_PG_TEST, (void*) tx_cal_f);

        default:
            return _dw_write_reg(TX_CAL, TC_SARC, (void*) tx_cal_f) &&
                   _dw_write_reg(TX_CAL, TC_PG_CTRL, (void*) tx_cal_f) &&
                   _dw_write_reg(TX_CAL, TC_PG_DELAY, (void*) tx_cal_f) &&
                   _dw_write_reg(TX_CAL, TC_PG_TEST, (void*) tx_cal_f);
    }


}

bool get_fs_ctrl(fs_ctrl_format* fs_ctrl_f, const fs_ctrl_subregister subregister) {

    switch(subregister) {

        case FS_PLLCFG:
            return _dw_read_reg(FS_CTRL, FS_PLLCFG, (void*) fs_ctrl_f);

        case FS_PLLTUNE:
            return _dw_read_reg(FS_CTRL, FS_PLLTUNE, (void*) fs_ctrl_f);

        case FS_XTALT:
            return _dw_read_reg(FS_CTRL, FS_XTALT, (void*) fs_ctrl_f);

        default:
            return _dw_read_reg(FS_CTRL, FS_PLLCFG, (void*) fs_ctrl_f) &&
                   _dw_read_reg(FS_CTRL, FS_PLLTUNE, (void*) fs_ctrl_f) &&
                   _dw_read_reg(FS_CTRL, FS_XTALT, (void*) fs_ctrl_f);
    }

}

bool set_fs_ctrl(fs_ctrl_format* fs_ctrl_f, const fs_ctrl_subregister subregister) {

    switch(subregister) {

        case FS_PLLCFG:
            return _dw_write_reg(FS_CTRL, FS_PLLCFG, (void*) fs_ctrl_f);

        case FS_PLLTUNE:
            return _dw_write_reg(FS_CTRL, FS_PLLTUNE, (void*) fs_ctrl_f);

        case FS_XTALT:
            return _dw_write_reg(FS_CTRL, FS_XTALT, (void*) fs_ctrl_f);

        default:
            return _dw_write_reg(FS_CTRL, FS_PLLCFG, (void*) fs_ctrl_f) &&
                   _dw_write_reg(FS_CTRL, FS_PLLTUNE, (void*) fs_ctrl_f) &&
                   _dw_write_reg(FS_CTRL, FS_XTALT, (void*) fs_ctrl_f);
    }

}

bool get_aon(aon_format* aon_f, const aon_subregister subregister) {

    switch(subregister) {

        case AON_WCFG:
            return _dw_read_reg(AON, AON_WCFG, (void*) aon_f);

        case AON_CTRL:
            return _dw_read_reg(AON, AON_CTRL, (void*) aon_f);

        case AON_RDAT:
            return _dw_read_reg(AON, AON_RDAT, (void*) aon_f);

        case AON_ADDR:
            return _dw_read_reg(AON, AON_ADDR, (void*) aon_f);

        case AON_CFG0:
            return _dw_read_reg(AON, AON_CFG0, (void*) aon_f);

        case AON_CFG1:
            return _dw_read_reg(AON, AON_CFG1, (void*) aon_f);

        default:
            return _dw_read_reg(AON, AON_WCFG, (void*) aon_f) &&
                   _dw_read_reg(AON, AON_CTRL, (void*) aon_f) &&
                   _dw_read_reg(AON, AON_RDAT, (void*) aon_f) &&
                   _dw_read_reg(AON, AON_ADDR, (void*) aon_f) &&
                   _dw_read_reg(AON, AON_CFG0, (void*) aon_f) &&
                   _dw_read_reg(AON, AON_CFG1, (void*) aon_f);
    }

}

bool set_aon(aon_format* aon_f, const aon_subregister subregister) {

    switch(subregister) {

        case AON_WCFG:
            return _dw_write_reg(AON, AON_WCFG, (void*) aon_f);

        case AON_CTRL:
            return _dw_write_reg(AON, AON_CTRL, (void*) aon_f);

        case AON_RDAT:
            return _dw_write_reg(AON, AON_RDAT, (void*) aon_f);

        case AON_ADDR:
            return _dw_write_reg(AON, AON_ADDR, (void*) aon_f);

        case AON_CFG0:
            return _dw_write_reg(AON, AON_CFG0, (void*) aon_f);

        case AON_CFG1:
            return _dw_write_reg(AON, AON_CFG1, (void*) aon_f);

        default:
            return _dw_write_reg(AON, AON_WCFG, (void*) aon_f) &&
                   _dw_write_reg(AON, AON_CTRL, (void*) aon_f) &&
                   _dw_write_reg(AON, AON_RDAT, (void*) aon_f) &&
                   _dw_write_reg(AON, AON_ADDR, (void*) aon_f) &&
                   _dw_write_reg(AON, AON_CFG0, (void*) aon_f) &&
                   _dw_write_reg(AON, AON_CFG1, (void*) aon_f);
    }

}

bool get_otp_if(otp_if_format* otp_if_f, const otp_if_subregister subregister) {

    switch(subregister) {

        case OTP_WDAT:
            return _dw_read_reg(OTP_IF, OTP_WDAT, (void*) otp_if_f);

        case OTP_ADDR:
            return _dw_read_reg(OTP_IF, OTP_ADDR, (void*) otp_if_f);

        case OTP_CTRL:
            return _dw_read_reg(OTP_IF, OTP_CTRL, (void*) otp_if_f);

        case OTP_STAT:
            return _dw_read_reg(OTP_IF, OTP_STAT, (void*) otp_if_f);

        case OTP_RDAT:
            return _dw_read_reg(OTP_IF, OTP_RDAT, (void*) otp_if_f);

        case OTP_SRDAT:
            return _dw_read_reg(OTP_IF, OTP_SRDAT, (void*) otp_if_f);

        case OTP_SF:
            return _dw_read_reg(OTP_IF, OTP_SF, (void*) otp_if_f);

        default:
            return _dw_read_reg(OTP_IF, OTP_WDAT, (void*) otp_if_f) &&
                   _dw_read_reg(OTP_IF, OTP_ADDR, (void*) otp_if_f) &&
                   _dw_read_reg(OTP_IF, OTP_CTRL, (void*) otp_if_f) &&
                   _dw_read_reg(OTP_IF, OTP_STAT, (void*) otp_if_f) &&
                   _dw_read_reg(OTP_IF, OTP_RDAT, (void*) otp_if_f) &&
                   _dw_read_reg(OTP_IF, OTP_SRDAT, (void*) otp_if_f) &&
                   _dw_read_reg(OTP_IF, OTP_SF, (void*) otp_if_f);
    }

}

bool set_otp_if(otp_if_format* otp_if_f, const otp_if_subregister subregister) {

    switch(subregister) {

        case OTP_WDAT:
            return _dw_write_reg(OTP_IF, OTP_WDAT, (void*) otp_if_f);

        case OTP_ADDR:
            return _dw_write_reg(OTP_IF, OTP_ADDR, (void*) otp_if_f);

        case OTP_CTRL:
            return _dw_write_reg(OTP_IF, OTP_CTRL, (void*) otp_if_f);

        case OTP_STAT:
            return _dw_write_reg(OTP_IF, OTP_STAT, (void*) otp_if_f);

        case OTP_SRDAT:
            return _dw_write_reg(OTP_IF, OTP_SRDAT, (void*) otp_if_f);

        case OTP_SF:
            return _dw_write_reg(OTP_IF, OTP_SF, (void*) otp_if_f);

        default:
            return _dw_write_reg(OTP_IF, OTP_WDAT, (void*) otp_if_f) &&
                   _dw_write_reg(OTP_IF, OTP_ADDR, (void*) otp_if_f) &&
                   _dw_write_reg(OTP_IF, OTP_CTRL, (void*) otp_if_f) &&
                   _dw_write_reg(OTP_IF, OTP_STAT, (void*) otp_if_f) &&
                   _dw_write_reg(OTP_IF, OTP_SRDAT, (void*) otp_if_f) &&
                   _dw_write_reg(OTP_IF, OTP_SF, (void*) otp_if_f);
    }

}

bool get_lde_if(lde_if_format* lde_if_f, const lde_if_subregister subregister) {

    switch(subregister) {

        case LDE_REPC:
            return _dw_read_reg(LDE_IF, LDE_REPC, (void*) lde_if_f);

        case LDE_CFG2:
            return _dw_read_reg(LDE_IF, LDE_CFG2, (void*) lde_if_f);

        case LDE_RXANTD:
            return _dw_read_reg(LDE_IF, LDE_RXANTD, (void*) lde_if_f);

        case LDE_PPAMPL:
            return _dw_read_reg(LDE_IF, LDE_PPAMPL, (void*) lde_if_f);

        case LDE_PPINDX:
            return _dw_read_reg(LDE_IF, LDE_PPINDX, (void*) lde_if_f);

        case LDE_CFG1:
            return _dw_read_reg(LDE_IF, LDE_CFG1, (void*) lde_if_f);

        case LDE_THRESH:
            return _dw_read_reg(LDE_IF, LDE_THRESH, (void*) lde_if_f);

        default:
            return _dw_read_reg(LDE_IF, LDE_REPC, (void*) lde_if_f) &&
                   _dw_read_reg(LDE_IF, LDE_CFG2, (void*) lde_if_f) &&
                   _dw_read_reg(LDE_IF, LDE_RXANTD, (void*) lde_if_f) &&
                   _dw_read_reg(LDE_IF, LDE_PPAMPL, (void*) lde_if_f) &&
                   _dw_read_reg(LDE_IF, LDE_PPINDX, (void*) lde_if_f) &&
                   _dw_read_reg(LDE_IF, LDE_CFG1, (void*) lde_if_f) &&
                   _dw_read_reg(LDE_IF, LDE_THRESH, (void*) lde_if_f);

    }

}

bool set_lde_if(lde_if_format* lde_if_f, const lde_if_subregister subregister) {

    switch(subregister) {

        case LDE_REPC:
            return _dw_write_reg(LDE_IF, LDE_REPC, (void*) lde_if_f);

        case LDE_CFG2:
            return _dw_write_reg(LDE_IF, LDE_CFG2, (void*) lde_if_f);

        case LDE_RXANTD:
            return _dw_write_reg(LDE_IF, LDE_RXANTD, (void*) lde_if_f);

        case LDE_PPAMPL:
            return false;

        case LDE_PPINDX:
            return false;

        case LDE_CFG1:
            return _dw_write_reg(LDE_IF, LDE_CFG1, (void*) lde_if_f);

        case LDE_THRESH:
            return false;

        default:
            return _dw_write_reg(LDE_IF, LDE_REPC, (void*) lde_if_f) &&
                   _dw_write_reg(LDE_IF, LDE_CFG2, (void*) lde_if_f) &&
                   _dw_write_reg(LDE_IF, LDE_RXANTD, (void*) lde_if_f) &&
                   _dw_write_reg(LDE_IF, LDE_CFG1, (void*) lde_if_f);

    }

}

bool get_dig_diag(dig_diag_format* dig_diag_f, const dig_diag_subregister subregister) {

    switch(subregister) {

        case DIAG_TMC:
            return _dw_read_reg(DIG_DIAG, DIAG_TMC, (void*) dig_diag_f);

        case EVC_TPW:
            return _dw_read_reg(DIG_DIAG, EVC_TPW, (void*) dig_diag_f);

        case EVC_HPW:
            return _dw_read_reg(DIG_DIAG, EVC_HPW, (void*) dig_diag_f);

        case EVC_TXFS:
            return _dw_read_reg(DIG_DIAG, EVC_TXFS, (void*) dig_diag_f);

        case EVC_FWTO:
            return _dw_read_reg(DIG_DIAG, EVC_FWTO, (void*) dig_diag_f);

        case EVC_PTO:
            return _dw_read_reg(DIG_DIAG, EVC_PTO, (void*) dig_diag_f);

        case EVC_STO:
            return _dw_read_reg(DIG_DIAG, EVC_STO, (void*) dig_diag_f);

        case EVC_OVR:
            return _dw_read_reg(DIG_DIAG, EVC_OVR, (void*) dig_diag_f);

        case EVC_FFR:
            return _dw_read_reg(DIG_DIAG, EVC_FFR, (void*) dig_diag_f);

        case EVC_FCE:
            return _dw_read_reg(DIG_DIAG, EVC_FCE, (void*) dig_diag_f);

        case EVC_FCG:
            return _dw_read_reg(DIG_DIAG, EVC_FCG, (void*) dig_diag_f);

        case EVC_RSE:
            return _dw_read_reg(DIG_DIAG, EVC_RSE, (void*) dig_diag_f);

        case EVC_PHE:
            return _dw_read_reg(DIG_DIAG, EVC_PHE, (void*) dig_diag_f);

        case EVC_CTRL:
            return _dw_read_reg(DIG_DIAG, EVC_CTRL, (void*) dig_diag_f);

        default:
            return _dw_read_reg(DIG_DIAG, DIAG_TMC, (void*) dig_diag_f) &&
                   _dw_read_reg(DIG_DIAG, EVC_TPW, (void*) dig_diag_f) &&
                   _dw_read_reg(DIG_DIAG, EVC_HPW, (void*) dig_diag_f) &&
                   _dw_read_reg(DIG_DIAG, EVC_TXFS, (void*) dig_diag_f) &&
                   _dw_read_reg(DIG_DIAG, EVC_FWTO, (void*) dig_diag_f) &&
                   _dw_read_reg(DIG_DIAG, EVC_PTO, (void*) dig_diag_f) &&
                   _dw_read_reg(DIG_DIAG, EVC_STO, (void*) dig_diag_f) &&
                   _dw_read_reg(DIG_DIAG, EVC_OVR, (void*) dig_diag_f) &&
                   _dw_read_reg(DIG_DIAG, EVC_FFR, (void*) dig_diag_f) &&
                   _dw_read_reg(DIG_DIAG, EVC_FCE, (void*) dig_diag_f) &&
                   _dw_read_reg(DIG_DIAG, EVC_FCG, (void*) dig_diag_f) &&
                   _dw_read_reg(DIG_DIAG, EVC_RSE, (void*) dig_diag_f) &&
                   _dw_read_reg(DIG_DIAG, EVC_PHE, (void*) dig_diag_f) &&
                   _dw_read_reg(DIG_DIAG, EVC_CTRL, (void*) dig_diag_f);
    }

}

bool set_dig_diag(dig_diag_format* dig_diag_f, const dig_diag_subregister subregister) {

    switch(subregister) {

        case DIAG_TMC:
            return _dw_write_reg(DIG_DIAG, DIAG_TMC, (void*) dig_diag_f);

        case EVC_TPW:
            return false;

        case EVC_HPW:
            return false;

        case EVC_TXFS:
            return false;

        case EVC_FWTO:
            return false;

        case EVC_PTO:
            return false;

        case EVC_STO:
            return false;

        case EVC_OVR:
            return false;

        case EVC_FFR:
            return false;

        case EVC_FCE:
            return false;

        case EVC_FCG:
            return false;

        case EVC_RSE:
            return false;

        case EVC_PHE:
            return false;

        case EVC_CTRL:
            return _dw_write_reg(DIG_DIAG, EVC_CTRL, (void*) dig_diag_f);

        default:
            return _dw_write_reg(DIG_DIAG, DIAG_TMC, (void*) dig_diag_f) &&
                   _dw_write_reg(DIG_DIAG, EVC_CTRL, (void*) dig_diag_f);
    }

}

bool get_pmsc(pmsc_format* pmsc_f, const pmsc_subregister subregister) {

    switch(subregister) {

        case PMSC_LEDC:
            return _dw_read_reg(PMSC, PMSC_LEDC, (void*) pmsc_f);

        case PMSC_TXFSEQ:
            return _dw_read_reg(PMSC, PMSC_TXFSEQ, (void*) pmsc_f);

        case PMSC_SNOZT:
            return _dw_read_reg(PMSC, PMSC_SNOZT, (void*) pmsc_f);

        case PMSC_CTRL1:
            return _dw_read_reg(PMSC, PMSC_CTRL1, (void*) pmsc_f);

        case PMSC_CTRL0:
            return _dw_read_reg(PMSC, PMSC_CTRL0, (void*) pmsc_f);

        default:
            return _dw_read_reg(PMSC, PMSC_LEDC, (void*) pmsc_f) &&
                   _dw_read_reg(PMSC, PMSC_TXFSEQ, (void*) pmsc_f) &&
                   _dw_read_reg(PMSC, PMSC_SNOZT, (void*) pmsc_f) &&
                   _dw_read_reg(PMSC, PMSC_CTRL1, (void*) pmsc_f) &&
                   _dw_read_reg(PMSC, PMSC_CTRL0, (void*) pmsc_f);
    }

}

bool set_pmsc(pmsc_format* pmsc_f, const pmsc_subregister subregister) {

    switch(subregister) {

        case PMSC_LEDC:
            return _dw_write_reg(PMSC, PMSC_LEDC, (void*) pmsc_f);

        case PMSC_TXFSEQ:
            return _dw_write_reg(PMSC, PMSC_TXFSEQ, (void*) pmsc_f);

        case PMSC_SNOZT:
            return _dw_write_reg(PMSC, PMSC_SNOZT, (void*) pmsc_f);

        case PMSC_CTRL1:
            return _dw_write_reg(PMSC, PMSC_CTRL1, (void*) pmsc_f);

        case PMSC_CTRL0:
            return _dw_write_reg(PMSC, PMSC_CTRL0, (void*) pmsc_f);

        default:
            return _dw_write_reg(PMSC, PMSC_LEDC, (void*) pmsc_f) &&
                   _dw_write_reg(PMSC, PMSC_TXFSEQ, (void*) pmsc_f) &&
                   _dw_write_reg(PMSC, PMSC_SNOZT, (void*) pmsc_f) &&
                   _dw_write_reg(PMSC, PMSC_CTRL1, (void*) pmsc_f) &&
                   _dw_write_reg(PMSC, PMSC_CTRL0, (void*) pmsc_f);
    }

}
