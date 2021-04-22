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
    void (*formater)(spi_frame f, void* structure, const size_t sub_register);     // Pointer to the function that formats the spi_frame.
    size_t (*unformater)(void* structure, spi_frame f, const size_t sub_register); // Pointer to the function that unformats the spi_frame.
} command;

#define RESERVED_REGISTER {0U, RE, NULL, NULL}

// Structure which contains all the registers information of the dw1000.
const command COMMAND_PANEL[] = {
        {4U, RO, dev_id_formater, NULL},                        // DEV_ID
        {8U, RW, eui_formater, eui_unformater},                 // EUI
        RESERVED_REGISTER,                                      // RESERVED_1
        {4U, RW, pan_addr_formater, pan_addr_unformater},       // PAN_ADR
        {4U, RW, sys_cfg_formater, sys_cfg_unformater},         // SYS_CFG
        RESERVED_REGISTER,                                      // RESERVED_2
        {5U, RO, sys_time_formater, NULL},                      // SYS_TIME
        RESERVED_REGISTER,                                      // RESERVED_3
        {4U, RW, tx_fctrl_formater, tx_fctrl_unformater},       // TX_FCTRL
        {1024U, WO, NULL, tx_buffer_unformater},                // TX_BUFFER
        {5U, RW, dx_time_formater, dx_time_unformater},         // DX_TIME
        RESERVED_REGISTER,                                      // RESERVED_4
        {2U, RW, rx_fwto_formater, rx_fwto_unformater},         // RX_FWTO
        {4U, RW, tx_ctrl_formater, tx_ctrl_unformater},         // SYS_CTRL
        {4U, RW, sys_evt_msk_formater, sys_evt_msk_unformater}, // SYS_MASK
        {4U, RW, sys_evt_sts_formater, sys_evt_sts_unformater}, // SYS_STATUS
        {4U, RO, rx_finfo_formater, NULL},                      // RX_FINFO
        {1024U, RO, rx_buffer_formater, NULL},                  // RX_BUFFER
        {4U, RO, rx_fqual_formater, NULL},                      // RX_FQUAL
        {4U, RO, rx_ttcki_formater, NULL},                      // RX_TTCKI
        {4U, RO, rx_ttcko_formater, NULL},                      // RX_TTCKO
        {4U, RO, rx_time_formater, NULL},                       // RX_TIME
        RESERVED_REGISTER,                                      // RESERVED_5
        {4U, RO, tx_time_formater, NULL},                       // TX_TIME
        {2U, RW, tx_antd_formater, tx_antd_unformater},         // TX_ANTD
        {4U, RO, sys_state_formater, NULL},                     // SYS_STATE
        {4U, RW, ack_resp_t_formater, ack_resp_t_unformater},   // ACK_RESP_T
        RESERVED_REGISTER,                                      // RESERVED_6
        RESERVED_REGISTER,                                      // RESERVED_7
        {4U, RW, rx_sniff_formater, rx_sniff_unformater},       // RX_SNIFF
        {4U, RW, tx_power_formater, tx_power_unformater},       // TX_POWER
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
bool is_access_permission_valid(const register_id id, const bool read) {

    switch(COMMAND_PANEL[id].ra) {
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
bool is_id_valid(const register_id id) {
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
bool is_command_valid(const register_id id, const bool read) {
    return is_id_valid(id) && is_access_permission_valid(id, read);
}

/**
 * @brief Prepare SPI header frame to be send.
 *
 * @param[in] reg_id: Register identifier.
 * @param[in] offset: Sub-register index.
 * @param[in] read: If the operation will be a read operation.
 * @param[out] header: SPI header frame formed.
 *
 * @return size_t: Size of the SPI header frame. If this function returns
 *                 zero that means that the given parameters was invalid.
 *
 */
size_t compose_spi_header(const register_id reg_id, size_t offset, const bool read, spi_frame header) {

    size_t spi_header_size = 0;

    if(is_command_valid(reg_id, read)) {

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
bool dw_read_reg(const register_id reg_id, const size_t offset, void* parsed_reg) {

    uint8_t header[3] = {0X00, 0X00, 0X00};
    size_t spi_header_size = compose_spi_header(reg_id, offset, true, header);

    if(spi_header_size > 0) {

        command c = COMMAND_PANEL[reg_id];

        uint8_t rx[c.rx_buf_size];

        spiAcquireBus(&SPID1);
        spiSelect(&SPID1);

        spiSend(&SPID1, spi_header_size, header);
        spiReceive(&SPID1, c.rx_buf_size, rx);

        spiUnselect(&SPID1);
        spiReleaseBus(&SPID1);

        c.formater(rx, parsed_reg, offset);

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
bool dw_write_reg(const register_id reg_id, const size_t offset, void* parsed_reg) {

    uint8_t header[3] = {0X00, 0X00, 0X00};
    size_t spi_header_size = compose_spi_header(reg_id, offset, false, header);

    if(spi_header_size > 0) {

        command c = COMMAND_PANEL[reg_id];

        uint8_t data[c.rx_buf_size];
        size_t data_size = c.unformater(parsed_reg, data, offset);

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
    return dw_read_reg(DEV_ID, 0, (void*) dev_id_f);
}

bool get_eui(eui_format* eui_f) {
    return dw_read_reg(EUI, 0, (void*) eui_f);
}

bool set_eui(eui_format* eui_f) {
    return dw_write_reg(EUI, 0, (void*) eui_f);
}

bool get_pan_adr(pan_adr_format *pan_adr_f) {
    return dw_read_reg(PAN_ADR, 0, (void*) pan_adr_f);
}

bool set_pan_adr(pan_adr_format* pan_adr_f) {
    return dw_write_reg(PAN_ADR, PAN_ID, (void*) pan_adr_f) &&
           dw_write_reg(PAN_ADR, SHORT_ADR, (void*) pan_adr_f);
}

bool get_sys_cfg(sys_cfg_format* sys_cfg_f) {
    return dw_read_reg(SYS_CFG, 0, (void*) sys_cfg_f);
}

bool set_sys_cfg(sys_cfg_format* sys_cfg_f) {
    return dw_write_reg(SYS_CFG, 0, (void*) sys_cfg_f);
}

bool get_sys_time(double* seconds) {
    return dw_read_reg(SYS_TIME, 0, (void*) seconds);
}

bool get_tx_fctrl(tx_fctrl_format* tx_fctrl_f) {
    return dw_read_reg(TX_FCTRL, REST, (void*) tx_fctrl_f) &&
           dw_read_reg(TX_FCTRL, IFSDELAY, (void*) tx_fctrl_f);
}

bool set_tx_fctrl(tx_fctrl_format* tx_fctrl_f) {
    return dw_write_reg(TX_FCTRL, REST, (void*) tx_fctrl_f) &&
           dw_write_reg(TX_FCTRL, IFSDELAY, (void*) tx_fctrl_f);
}

bool set_tx_buffer(uwb_frame frame, const uint16_t frame_size) {

    const uint16_t offset = TX_RX_BUFFER_MAX_SIZE - frame_size;
    bool ret = false;

    if(offset + frame_size <= TX_RX_BUFFER_MAX_SIZE) {
        ret = dw_write_reg(TX_BUFFER, offset, (void*) frame);
    }

    return ret;

}

bool get_dx_time(double* seconds) {
    return dw_read_reg(DX_TIME, 0, (void*) seconds);
}

bool set_dx_time(double* seconds) {
    return (*seconds <= DTR_COUNTER_WRAP_PERIOD) &&
           (*seconds >= 0.0) && dw_write_reg(DX_TIME, 0, (void*) seconds);
}

bool get_rx_fwto(double* seconds) {
    return dw_read_reg(RX_FWTO, 0, (void*) seconds);
}

bool set_rx_fwto(double* seconds) {
    return (*seconds <= RFR_COUNTER_WRAP_PERIOD) &&
           (*seconds >= 0.0) && dw_write_reg(RX_FWTO, 0, (void*) seconds);
}

bool get_sys_ctrl(sys_ctrl_format* sys_ctrl_f) {
    return dw_read_reg(SYS_CTRL, 0, (void*) sys_ctrl_f);
}

bool set_sys_ctrl(sys_ctrl_format* sys_ctrl_f) {
    return dw_write_reg(SYS_CTRL, 0, (void*) sys_ctrl_f);
}

bool get_sys_event_msk(sys_evt_msk_format* sys_evt_msk_f) {
    return dw_read_reg(SYS_MASK, 0, (void*) sys_evt_msk_f);
}

bool set_sys_event_msk(sys_evt_msk_format* sys_evt_msk_f) {
    return dw_write_reg(SYS_MASK, 0, (void*) sys_evt_msk_f);
}

bool get_sys_event_sts(sys_evt_sts_format* sys_evt_sts_f) {
    return dw_read_reg(SYS_STATUS, SES_OCT_0_TO_3, (void*) sys_evt_sts_f) &&
           dw_read_reg(SYS_STATUS, SES_OCT_4, (void*) sys_evt_sts_f);
}

bool set_sys_event_sts(sys_evt_sts_format* sys_evt_sts_f) {
    return dw_write_reg(SYS_STATUS, SES_OCT_0_TO_3, (void*) sys_evt_sts_f) &&
           dw_write_reg(SYS_STATUS, SES_OCT_4, (void*) sys_evt_sts_f);
}

bool get_rx_finfo(rx_finfo_format* rx_finfo_f) {
    return dw_read_reg(RX_FINFO, 0, (void*) rx_finfo_f);
}

bool get_rx_buffer(uwb_frame frame, const uint16_t frame_size) {

    const uint16_t offset = TX_RX_BUFFER_MAX_SIZE - frame_size;
    bool ret = false;

    if(offset + frame_size <= TX_RX_BUFFER_MAX_SIZE) {
        ret = dw_read_reg(RX_BUFFER, offset, (void*) frame);
    }

    return ret;
}

bool get_rx_fqual(rx_fqual_format* rx_fqual_f) {
    return dw_read_reg(RX_FQUAL, FP_AMPL2_STD_NOISE, (void*) rx_fqual_f) &&
           dw_read_reg(RX_FQUAL, CIR_PWR_PP_AMPL3, (void*) rx_fqual_f);
}

bool get_rx_ttcki(rx_ttcki_value* rx_ttcki) {
    return dw_read_reg(RX_TTCKI, 0, (void*) rx_ttcki);
}

bool get_rx_ttcko(rx_ttcko_format* rx_ttcko_f) {
    return dw_read_reg(RX_TTCKO, RX_TTCKO_OCT_0_TO_3, (void*) rx_ttcko_f) &&
           dw_read_reg(RX_TTCKO, RX_TTCKO_OCT_4, (void*) rx_ttcko_f) ;
}

bool get_rx_time(rx_time_format* rx_time_f) {
    return dw_read_reg(RX_TIME, RX_TIME_OCT_0_TO_3, (void*) rx_time_f) &&
           dw_read_reg(RX_TIME, RX_TIME_OCT_4_TO_7, (void*) rx_time_f) &&
           dw_read_reg(RX_TIME, RX_TIME_OCT_8_TO_11, (void*) rx_time_f) &&
           dw_read_reg(RX_TIME, RX_TIME_OCT_12_TO_13, (void*) rx_time_f);
}

bool get_tx_time(tx_time_format* tx_time_f) {
    return dw_read_reg(TX_TIME, TX_TIME_OCT_0_TO_3, (void*) tx_time_f) &&
           dw_read_reg(TX_TIME, TX_TIME_OCT_4_TO_7, (void*) tx_time_f) &&
           dw_read_reg(TX_TIME, TX_TIME_OCT_8_TO_9, (void*) tx_time_f);
}

bool get_tx_antd(double* seconds) {
    return dw_read_reg(TX_ANTD, 0, (void*) seconds);
}

bool set_tx_antd(double* seconds) {
    return (*seconds <= TX_ANTD_WRAP_PERIOD) &&
           (*seconds >= 0.0) && dw_write_reg(TX_ANTD, 0, (void*) seconds);
}

bool get_sys_status(sys_status_format* sys_status_f) {
    return dw_read_reg(SYS_STATUS, 0, (void*) sys_status_f);
}

bool get_ack_resp_t(ack_resp_t_format* ack_resp_t_f) {
    return dw_read_reg(ACK_RESP_T, 0, (void*) ack_resp_t_f);
}

bool set_ack_resp_t(ack_resp_t_format* ack_resp_t_f) {
    return (ack_resp_t_f->w4r_tim <= W4RTIM_WRAP_PERIOD) &&
           (ack_resp_t_f->w4r_tim >= 0.0) &&
           dw_write_reg(ACK_RESP_T, 0, (void*) ack_resp_t_f);
}

bool get_rx_sniff(rx_sniff_format* rx_sniff_f) {
    return dw_read_reg(RX_SNIFF, 0, (void*) rx_sniff_f);
}

bool set_rx_sniff(rx_sniff_format* rx_sniff_f) {
    return (rx_sniff_f->sniff_offt <= SNIFF_OFFT_WRAP_PERIOD) &&
           (rx_sniff_f->sniff_offt >= 0.0) &&
           dw_write_reg(RX_SNIFF, 0, (void*) rx_sniff_f);
}

bool get_tx_power(tx_power_format* tx_power_f) {
    return dw_read_reg(TX_POWER, 0, (void*) tx_power_f);
}

bool set_tx_power(tx_power_format* tx_power_f) {
    return dw_write_reg(TX_POWER, 0, (void*) tx_power_f);
}
