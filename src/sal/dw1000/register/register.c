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
