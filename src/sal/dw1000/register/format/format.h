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

#ifndef _FORMAT_H_
#define _FORMAT_H_

#include <stdint.h>
#include <stddef.h>
#include <math.h>

// Defines frames of the SPI dwm1000 communications.
typedef uint8_t *spi_frame;

/******* DEV_ID *******/

// Structure of the device identifier register fields.
typedef struct {
    unsigned int ridtag :16; // Register identification tag.
    unsigned int model :8;   // Device model.
    unsigned int ver :4;     // Version.
    unsigned int rev :4;     // Revision.
} dev_id_format;

/**
 * @brief Formats a spi_frame to a dev_id_format structure.
 *
 * @param[in] fr: The spi_frame to initialize the structure.
 * @param[out] format: Structure which will contains the spi_frame formatted.
 *
 * @note: This function must receive an dev_id_format structure to work.
 *
 */
void dev_id_formater(spi_frame fr, void *format);

/******* EUI *******/

// Structure of the extended unique identifier register fields.
typedef struct {
    uint64_t ext_ID :40;    // Extension ID.
    unsigned int mc_ID :24; // Manufacturer company ID.
} eui_format;

/**
 * @brief Formats a spi_frame to an eui_format structure.
 *
 * @param[in] fr: The spi_frame to initialize the structure.
 * @param[out] format: Structure which will contains the spi_frame formatted.
 *
 * @note: This function must receive an eui_format structure to work.
 *
 */
void eui_formater(spi_frame fr, void *format);

/**
 * @brief Formats an eui_format structure to a spi_frame.
 *
 * @param[in] format: Structure which contains the values of the fields of the EUI register.
 * @param[out] fr: spi_frame where this functions will load the eui_format structure.
 *
 * @return size_t: Size of the spi_frame formed with the given eui_format structure.
 *
 * @note: This function must receive an eui_format structure to work.
 *
 */
size_t eui_unformater(void *format, spi_frame fr);

/******* PANADR *******/

// Sub-registers of the pan address register.
typedef enum {
    SHORT_ADR = 0, PAN_ID = 2,
} pan_field;

// Structure of the extended unique identifier register fields.
typedef struct {
    uint16_t pan_id;
    uint16_t short_addr;
} pan_adr_format;

/**
 * @brief Formats a spi_frame to a pan_addr_formater structure.
 *
 * @param[in] fr: The spi_frame to initialize the structure.
 * @param[out] format: Structure which will contains the spi_frame formatted.
 *
 * @note: This function must receive an pan_adr_format structure to work.
 *
 */
void pan_addr_formater(spi_frame fr, void *format);

/**
 * @brief Formats a pan_adr_format structure to a spi_frame.
 *
 * @param[in] format: Structure which contains the values of the fields of the PAN_ADR register.
 * @param[out] fr: spi_frame where this functions will load the pan_adr_format structure.
 *
 * @return size_t: Size of the spi_frame formed with the given pan_adr_format structure.
 *
 * @note: This function must receive an pan_adr_format structure to work.
 *
 */
size_t pan_addr_unformater(void *format, spi_frame fr);

/******* SYS_CFG *******/

// Structure of the system configuration register fields.
typedef struct {
    unsigned int aackpend :1;   // Automatic acknowledgment pending bit control.
    unsigned int autoack :1;    // Automatic acknowledgment enable.
    unsigned int rxautr :1;     // Receiver auto-re-enable.
    unsigned int rxwtoe :1;     // Receive wait timeout enable.
    unsigned int rxm110k :1;    // Receiver mode 110 kbps data rate.
    unsigned int dis_stxp :1;   // Disable smart TX power control.
    unsigned int phr_mode :2;   // This configuration allows selection of PHR type to be one of two options.
    unsigned int fcs_init2f :1; // This bit allows selection of the initial seed value for the FCS generation and checking
                                // function that is set at the start of each frame transmission and reception.
    unsigned int dis_rsde :1;   // Disable receiver abort on RSD error.
    unsigned int dis_phe :1;    // Disable receiver abort on PHR error.
    unsigned int dis_drxb :1;   // Disable double RX Buffer.
    unsigned int dis_fce :1;    // Disable frame check error handling.
    unsigned int spi_edge :1;   // SPI data launch edge.
    unsigned int hirq_pol :1;   // Host interrupt polarity.
    unsigned int ffa5 :1;       // Frame filtering allow frames with frame type field of 5.
    unsigned int ffa4 :1;       // Frame filtering allow frames with frame type field of 4.
    unsigned int ffar :1;       // Frame filtering allow reserved frame types.
    unsigned int ffam :1;       // Frame filtering allow MAC command frame reception.
    unsigned int ffaa :1;       // Frame filtering allow acknowledgment frame reception.
    unsigned int ffad :1;       // Frame filtering allow data frame reception.
    unsigned int ffab :1;       // Frame filtering allow beacon frame reception.
    unsigned int ffbc :1;       // Frame filtering behave as a coordinator.
    unsigned int ffen :1;       // Frame filtering enable.
} sys_cfg_format;

/**
 * @brief Formats a spi_frame to a sys_cfg_formater structure.
 *
 * @param[in] fr: The spi_frame to initialize the structure.
 * @param[out] format: Structure which will contains the spi_frame formatted.
 *
 * @note: This function must receive an sys_cfg_format structure to work.
 *
 */
void sys_cfg_formater(spi_frame fr, void *format);

/**
 * @brief Formats a sys_cfg_format structure to a spi_frame.
 *
 * @param[in] format: Structure which contains the values of the fields of the SYS_CFG register.
 * @param[out] fr: spi_frame where this functions will load the sys_cfg_format structure.
 *
 * @return size_t: Size of the spi_frame formed with the given sys_cfg_format structure.
 *
 * @note: This function must receive an sys_cfg_format structure to work.
 *
 */
size_t sys_cfg_unformater(void *format, spi_frame fr);

/******* SYS_TIME *******/
extern const double COUNTER_WRAP_PERIOD;
extern const double COUNTER_RESOLUTION;
#define calculate_seconds(register_value) register_value * (COUNTER_WRAP_PERIOD/COUNTER_RESOLUTION)

/**
 * @brief Formats a spi_frame to a double.
 *
 * @param[in] fr: The spi_frame to initialize the double value.
 * @param[out] format: Double which will contains the spi_frame formatted.
 *
 * @note: This function must receive an double to work.
 * @note: The returned value will be in units of seconds.
 *
 */
void sys_time_formater(spi_frame fr, void *format);

/******* TX_FCTRL *******/
typedef enum {
    KBPS110 = 0b00,
    KBPS850 = 0b01,
    MBPS6_8 = 0b10,
    TXPRF_RESERVED = 0b10, // This value should be 0b11, but this is to prevent used it.
} transmit_bit_rate;

typedef enum {
    MHZ4 = 0b00,
    MHZ16 = 0b01,
    MHZ64 = 0b10,
    TXBR_RESERVED = 0b10 // This value should be 0b11, but this is to prevent used it.
} transmit_pulse_repetition_freq;

typedef enum {
    L_64 = 0b0001,
    L_128 = 0b0101,
    L_256 = 0b1001,
    L_512 = 0b1101,
    L_1024 = 0b0010,
    L_1536 = 0b0110,
    L_2048 = 0b1010,
    L_4096 = 0b0011,
} preamble_lenght_selection;

typedef struct {
    unsigned int txboffs :10;             // Transmit buffer index offset.
    unsigned int pe_txpsr : 4;            // Preamble extension and transmit preamble symbol repetitions.
    transmit_pulse_repetition_freq txprf; // Transmit pulse repetition frequency.
    unsigned int tr :1;                   // Transmit ranging enable.
    transmit_bit_rate txbr;               // Transmit bit rate.
    unsigned int tfle :3;                 // Transmit frame length extension.
    unsigned int tflen :7;                // Transmit frame length.
    unsigned int ifsdelay :8;             // Inter-frame spacing.
} tx_fctrl_format;

/**
 * @brief Formats a spi_frame to a tx_fctrl_format structure.
 *
 * @param[in] fr: The spi_frame to initialize the structure.
 * @param[out] format: Structure which will contains the spi_frame formatted.
 *
 * @note: This function must receive an tx_fctrl_format structure to work.
 *
 */
void tx_fctrl_formater(spi_frame fr, void *format);

/**
 * @brief Formats a tx_fctrl_format structure to a spi_frame.
 *
 * @param[in] format: Structure which contains the values of the fields of the TX_FCTRL register.
 * @param[out] fr: spi_frame where this functions will load the sys_cfg_format structure.
 *
 * @return size_t: Size of the spi_frame formed with the given tx_fctrl_format structure.
 *
 * @note: This function must receive an tx_fctrl_format structure to work.
 *
 */
size_t tx_fctrl_unformater(void *format, spi_frame fr);

#endif // _FORMAT_H_
