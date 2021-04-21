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

// Defines frames of the UWB dwm1000 communications.
typedef uint8_t *uwb_frame;

/******* DEV_ID *******/

// Structure of the device identifier register.
typedef struct {
    uint16_t ridtag;     // Register identification tag.
    uint8_t model;       // Device model.
    unsigned int ver :4; // Version.
    unsigned int rev :4; // Revision.
} dev_id_format;

/**
 * @brief Formats a spi_frame to a dev_id format structure.
 *
 * @param[in] fr: The spi_frame to initialize the structure.
 * @param[out] format: Structure which will contains the spi_frame formatted.
 * @param[in] sub_register: Enumerate that indicates the sub-register.
 *
 * @note: This function must receive a dev_id_format structure to work.
 *
 */
void dev_id_formater(spi_frame fr, void *format, const size_t sub_register);

/******* EUI *******/

// Structure of the extended unique identifier register.
typedef struct {
    uint64_t ext_ID :40;    // Extension ID.
    unsigned int mc_ID :24; // Manufacturer company ID.
} eui_format;

/**
 * @brief Formats a spi_frame to an eui_format structure.
 *
 * @param[in] fr: The spi_frame to initialize the structure.
 * @param[out] format: Structure which will contains the spi_frame formatted.
 * @param[in] sub_register: Enumerate that indicates the sub-register.
 *
 * @note: This function must receive an eui_format structure to work.
 *
 */
void eui_formater(spi_frame fr, void *format, const size_t sub_register);

/**
 * @brief Formats an eui_format structure to a spi_frame.
 *
 * @param[in] format: Structure which contains the values of the fields of the EUI register.
 * @param[out] fr: spi_frame where this function will store the eui_format structure.
 * @param[in] sub_register: Enumerate that indicates the sub-register.
 *
 * @return size_t: Size of the spi_frame formed with the given eui_format structure.
 *
 * @note: This function must receive an eui_format structure to work.
 *
 */
size_t eui_unformater(void *format, spi_frame fr, const size_t sub_register);

/******* PANADR *******/

// Sub-registers of the pan address register.
typedef enum {
    SHORT_ADR = 0,
    PAN_ID = 2,
} pan_adr_subregister;

// Structure of the extended unique identifier register.
typedef struct {
    uint16_t pan_id;
    uint16_t short_addr;
} pan_adr_format;

/**
 * @brief Formats a spi_frame to a pan_addr_format structure.
 *
 * @param[in] fr: The spi_frame to initialize the structure.
 * @param[out] format: Structure which will contains the spi_frame formatted.
 * @param[in] sub_register: Enumerate that indicates the sub-register.
 *
 * @note: This function must receive a pan_adr_format structure to work.
 *
 */
void pan_addr_formater(spi_frame fr, void *format, const size_t sub_register);

/**
 * @brief Formats a pan_adr_format structure to a spi_frame.
 *
 * @param[in] format: Structure which contains the values of the fields of the PAN_ADR register.
 * @param[out] fr: spi_frame where this function will store the pan_adr_format structure.
 * @param[in] sub_register: Enumerate that indicates the sub-register.
 *
 * @return size_t: Size of the spi_frame formed with the given pan_adr_format structure.
 *
 * @note: This function must receive a pan_adr_format structure to work.
 *
 */
size_t pan_addr_unformater(void *format, spi_frame fr, const size_t sub_register);

/******* SYS_CFG *******/

// Structure of the system configuration register.
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
 * @note: This function must receive a sys_cfg_format structure to work.
 *
 */
void sys_cfg_formater(spi_frame fr, void *format, const size_t sub_register);

/**
 * @brief Formats a sys_cfg_format structure to a spi_frame.
 *
 * @param[in] format: Structure which contains the values of the fields of the SYS_CFG register.
 * @param[out] fr: spi_frame where this function will store the sys_cfg_format structure.
 * @param[in] sub_register: Enumerate that indicates the sub-register.
 *
 * @return size_t: Size of the spi_frame formed with the given sys_cfg_format structure.
 *
 * @note: This function must receive a sys_cfg_format structure to work.
 *
 */
size_t sys_cfg_unformater(void *format, spi_frame fr, const size_t sub_register);

/******* SYS_TIME *******/
#define STR_COUNTER_WRAP_PERIOD 17.2074 // (seconds)
#define STR_COUNTER_RESOLUTION pow(2, 40)
#define STR_K STR_COUNTER_WRAP_PERIOD/STR_COUNTER_RESOLUTION
#define str_calculate_seconds(register_value) register_value * STR_K

/**
 * @brief Formats a spi_frame to a double.
 *
 * @param[in] fr: The spi_frame to initialize the double value.
 * @param[out] format: Double which will contains the spi_frame formatted.
 * @param[in] sub_register: Enumerate that indicates the sub-register.
 *
 * @note: This function must receive an double to work.
 * @note: The returned value will be in units of seconds.
 *
 */
void sys_time_formater(spi_frame fr, void *format, const size_t sub_register);

/******* TX_FCTRL *******/

// Possible values of the transmit/receive bit rate field.
typedef enum {
    KBPS110 = 0b00,
    KBPS850 = 0b01,
    MBPS6_8 = 0b10,
    TXPRF_RESERVED = 0b10, // This value should be 0b11, but this is to prevent used it.
} transmit_receive_bit_rate;

// Possible values of the transmit/receive pulse repetition frequency field.
typedef enum {
    TRPR_MHZ4 = 0b00,
    TRPR_MHZ16 = 0b01,
    TRPR_MHZ64 = 0b10,
    TRPR_RESERVED = 0b10 // This value should be 0b11, but this is to prevent used it.
} transmit_receive_pulse_repetition_freq;

// Possible values of the preamble extension and transmit preamble symbol repetitions fields.
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

// Structure of the transmit frame control register.
typedef struct {
    unsigned int txboffs :10;                      // Transmit buffer index offset.
    preamble_lenght_selection pe_txpsr :4;         // Preamble extension and transmit preamble symbol repetitions.
    transmit_receive_pulse_repetition_freq txprf;  // Transmit pulse repetition frequency.
    unsigned int tr :1;                            // Transmit ranging enable.
    transmit_receive_bit_rate txbr;                // Transmit bit rate.
    unsigned int tfle :3;                          // Transmit frame length extension.
    unsigned int tflen :7;                         // Transmit frame length.
    uint8_t ifsdelay;                              // Inter-frame spacing.
} tx_fctrl_format;

// Fields of the transmit frame control register.
typedef enum {
    IFSDELAY = 4,
    REST = 0
} tx_fctrl_subregister;

/**
 * @brief Formats a spi_frame to a tx_fctrl_format structure.
 *
 * @param[in] fr: The spi_frame to initialize the structure.
 * @param[out] format: Structure which will contains the spi_frame formatted.
 * @param[in] sub_register: Enumerate that indicates the sub-register.
 *
 * @note: This function must receive a tx_fctrl_format structure to work.
 *
 */
void tx_fctrl_formater(spi_frame fr, void *format, const size_t sub_register);

/**
 * @brief Formats a tx_fctrl_format structure to a spi_frame.
 *
 * @param[in] format: Structure which contains the values of the fields of the TX_FCTRL register.
 * @param[out] fr: spi_frame where this function will store the sys_cfg_format structure.
 * @param[in] sub_register: Enumerate that indicates the sub-register.
 *
 * @return size_t: Size of the spi_frame formed with the given tx_fctrl_format structure.
 *
 * @note: This function must receive a tx_fctrl_format structure to work.
 *
 */
size_t tx_fctrl_unformater(void *format, spi_frame fr, const size_t sub_register);

/******* TX_BUFFER *******/

#define TX_RX_BUFFER_MAX_SIZE 128

/**
 * @brief Formats an uint8_t* to a spi_frame.
 *
 * @param[in] format: Array which contains the values of the fields of the TX_BUFFER register.
 * @param[out] fr: spi_frame where this function will store the array(format).
 * @param[in] sub_register: Enumerate that indicates the sub-register.
 *
 * @return size_t: Size of the spi_frame formed with the given array.
 *
 * @note: This function must receive a uint8_t* array to work.
 *
 */
size_t tx_buffer_unformater(void *format, spi_frame fr, const size_t sub_register);

/******* DX_TIME *******/

#define DTR_COUNTER_WRAP_PERIOD 8.012820513 * pow(10, -9) // (seconds)
#define DTR_COUNTER_RESOLUTION pow(2, 40)
#define DTR_K (DTR_COUNTER_WRAP_PERIOD/DTR_COUNTER_RESOLUTION)
#define dtr_calculate_seconds(register_value) register_value * DTR_K
#define dtr_calculate_register_val(seconds) seconds / DTR_K

/**
 * @brief Formats a spi_frame to a double
 *
 * @param[in] fr: The spi_frame to initialize the double.
 * @param[out] format: Double which will contains the spi_frame formatted.
 * @param[in] sub_register: Enumerate that indicates the sub-register.
 *
 * @note: This function must receive a double to work.
 *
 */
void dx_time_formater(spi_frame fr, void *format, const size_t sub_register);

/**
 * @brief Formats a double to a spi_frame.
 *
 * @param[in] format: Double which contains the values of the fields of the DX_TIME register.
 * @param[out] fr: spi_frame where this function will store the double(format).
 * @param[in] sub_register: Enumerate that indicates the sub-register.
 *
 * @note: This function must receive a double to work.
 *
 */
size_t dx_time_unformater(void *format, spi_frame fr, const size_t sub_register);

/******* RX_FWTO *******/

#define RFR_COUNTER_WRAP_PERIOD 1.026 * pow(10, -6) // (seconds)
#define RFR_COUNTER_RESOLUTION pow(2, 16)
#define RFR_K (RFR_COUNTER_WRAP_PERIOD/RFR_COUNTER_RESOLUTION)
#define rfr_calculate_seconds(register_value) register_value * RFR_K
#define rfr_calculate_register_val(seconds) seconds / RFR_K

/**
 * @brief Formats a spi_frame to a double
 *
 * @param[in] fr: The spi_frame to initialize the double.
 * @param[out] format: Double which will contains the spi_frame formatted.
 * @param[in] sub_register: Enumerate that indicates the sub-register.
 *
 * @note: This function must receive a double to work.
 *
 */
void rx_fwto_formater(spi_frame fr, void *format, const size_t sub_register);

/**
 * @brief Formats a double to a spi_frame.
 *
 * @param[in] format: Double which contains the values of the fields of the RX_FWTO register.
 * @param[out] fr: spi_frame where this function will store the double(format).
 * @param[in] sub_register: Enumerate that indicates the sub-register.
 *
 * @note: This function must receive a double to work.
 *
 */
size_t rx_fwto_unformater(void *format, spi_frame fr, const size_t sub_register);

/******* SYS_CTRL *******/

// Structure of the system control register.
typedef struct {
    unsigned int hrbpt :1;     // Host side receive buffer pointer toggle.
    unsigned int rxdlye :1;    // Receiver delayed enable.
    unsigned int rxenab :1;    // Enable receiver.
    unsigned int wait4resp :1; // Wait for response.
    unsigned int trxoff :1;    // Transceiver off.
    unsigned int cansfcs :1;   // Cancel suppression of auto-FCS transmission (on the current frame).
    unsigned int txdlys :1;    // Transmitter delayed sending.
    unsigned int txstrt :1;    // Transmit start.
    unsigned int sfcst :1;     // Suppress auto-FCS Transmission (on this next frame).
} sys_ctrl_format;

/**
 * @brief Formats a spi_frame to a sys_ctrl_format structure.
 *
 * @param[in] fr: The spi_frame to initialize the structure.
 * @param[out] format: Structure which will contains the spi_frame formatted.
 * @param[in] sub_register: Enumerate that indicates the sub-register.
 *
 * @note: This function must receive a sys_ctrl_format structure to work.
 *
 */
void tx_ctrl_formater(spi_frame fr, void *format, const size_t sub_register);

/**
 * @brief Formats a sys_ctrl_format structure to a spi_frame.
 *
 * @param[in] format: Structure which contains the values of the fields of the SYS_CTRL register.
 * @param[out] fr: spi_frame where this function will store the sys_ctrl_format structure.
 * @param[in] sub_register: Enumerate that indicates the sub-register.
 *
 * @return size_t: Size of the spi_frame formed with the given sys_evt_msk_format structure.
 *
 * @note: This function must receive a sys_ctrl_format structure to work.
 *
 */
size_t tx_ctrl_unformater(void *format, spi_frame fr, const size_t sub_register);

/******* SYS_MASK *******/

// Structure of the system event mask register.
typedef struct {
    unsigned int maffrej :1;   // Mask automatic frame filtering rejection event.
    unsigned int mtxberr :1;   // Mask transmit buffer error event.
    unsigned int mhpdwarn :1;  // Mask half period delay warning event.
    unsigned int mrxsfdto: 1;  // Mask receive SFD timeout event.
    unsigned int mcpllll :1;   // Mask clock PLL losing lock warning event.
    unsigned int mrfpllll :1;  // Mask RF PLL losing lock warning event.
    unsigned int mslp2init :1; // Mask SLEEP to INIT event.
    unsigned int mgpioirq :1;  // Mask GPIO interrupt event.
    unsigned int mrxpto :1;    // Mask Preamble detection timeout event.
    unsigned int mrxovrr :1;   // Mask receiver overrun event.
    unsigned int mldeerr :1;   // Mask leading edge detection processing error event.
    unsigned int mrxrfto :1;   // Mask receive frame wait timeout event.
    unsigned int mrxrfsl :1;   // Mask receiver reed solomon frame sync loss event.
    unsigned int mrxfce :1;    // Mask receiver FCS error event.
    unsigned int mrxfcg :1;    // Mask receiver FCS good event.
    unsigned int mrxdfr :1;    // Mask receiver data frame ready event.
    unsigned int mrxphe :1;    // Mask receiver PHY header error event.
    unsigned int mrxphd :1;    // Mask receiver PHY header detect event.
    unsigned int mldedone :1;  // Mask LDE processing done event.
    unsigned int mrxsfdd :1;   // Mask receiver SFD detected event.
    unsigned int mrxprd :1;    // Mask receiver preamble detected event.
    unsigned int mtxfrs :1;    // Mask transmit frame sent event.
    unsigned int mtxphs :1;    // Mask transmit PHY header sent event.
    unsigned int mtxprs :1;    // Mask transmit preamble sent event.
    unsigned int mtxfrb :1;    // Mask transmit frame begins event.
    unsigned int maat :1;      // Mask automatic acknowledge trigger event.
    unsigned int mesyncr :1;   // Mask external sync clock reset event.
    unsigned int mcplock :1;   // Mask clock PLL lock event.
} sys_evt_msk_format;

/**
 * @brief Formats a spi_frame to a sys_evt_msk_format structure.
 *
 * @param[in] fr: The spi_frame to initialize the structure.
 * @param[out] format: Structure which will contains the spi_frame formatted.
 * @param[in] sub_register: Enumerate that indicates the sub-register.
 *
 * @note: This function must receive a sys_evt_msk_format structure to work.
 *
 */
void sys_evt_msk_formater(spi_frame fr, void *format, const size_t sub_register);

/**
 * @brief Formats a sys_evt_msk_format structure to a spi_frame.
 *
 * @param[in] format: Structure which contains the values of the fields of the SYS_MASK register.
 * @param[out] fr: spi_frame where this function will store the sys_evt_msk_format structure.
 * @param[in] sub_register: Enumerate that indicates the sub-register.
 *
 * @return size_t: Size of the spi_frame formed with the given sys_evt_msk_format structure.
 *
 * @note: This function must receive a sys_evt_msk_format structure to work.
 *
 */
size_t sys_evt_msk_unformater(void *format, spi_frame fr, const size_t sub_register);

/******* SYS_STATUS *******/

// Fields of the system events status register.
typedef enum {
    SES_OCT_0_TO_3 = 0,
    SES_OCT_4 = 4
} sys_evt_sts_subregister;

// Structure of the system event status register.
typedef struct {
    unsigned int icrbp :1;     // IC side receive buffer pointer. (READ ONLY)
    unsigned int hsrbp :1;     // Host side receive buffer pointer. (READ ONLY)
    unsigned int affrej :1;    // Automatic frame filtering rejection.
    unsigned int txberr :1;    // Transmit buffer error.
    unsigned int hpdwarn :1;   // Half period delay warning. (READ ONLY)
    unsigned int rxsfdto :1;   // Receive SFD timeout.
    unsigned int clkpll_ll :1; // Clock PLL losing lock.
    unsigned int rfpll_ll :1;  // RF PLL losing lock.
    unsigned int slp2init :1;  // SLEEP to INIT.
    unsigned int gpioirq :1;   // GPIO interrupt.
    unsigned int rxpto :1;     // Preamble detection timeout.
    unsigned int rxovrr :1;    // Receiver overrun. (READ ONLY)
    unsigned int ldeerr :1;    // Leading edge detection processing error.
    unsigned int rxrfto :1;    // Receive frame wait timeout.
    unsigned int rxrfsl :1;    // Receiver reed solomon frame sync loss.
    unsigned int rxfce :1;     // Receiver FCS error.
    unsigned int rxfcg :1;     // Receiver FCS good.
    unsigned int rxdfr :1;     // Receiver data frame ready.
    unsigned int rxphe :1;     // Receiver PHY header error.
    unsigned int rxphd :1;     // Receiver PHY header detect.
    unsigned int ldedone :1;   // LDE processing done.
    unsigned int rxsfdd :1;    // Receiver SFD detected.
    unsigned int rxprd :1;     // Receiver preamble detected status.
    unsigned int txfrs :1;     // Transmit frame sent.
    unsigned int txphs :1;     // Transmit PHY header sent.
    unsigned int txprs :1;     // Transmit preamble sent.
    unsigned int txfrb :1;     // Transmit frame begins.
    unsigned int aat :1;       // Automatic acknowledge trigger.
    unsigned int esyncr :1;    // External sync clock reset.
    unsigned int cplock :1;    // Clock PLL lock.
    unsigned int irqs :1;      // Interrupt request status. (READ ONLY)
    unsigned int txpute :1;    // Transmit power up time error. (READ ONLY)
    unsigned int rxprej :1;    // Receiver preamble rejection.
    unsigned int rxrscs :1;    // Receiver Reed-solomon correction status.
} sys_evt_sts_format;

/**
 * @brief Formats a spi_frame to a sys_evt_sts_format structure.
 *
 * @param[in] fr: The spi_frame to initialize the structure.
 * @param[out] format: Structure which will contains the spi_frame formatted.
 * @param[in] sub_register: Enumerate that indicates the sub-register.
 *
 * @note: This function must receive a sys_evt_sts_format structure to work.
 *
 */
void sys_evt_sts_formater(spi_frame fr, void *format, const size_t sub_register);

/**
 * @brief Formats a sys_evt_sts_format structure to a spi_frame.
 *
 * @param[in] format: Structure which contains the values of the fields of the SYS_STATUS register.
 * @param[out] fr: spi_frame where this function will store the sys_evt_sts_format structure.
 * @param[in] sub_register: Enumerate that indicates the sub-register.
 *
 * @return size_t: Size of the spi_frame formed with the given sys_evt_sts_format structure.
 *
 * @note: This function must receive a sys_evt_sts_format structure to work.
 *
 */
size_t sys_evt_sts_unformater(void *format, spi_frame fr, const size_t sub_register);

/******* RX_FINFO *******/

// Structure of the rx frame information register.
typedef struct {
    unsigned int rxpacc :12;                          // Preamble accumulation count.
    preamble_lenght_selection rxnspl_rxpsr: 4;        // Receive non-standard preamble length and RX preamble repetition.
    transmit_receive_pulse_repetition_freq rxprfr :2; // RX pulse repetition rate report.
    unsigned int rng :1;                              // Receiver ranging.
    transmit_receive_bit_rate rxbr;                   // Receive bit rate report.
    unsigned int rxfle :3;                            // Receive frame length extension.
    unsigned int rxflen :7;                           // Receive frame length.
} rx_finfo_format;

/**
 * @brief Formats a spi_frame to a rx_finfo_format structure.
 *
 * @param[in] fr: The spi_frame to initialize the structure.
 * @param[out] format: Structure which will contains the spi_frame formatted.
 * @param[in] sub_register: Enumerate that indicates the sub-register.
 *
 * @note: This function must receive a rx_finfo_format structure to work.
 *
 */
void rx_finfo_formater(spi_frame fr, void *format, const size_t sub_register);

/******* RX_BUFFER *******/

/**
 * @brief Formats a spi_frame to an uint8_t*.
 *
 * @param[in] fr: spi_frame to initialize the array(format).
 * @param[out] format: Array which will contains the spi_frame formatted.
 * @param[in] sub_register: Enumerate that indicates the sub-register.
 *
 * @note: This function must receive a uint8_t* array to work.
 *
 */
void rx_buffer_formater(spi_frame fr, void *format, const size_t sub_register);

/******* RX_FQUAL *******/

// Fields of the rx frame quality information register.
typedef enum {
    FP_AMPL2_STD_NOISE = 0,
    CIR_PWR_PP_AMPL3 = 4
} rx_finfo_subregister;

// Structure of the rx frame quality information register.
typedef struct {
    uint16_t fp_ampl2;  // First path amplitude point 2.
    uint16_t std_noise; // Standard deviation of noise.
    uint16_t cir_pwr;   // Channel impulse response power.
    uint16_t pp_ampl3;  // First path amplitude point 3.
} rx_fqual_format;

/**
 * @brief Formats a spi_frame to an rx_fqual_format.
 *
 * @param[in] fr: spi_frame to initialize the structure(format).
 * @param[out] format: Structure which will contains the spi_frame formatted.
 * @param[in] sub_register: Enumerate that indicates the sub-register.
 *
 * @note: This function must receive a rx_fqual_format to work.
 *
 */
void rx_fqual_formater(spi_frame fr, void *format, const size_t sub_register);

/******* RX_TTCKI *******/

// Possible values of the rx_ttcki register.
typedef enum {
    RX_TTCKI_NONE = 0,
    RX_TTCKI_MHZ16 = 0x01F00000,
    RX_TTCKI_MHZPRF = 0x01FC0000
} rx_ttcki_value;

/**
 * @brief Formats a spi_frame to a rx_ttcki_value.
 *
 * @param[in] fr: spi_frame to initialize the rx_ttcki_value(format).
 * @param[out] format: Structure which will contains the spi_frame formatted.
 * @param[in] sub_register: Enumerate that indicates the sub-register.
 *
 * @note: This function must receive a rx_ttcki_value to work.
 *
 */
void rx_ttcki_formater(spi_frame fr, void *format, const size_t sub_register);

/******* RX_TTCKO *******/

#define PHASE_ADJUSTMENT_PRECISION 2.8125 // 360.0/pow(2, 7)
#define calculate_phase(register_value) register_value * PHASE_ADJUSTMENT_PRECISION

// Fields of the receiver time tracking offset register.
typedef enum {
    RX_TTCKO_OCT_0_TO_3 = 0,
    RX_TTCKO_OCT_4 = 4
} rx_ttcko_subregister;

// Structure of the receiver time tracking offset register.
typedef struct {
    uint8_t rsmpdel;         // Reports an internal re-sampler delay value.
    signed int rxtofs: 19;   // RX time tracking offset.
    unsigned int rcphase :9; // Reports the receive carrier phase adjustment at time the ranging
                             // Time stamp is made. This field contains the value in degrees.
                             // Therefore its maximum value must be 360
} rx_ttcko_format;

/**
 * @brief Formats a spi_frame to a rx_ttcko_format.
 *
 * @param[in] fr: spi_frame to initialize the rx_ttcko_format(format).
 * @param[out] format: Structure which will contains the spi_frame formatted.
 * @param[in] sub_register: Enumerate that indicates the sub-register.
 *
 * @note: This function must receive a rx_ttcko_format to work.
 *
 */
void rx_ttcko_formater(spi_frame fr, void *format, const size_t sub_register);

/******* RX_TIME *******/

#define RX_TX_STAMP_RESOLUTION (1.0/(128*499.2*pow(10, 6))) // (seconds)
#define calculate_stamp(register_value) register_value * RX_TX_STAMP_RESOLUTION

// Fields of the receiver time stamp register.
typedef enum {
    RX_TIME_OCT_0_TO_3 = 0,
    RX_TIME_OCT_4_TO_7 = 0X04,
    RX_TIME_OCT_8_TO_11 = 0X08,
    RX_TIME_OCT_12_TO_13 = 0X0C
} rx_time_subregister;

// Structure of the receiver time stamp register.
typedef struct {
    double rx_stamp;   // Reports the the fully adjusted time of reception.
    uint16_t fp_index; // Reporting the position within the accumulator that the
                       // LDE algorithm has determined to be the first path.
    uint16_t fp_ampl1; // Reporting the magnitude of the leading edge signal seen in
                       // the accumulator data memory during the LDE algorithm’s analysis.
    uint64_t rx_rawst; // Reports the Raw Time stamp for the frame.
} rx_time_format;

/**
 * @brief Formats a spi_frame to a rx_time_format.
 *
 * @param[in] fr: spi_frame to initialize the rx_time_format(format).
 * @param[out] format: Structure which will contains the spi_frame formatted.
 * @param[in] sub_register: Enumerate that indicates the sub-register.
 *
 * @note: This function must receive a rx_time_format to work.
 *
 */
void rx_time_formater(spi_frame fr, void *format, const size_t sub_register);

/******* TX_TIME *******/

// Fields of the transmit time stamp register.
typedef enum {
    TX_TIME_OCT_0_TO_3 = 0,
    TX_TIME_OCT_4_TO_7 = 0X04,
    TX_TIME_OCT_8_TO_9 = 0X08,
} tx_time_subregister;

// Structure of the transmit time stamp register.
typedef struct {
    uint64_t tx_stamp; // Reports the the fully adjusted time of reception.
    uint64_t tx_rawst; // Reports the Raw Time stamp for the frame.
} tx_time_format;

#endif // _FORMAT_H_
