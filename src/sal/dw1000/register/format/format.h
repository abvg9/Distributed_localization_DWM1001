/**
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
 * @brief Unformats an eui_format structure to a spi_frame.
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
    SHORT_ADR = 0x00,
    PAN_ID = 0x02,
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
 * @brief Unformats a pan_adr_format structure to a spi_frame.
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

// Posible values of the phr_mode field.
typedef enum {
    standard = 0b00,
    long_frames = 0b11
} phr_mode;

// Structure of the system configuration register.
typedef struct {
    unsigned int aackpend :1;   // Automatic acknowledgment pending bit control.
    unsigned int autoack :1;    // Automatic acknowledgment enable.
    unsigned int rxautr :1;     // Receiver auto-re-enable.
    unsigned int rxwtoe :1;     // Receive wait timeout enable.
    unsigned int rxm110k :1;    // Receiver mode 110 kbps data rate.
    unsigned int dis_stxp :1;   // Disable smart TX power control.
    phr_mode phr_mode :2;       // This configuration allows selection of PHR type to be one of two options.
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
 * @brief Unformats a sys_cfg_format structure to a spi_frame.
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
 * @brief Unformats a tx_fctrl_format structure to a spi_frame.
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
 * @brief Unformats an uint8_t* to a spi_frame.
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
 * @brief Unformats a double to a spi_frame.
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
 * @brief Unformats a double to a spi_frame.
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
 * @brief Unformats a sys_ctrl_format structure to a spi_frame.
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
 * @brief Unformats a sys_evt_msk_format structure to a spi_frame.
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
 * @brief Unformats a sys_evt_sts_format structure to a spi_frame.
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
    uint16_t fp_ampl3;  // First path amplitude point 3.
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
    double rx_rawst; // Reports the Raw time stamp for the frame.
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
    double tx_stamp; // Reports the the fully adjusted time of reception.
    double tx_rawst; // Reports the Raw Time stamp for the frame.
} tx_time_format;

/**
 * @brief Formats a spi_frame to a tx_time_format.
 *
 * @param[in] fr: spi_frame to initialize the tx_time_format(format).
 * @param[out] format: Structure which will contains the spi_frame formatted.
 * @param[in] sub_register: Enumerate that indicates the sub-register.
 *
 * @note: This function must receive a tx_time_format to work.
 *
 */
void tx_time_formater(spi_frame fr, void *format, const size_t sub_register);

/******* TX_ANTD *******/

#define TX_ANTD_LEAST_BIT_VALUE (15.65 * pow(10, -12)) // (seconds)
#define TX_ANTD_WRAP_PERIOD (TX_ANTD_LEAST_BIT_VALUE * pow(2, 16))
#define calculate_tx_antd(register_value) register_value * TX_ANTD_LEAST_BIT_VALUE
#define tx_antd_calculate_register_val(seconds) seconds / TX_ANTD_LEAST_BIT_VALUE

/**
 * @brief Formats a spi_frame to a double.
 *
 * @param[in] fr: spi_frame to initialize the double(format).
 * @param[out] format: double which will contains the spi_frame formatted.
 * @param[in] sub_register: Enumerate that indicates the sub-register.
 *
 * @note: This function must receive a double to work.
 *
 */
void tx_antd_formater(spi_frame fr, void *format, const size_t sub_register);

/**
 * @brief Unformats a double to a spi_frame.
 *
 * @param[in] format: double which contains the values of the fields of the TX_ANTD register.
 * @param[out] fr: spi_frame where this function will store the double value.
 * @param[in] sub_register: Enumerate that indicates the sub-register.
 *
 * @return size_t: Size of the spi_frame formed with the given double value.
 *
 * @note: This function must receive a double value to work.
 *
 */
size_t tx_antd_unformater(void *format, spi_frame fr, const size_t sub_register);

/******* SYS_STATE *******/

// Possible values of the tx_state field.
typedef enum {
    TX_IDLE = 0x0,     // Transmitter is idle.
    TX_PREAMBLE = 0x1, // Transmitting preamble.
    TX_SFD = 0x2,      // Transmitting SFD.
    TX_PHR = 0x3,      // Transmitting PHY Header data.
    TX_SDE = 0x4,      // Transmitting PHR parity SECDED bits.
    TX_DATA = 0x5      // Transmitting data block (330 symbols).
} tx_state_value;

// Possible values of the rx_state field.
typedef enum {
    RX_IDLE = 0x00,          // Receiver is in idle.
    RX_START_ANALOG = 0x01,  // Start analog receiver blocks.
    RX_RX_RDY = 0x04,        // Receiver ready.
    RX_PREAMBLE_FND = 0x05,  // Receiver is waiting to detect preamble.
    RX_PRMBL_TIMEOUT = 0x06, // Preamble timeout.
    RX_SFD_FND = 0x07,       // SFD found.
    RX_CNFG_PHR_RX = 0x08,   // Configure for PHR reception.
    RX_PHR_RX_STRT = 0x09,   // PHR reception started.
    RX_DATA_RATE_RDY = 0x0A, // Ready for data reception.
    RX_DATA_RX_SEQ = 0x0C,   // Data reception.
    RX_CNFG_DATA_RX = 0x0D,  // Configure for data.
    RX_PHR_NOT_OK = 0x0E,    // PHR error.
    RX_LAST_SYMBOL = 0x0F,   // Received last symbol.
    RX_WAIT_RSD_DONE = 0x10, // Wait for Reed Solomon decoder to finish.
    RX_RSD_OK = 0x11,        // Reed Solomon correct.
    RX_RSD_NOT_OK = 0x12,    // Reed Solomon error.
    RX_RECONFIG_110 = 0x13,  // Reconfigure for 110 kbps data.
    RX_WAIT_110_PHR = 0x14,  // Wait for 110 kbps PHR.
} rx_state_value;

// Possible values of the psmc_state field.
typedef enum {
    INIT = 0x0,    // DW1000 is in init.
    IDLE = 0x1,    // DW1000 is in idle.
    TX_WAIT = 0x2, // DW1000 is waiting to start transmitting.
    RX_WAIT = 0x3, // DW1000 is waiting to enter receive mode.
    TX = 0x4,      // DW1000 is transmitting.
    RX = 0x5,      // DW1000 is in receive mode.
} psmc_state_value;

// Structure of the system status register.
typedef struct {
    psmc_state_value pmsc_state; // Current transmit state machine value.
    rx_state_value rx_state;     // Current receive state machine value.
    tx_state_value tx_state;     // Current PMSC state machine value.
} sys_status_format;

/**
 * @brief Formats a spi_frame to a sys_state_format.
 *
 * @param[in] fr: spi_frame to initialize the sys_state_format(format).
 * @param[out] format: Structure which will contains the spi_frame formatted.
 * @param[in] sub_register: Enumerate that indicates the sub-register.
 *
 * @note: This function must receive a sys_state_format to work.
 *
 */
void sys_state_formater(spi_frame fr, void *format, const size_t sub_register);

/******* ACK_RESP_T *******/

#define W4RTIM_LEAST_BIT_VALUE (1 * pow(10, -6)) // (seconds)
#define W4RTIM_WRAP_PERIOD (W4RTIM_LEAST_BIT_VALUE * pow(2, 20))
#define calculate_w4r_tim(register_value) register_value * W4RTIM_LEAST_BIT_VALUE
#define w4r_tim_calculate_register_val(seconds) seconds / W4RTIM_LEAST_BIT_VALUE

// Structure of the acknowledgment time and response time register.
typedef struct {
    uint8_t ack_tim; // Auto-acknowledgment turn-around time.
    double w4r_tim;  // Wait for response turn-around time.
} ack_resp_t_format;

/**
 * @brief Formats a spi_frame to an ack_resp_t_format.
 *
 * @param[in] fr: spi_frame to initialize the ack_resp_t_format(format).
 * @param[out] format: Structure which will contains the spi_frame formatted.
 * @param[in] sub_register: Enumerate that indicates the sub-register.
 *
 * @note: This function must receive an ack_resp_t_format to work.
 *
 */
void ack_resp_t_formater(spi_frame fr, void *format, const size_t sub_register);

/**
 * @brief Unformats an ack_resp_t_format to a spi_frame.
 *
 * @param[in] format: Structure which contains the values of the fields of the ACK_RESP_T register.
 * @param[out] fr: spi_frame where this function will store the ack_resp_t_format structure.
 * @param[in] sub_register: Enumerate that indicates the sub-register.
 *
 * @return size_t: Size of the spi_frame formed with the given ack_resp_t_format structure.
 *
 * @note: This function must receive a ack_resp_t_format value to work.
 *
 */
size_t ack_resp_t_unformater(void *format, spi_frame fr, const size_t sub_register);

/******* RX_SNIFF *******/

#define SNIFF_OFFT_LEAST_BIT_VALUE (1 * pow(10, -6)) // (seconds)
#define SNIFF_OFFT_WRAP_PERIOD (SNIFF_OFFT_LEAST_BIT_VALUE * pow(2, 4))
#define calculate_sniff_offt(register_value) register_value * SNIFF_OFFT_LEAST_BIT_VALUE
#define sniff_offt_calculate_register_val(seconds) seconds / SNIFF_OFFT_LEAST_BIT_VALUE

// Structure of the sniff mode configuration register.
typedef struct {
    double sniff_offt;         // Sniff mode off time specified in µs.
    unsigned int sniff_ont :4; // Auto-acknowledgment turn-around time.
} rx_sniff_format;

/**
 * @brief Formats a spi_frame to a rx_sniff_format.
 *
 * @param[in] fr: spi_frame to initialize the rx_sniff_format(format).
 * @param[out] format: Structure which will contains the spi_frame formatted.
 * @param[in] sub_register: Enumerate that indicates the sub-register.
 *
 * @note: This function must receive an rx_sniff_format to work.
 *
 */
void rx_sniff_formater(spi_frame fr, void *format, const size_t sub_register);

/**
 * @brief Unformats a rx_sniff_format to a spi_frame.
 *
 * @param[in] format: Structure which contains the values of the fields of the RX_SNIFF register.
 * @param[out] fr: spi_frame where this function will store the rx_sniff_format structure.
 * @param[in] sub_register: Enumerate that indicates the sub-register.
 *
 * @return size_t: Size of the spi_frame formed with the given rx_sniff_format structure.
 *
 * @note: This function must receive a rx_sniff_format value to work.
 *
 */
size_t rx_sniff_unformater(void *format, spi_frame fr, const size_t sub_register);

/******* TX_POWER *******/

// Possible values of the coarse_da field.
typedef enum {
    COARSE_DA_DB_GAIN_15,
    COARSE_DA_DB_GAIN_12_5,
    COARSE_DA_DB_GAIN_10,
    COARSE_DA_DB_GAIN_7_5,
    COARSE_DA_DB_GAIN_5,
    COARSE_DA_DB_GAIN_2_5,
    COARSE_DA_DB_GAIN_0,
    COARSE_DA_DB_GAIN_OFF,
} coarse_da_value;

// Possible values of the finer_mixer field.
typedef enum {
    FINER_MIXER_DB_GAIN_0,
    FINER_MIXER_DB_GAIN_0_5,
    FINER_MIXER_DB_GAIN_1,
    FINER_MIXER_DB_GAIN_1_5,
    FINER_MIXER_DB_GAIN_2,
    FINER_MIXER_DB_GAIN_2_5,
    FINER_MIXER_DB_GAIN_3,
    FINER_MIXER_DB_GAIN_3_5,
    FINER_MIXER_DB_GAIN_4,
    FINER_MIXER_DB_GAIN_4_5,
    FINER_MIXER_DB_GAIN_5,
    FINER_MIXER_DB_GAIN_5_5,
    FINER_MIXER_DB_GAIN_6,
    FINER_MIXER_DB_GAIN_6_5,
    FINER_MIXER_DB_GAIN_7,
    FINER_MIXER_DB_GAIN_7_5,
    FINER_MIXER_DB_GAIN_8,
    FINER_MIXER_DB_GAIN_8_5,
    FINER_MIXER_DB_GAIN_9,
    FINER_MIXER_DB_GAIN_9_5,
    FINER_MIXER_DB_GAIN_10,
    FINER_MIXER_DB_GAIN_10_5,
    FINER_MIXER_DB_GAIN_11,
    FINER_MIXER_DB_GAIN_11_5,
    FINER_MIXER_DB_GAIN_12,
    FINER_MIXER_DB_GAIN_12_5,
    FINER_MIXER_DB_GAIN_13,
    FINER_MIXER_DB_GAIN_13_5,
    FINER_MIXER_DB_GAIN_14,
    FINER_MIXER_DB_GAIN_14_5,
    FINER_MIXER_DB_GAIN_15,
    FINER_MIXER_DB_GAIN_15_5,
} finer_mixer_value;

// Fields of the tx_power register.
typedef struct {
    coarse_da_value coarse_da_setting;
    finer_mixer_value fine_mixer_setting;
} tx_power_field;

// Structure of the tx power control register.
// Depending on the value of dis_stxp(SYS_CFG)the meaning of the fields
// of this register changes. That is why they have generic names.
typedef struct {                // DIS_STXP = 0 |  DIS_STXP = 1
                                // -------------|---------------
    tx_power_field field_32_24; //  BOOSTNOMR   | Not applicable
    tx_power_field field_23_16; //  BOOSTP500   |    TXPOWSD
    tx_power_field field_15_8;  //  BOOSTP250   |    TXPOWPHR
    tx_power_field field_7_0;   //  BOOSTP125   | Not applicable
} tx_power_format;

/**
 * Legend:
 *
 *  DIS_STXP = 0
 *
 *   BOOSTNOMR: This is the normal power setting used for frames that do not fall within the data rate and
 *              frame length criteria required for a boost, i.e. the frame duration is more than 0.5 ms.
 *
 *   BOOSTP500: This value sets the power applied to the preamble and data portions of the frame during
 *              transmission at the 6.8 Mbps data rate for frames that are less than 0.5 ms duration.
 *
 *   BOOSTP250: This value sets the power applied to the preamble and data portions of the frame during
 *              transmission at the 6.8 Mbps data rate for frames that are less than 0.25 ms duration-
 *
 *   BOOSTP125: This value sets the power applied to the preamble and data portions of the frame during
 *              transmission at the 6.8 Mbps data rate for frames that are less than 0.125 ms duration
 *
 *  DIS_STXP = 1
 *
 *   TXPOWSD: This power setting is applied during the transmission of the PHY header (PHR) portion of
 *            the frame.
 *
 *   TXPOWSD: This power setting is applied during the transmission of the synchronisation header (SHR)
 *             and data portions of the frame.
 *
 */

/**
 * @brief Formats a spi_frame to a tx_power_format.
 *
 * @param[in] fr: spi_frame to initialize the tx_power_format(format).
 * @param[out] format: Structure which will contains the spi_frame formatted.
 * @param[in] sub_register: Enumerate that indicates the sub-register.
 *
 * @note: This function must receive an tx_power_format to work.
 *
 */
void tx_power_formater(spi_frame fr, void *format, const size_t sub_register);

/**
 * @brief Unformats a tx_power_format to a spi_frame.
 *
 * @param[in] format: Structure which contains the values of the fields of the TX_POWER register.
 * @param[out] fr: spi_frame where this function will store the tx_power_format structure.
 * @param[in] sub_register: Enumerate that indicates the sub-register.
 *
 * @return size_t: Size of the spi_frame formed with the given tx_power_format structure.
 *
 * @note: This function must receive a tx_power_format value to work.
 *
 */
size_t tx_power_unformater(void *format, spi_frame fr, const size_t sub_register);

/******* CHAN_CTRL *******/

// Possibles values of the rx_chan and tx_chan fields.
typedef enum {
    CH1 = 1,
    CH2 = 2,
    CH3 = 3,
    CH4 = 4,
    CH5 = 5,
    CH7 = 7
} channel;

// Structure of the channel control register.
typedef struct {
    unsigned int rx_pcode :5;                      // This field selects the preamble code used in the receiver.
    unsigned int tx_pcode :5;                      // This field selects the preamble code used in the transmitter.
    unsigned int rnssfd :1;                        // This bit enables the use of a user specified (non-standard) SFD in the receiver.
    unsigned int tnssfd :1;                        // This bit enables the use of a user specified (non-standard) SFD in the transmitter.
    transmit_receive_pulse_repetition_freq rxprf;  // This two bit field selects the PRF used in the receiver.
    unsigned int dwsfd :1;                         // This bit enables a non-standard decawave proprietary SFD sequence.
    channel rx_chan :4;                            // This selects the receive channel.
    channel tx_chan :4;                            // This selects the transmit channel.
} chan_ctrl_format;

/**
 * @brief Formats a spi_frame to an chan_ctrl_format.
 *
 * @param[in] fr: spi_frame to initialize the chan_ctrl_format(format).
 * @param[out] format: Structure which will contains the spi_frame formatted.
 * @param[in] sub_register: Enumerate that indicates the sub-register.
 *
 * @note: This function must receive an chan_ctrl_format to work.
 *
 */
void chan_ctrl_formater(spi_frame fr, void *format, const size_t sub_register);

/**
 * @brief Unformats a chan_ctrl_format to a spi_frame.
 *
 * @param[in] format: Structure which contains the values of the fields of the CHAN_CTRL register.
 * @param[out] fr: spi_frame where this function will store the chan_ctrl_format structure.
 * @param[in] sub_register: Enumerate that indicates the sub-register.
 *
 * @return size_t: Size of the spi_frame formed with the given chan_ctrl_format structure.
 *
 * @note: This function must receive a chan_ctrl_format value to work.
 *
 */
size_t chan_ctrl_unformater(void *format, spi_frame fr, const size_t sub_register);

/******* USR_SFD *******/

// Sub-registers of the user-specified short/long TX/RX SFD sequences register.
typedef enum {
    USR_SFD_OCT_0_TO_3 = 0x00,
    USR_SFD_OCT_4_TO_7 = 0x04,
    USR_SFD_OCT_8_TO_11 = 0x08,
    USR_SFD_OCT_12_TO_15 = 0x0C,
    USR_SFD_OCT_16_TO_19 = 0x10,
    USR_SFD_OCT_20_TO_23 = 0x14,
    USR_SFD_OCT_24_TO_27 = 0x18,
    USR_SFD_OCT_28_TO_31 = 0x1C,
    USR_SFD_OCT_32_TO_35 = 0x20,
    USR_SFD_OCT_36_TO_39 = 0x24,
    USR_SFD_OCT_40_TO_41 = 0x28,
} usr_sfd_subregister;

// Structure of the user-specified short/long TX/RX SFD sequences register.
typedef struct {
    uint8_t rx_lsfd_sgn7; // This field sets the long (64-symbol) SFD polarity data for the receive SFD
                          // sequence, for symbol intervals 63 to 56.
    uint8_t rx_lsfd_sgn6; // This field sets the long (64-symbol) SFD polarity data for the transmitted SFD
                          // sequence, for symbol intervals 55 to 48.
    uint8_t rx_lsfd_sgn5; // This field sets the long (64-symbol) SFD polarity data for the receive SFD
                          // sequence, for symbol intervals 47 to 40.
    uint8_t rx_lsfd_sgn4; // This field sets the long (64-symbol) SFD polarity data for the receive SFD
                          // sequence, for symbol intervals 39 to 32.
    uint8_t rx_lsfd_sgn3; // This field sets the long (64-symbol) SFD polarity data for the receive SFD
                          // sequence, for symbol intervals 31 to 24.
    uint8_t rx_lsfd_sgn2; // This field sets the long (64-symbol) SFD polarity data for the receive SFD
                          // sequence, for symbol intervals 23 to 16.
    uint8_t rx_lsfd_sgn1; // This field sets the long (64-symbol) SFD polarity data for the receive SFD
                          // sequence, for symbol intervals 15 to 8.
    uint8_t rx_lsfd_sgn0; // This field sets the long (64-symbol) SFD polarity data for the receive SFD
                          // sequence.
    uint8_t rx_lsfd_mag7; // This field sets the long (64-symbol) SFD magnitude data for the receive SFD
                          // sequence, for symbol intervals 63 to 56.
    uint8_t rx_lsfd_mag6; // This field sets the long (64-symbol) SFD magnitude data for the receive SFD
                          // sequence, for symbol intervals 55 to 48.
    uint8_t rx_lsfd_mag5; // This field sets the long (64-symbol) SFD magnitude data for the receive SFD
                          // sequence, for symbol intervals 47 to 40.
    uint8_t rx_lsfd_mag4; // This field sets the long (64-symbol) SFD magnitude data for the receive SFD
                          // sequence, for symbol intervals 39 to 32.
    uint8_t rx_lsfd_mag3; // This field sets the long (64-symbol) SFD magnitude data for the receive SFD
                          // sequence, for symbol intervals 31 to 24.
    uint8_t rx_lsfd_mag2; // This field sets the long (64-symbol) SFD magnitude data for the receive SFD
                          // sequence, for symbol intervals 23 to 16.
    uint8_t rx_lsfd_mag1; // This field sets the long (64-symbol) SFD magnitude data for the receive SFD
                          // sequence, for symbol intervals 15 to 8.
    uint8_t rx_lsfd_mag0; // This field sets the long (64-symbol) SFD magnitude data for the receive SFD
                          // sequence.
    uint8_t tx_lsfd_sgn7; // This field sets the long (64-symbol) SFD polarity data for the transmitted SFD
                          // sequence, for symbol intervals 63 to 56.
    uint8_t tx_lsfd_sgn6; // This field sets the long (64-symbol) SFD polarity data for the transmitted SFD
                          // sequence, for symbol intervals 55 to 48.
    uint8_t tx_lsfd_sgn5; // This field sets the long (64-symbol) SFD polarity data for the transmitted SFD
                          // sequence, for symbol intervals 47 to 40.
    uint8_t tx_lsfd_sgn4; // This field sets the long (64-symbol) SFD polarity data for the transmitted SFD
                          // sequence, for symbol intervals 39 to 32.
    uint8_t tx_lsfd_sgn3; // This field sets the long (64-symbol) SFD polarity data for the transmitted SFD
                          // sequence, for symbol intervals 31 to 24.
    uint8_t tx_lsfd_sgn2; // This field sets the long (64-symbol) SFD polarity data for the transmitted SFD
                          // sequence, for symbol intervals 23 to 16.
    uint8_t tx_lsfd_sgn1; // This field sets the long (64-symbol) SFD polarity data for the transmitted SFD
                          // sequence, for symbol intervals 15 to 8.
    uint8_t tx_lsfd_sgn0; // This field sets the long (64-symbol) SFD polarity data for the transmitted SFD
                          // sequence.
    uint8_t tx_lsfd_mag7; // This field sets the long (64-symbol) SFD magnitude data for the transmitted
                          // SFD sequence, for symbol intervals 63 to 56.
    uint8_t tx_lsfd_mag6; // This field sets the long (64-symbol) SFD magnitude data for the transmitted
                          // SFD sequence, for symbol intervals 55 to 48.
    uint8_t tx_lsfd_mag5; // This field sets the long (64-symbol) SFD magnitude data for the transmitted
                          // SFD sequence, for symbol intervals 47 to 40.
    uint8_t tx_lsfd_mag4; // This field sets the long (64-symbol) SFD magnitude data for the transmitted
                          // SFD sequence, for symbol intervals 39 to 32.
    uint8_t tx_lsfd_mag3; // This field sets the long (64-symbol) SFD magnitude data for the transmitted
                          // SFD sequence, for symbol intervals 31 to 24.
    uint8_t tx_lsfd_mag2; // This field sets the long (64-symbol) SFD magnitude data for the transmitted
                          // SFD sequence, for symbol intervals 23 to 16.
    uint8_t tx_lsfd_mag1; // This field sets the long (64-symbol) SFD magnitude data for the transmitted
                          // SFD sequence, for symbol intervals 15 to 8.
    uint8_t tx_lsfd_mag0; // This field sets the long (64-symbol) SFD magnitude data for the transmitted
                          // SFD sequence.
    uint8_t rx_ssfd_sgnh; // This field sets the short SFD polarity data for the receive SFD sequence, for
                          // the second 8 symbol intervals.
    uint8_t rx_ssfd_sgnl; // This field sets the short SFD polarity data for the receive SFD sequence, for
                          // the first 8 symbol intervals.
    uint8_t rx_ssfd_magh; // This field sets the short SFD magnitude data for the receive SFD sequence, for
                          // the second 8 symbol intervals.
    uint8_t rx_ssfd_magl; // This field sets the short SFD magnitude data for the receive SFD sequence, for
                          // the first 8 symbol intervals.
    uint8_t tx_ssfd_sgnh; // This field sets the short SFD polarity data for the transmitted SFD sequence,
                          // for the second 8 symbol intervals.
    uint8_t tx_ssfd_sgnl; // This field sets the short SFD polarity data for the transmitted SFD sequence,
                          // for the first 8 symbol intervals.
    uint8_t tx_ssfd_magh; // This field sets the short SFD magnitude data for the transmitted SFD
                          // sequence, for the second 8 symbol intervals.
    uint8_t tx_ssfd_magl; // This field sets the short SFD magnitude data for the transmitted SFD
                          // sequence, for the first 8 symbol intervals.
    uint8_t sfd_length;   // This is the length of the SFD sequence used when the data rate is 850 kbps
                          // and higher.
} usr_sfd_format;

/**
 * @brief Formats a spi_frame to an usr_sfd_format.
 *
 * @param[in] fr: spi_frame to initialize the usr_sfd_format(format).
 * @param[out] format: Structure which will contains the spi_frame formatted.
 * @param[in] sub_register: Enumerate that indicates the sub-register.
 *
 * @note: This function must receive an usr_sfd_format to work.
 *
 */
void usr_sfd_formater(spi_frame fr, void *format, const size_t sub_register);

/**
 * @brief Unformats an usr_sfd_format to a spi_frame.
 *
 * @param[in] format: Structure which contains the values of the fields of the USR_SFD register.
 * @param[out] fr: spi_frame where this function will store the usr_sfd_format structure.
 * @param[in] sub_register: Enumerate that indicates the sub-register.
 *
 * @return size_t: Size of the spi_frame formed with the given usr_sfd_format structure.
 *
 * @note: This function must receive a usr_sfd_format value to work.
 *
 */
size_t usr_sfd_unformater(void *format, spi_frame fr, const size_t sub_register);

/******* AGC_CTRL *******/

// Sub-registers of the automatic gain control configuration and control register.
typedef enum {
    AGC_CTRL1 = 0x00,
    AGC_TUNE1 = 0x04,
    AGC_TUNE2 = 0x0C,
    AGC_TUNE3 = 0x12,
    AGC_STAT1 = 0x1E,
} agc_ctrl_subregister;

// Structure of the automatic gain control configuration and control register.
typedef struct {
    unsigned int dis_am :1; // Disable AGC Measurement. (RW)
    uint16_t agc_tune1;     // Tuning register for the AGC. (RW)
    uint32_t agc_tune2;     // Tuning register for the AGC. (RW)
    uint16_t agc_tune3;     // Tuning register for the AGC. (RW)
    unsigned int edv2 :9;   // Relates to the input noise power measurement. (RO)
    unsigned int edg1 :5;   // Relates to input noise power measurement. EDG1 can be used in
                            // conjunction with the EDV2 value to give a measure of the background in-band noise energy
                            // level. (RO)
} agc_ctrl_format;

/**
 * @brief Formats a spi_frame to an agc_ctrl_format.
 *
 * @param[in] fr: spi_frame to initialize the agc_ctrl_format(format).
 * @param[out] format: Structure which will contains the spi_frame formatted.
 * @param[in] sub_register: Enumerate that indicates the sub-register.
 *
 * @note: This function must receive an agc_ctrl_format to work.
 *
 */
void agc_ctrl_formater(spi_frame fr, void *format, const size_t sub_register);

/**
 * @brief Unformats an agc_ctrl_format to a spi_frame.
 *
 * @param[in] format: Structure which contains the values of the fields of the AGC_CTRL register.
 * @param[out] fr: spi_frame where this function will store the agc_ctrl_format structure.
 * @param[in] sub_register: Enumerate that indicates the sub-register.
 *
 * @return size_t: Size of the spi_frame formed with the given agc_ctrl_format structure.
 *
 * @note: This function must receive a agc_ctrl_format value to work.
 *
 */
size_t agc_ctrl_unformater(void *format, spi_frame fr, const size_t sub_register);

/******* EXT_SYNC *******/

// Sub-registers of the external clock synchronization counter configuration register.
typedef enum {
    EC_CTRL = 0x00,
    EC_RXTC = 0x04,
    EC_GOLP = 0x0C,
} ext_sync_subregister;

// Structure of the external clock synchronization counter configuration register.
typedef struct {
    unsigned int ostrm :1;      // External time base reset mode enable. (RW)
    uint8_t wait;               // Wait counter used for external transmit synchronization and external time base reset. (RW)
    unsigned int pllldt :1;     // Clock PLL lock detect tune. This bit should be set to 1 to ensure reliable operation of the clock
                                // PLL lock detect flags. (RW)
    unsigned int osrsm :1;      // External receive synchronization mode enable. (RW)
    unsigned int ostsm :1;      // External transmit synchronization mode enable. (RW)
    uint32_t rx_ts_est;         // External clock synchronization counter captured on RMARKER. (RO)
    unsigned int offset_ext :6; // External clock offset to first path 1 GHz counter. (RO)
} ext_sync_format;

/**
 * @brief Formats a spi_frame to an ext_sync_format.
 *
 * @param[in] fr: spi_frame to initialize the ext_sync_format(format).
 * @param[out] format: Structure which will contains the spi_frame formatted.
 * @param[in] sub_register: Enumerate that indicates the sub-register.
 *
 * @note: This function must receive an ext_sync_format to work.
 *
 */
void ext_sync_formater(spi_frame fr, void *format, const size_t sub_register);

/**
 * @brief Unformats an ext_sync_format to a spi_frame.
 *
 * @param[in] format: Structure which contains the values of the fields of the EXT_SYNC register.
 * @param[out] fr: spi_frame where this function will store the ext_sync_format structure.
 * @param[in] sub_register: Enumerate that indicates the sub-register.
 *
 * @return size_t: Size of the spi_frame formed with the given ext_sync_format structure.
 *
 * @note: This function must receive a ext_sync_format value to work.
 *
 */
size_t ext_sync_unformater(void *format, spi_frame fr, const size_t sub_register);

/******* ACC_MEM *******/

// Fields of the read access to accumulator data memory register.
typedef struct {
    signed int real: 16;      // Real part.
    signed int imaginary: 16; // Imaginary part.
} acc_mem_field;

/**
 * @brief Formats a spi_frame to an acc_mem_format.
 *
 * @param[in] fr: spi_frame to initialize the acc_mem_format(format).
 * @param[out] format: Array which will contains the spi_frame formatted.
 * @param[in] sub_register: Enumerate that indicates the sub-register.
 *
 * @note: This function must receive an acc_mem_format to work.
 *
 */
void acc_mem_formater(spi_frame fr, void *format, const size_t sub_register);

/******* GPIO_CTRL *******/

// Sub-registers of the GPIO control register.
typedef enum {
    GPIO_MODE = 0x00,
    GPIO_DIR = 0x08,
    GPIO_DOUT = 0x0C,
    GPIO_IRQE = 0x10,
    GPIO_ISEN = 0x14,
    GPIO_IMODE = 0x18,
    GPIO_IBES = 0x1C,
    GPIO_ICLR = 0x20,
    GPIO_IDBE = 0x24,
    GPIO_RAW = 0x28,
} gpio_ctrl_subregister;

// Possible values of the msgp0 field.
typedef enum {
    MSGP0_DEFAULT_MODE = 0b00,
    MSGP0_RXOKLED = 0b01,          // The output is asserted briefly when the receiver
                                   // completes the reception of a frame with a good FCS/CRC.
    MSGP0_GET_SYSTEM_CLOCK = 0b10,
    MSGP0_RESERVED_VALUE = 0b00,   // This value should be 0b11, but this is to prevent used it.
} msgp0_value;

// Possible values of the msgp1 field.
typedef enum {
    MSGP1_DEFAULT_MODE = 0b00,
    MSGP1_SFDLED = 0b01,          // The output is asserted briefly when the receiver detects the SFD
                                  // sequence in the RX frame.
    MSGP1_RESERVED_VALUE1 = 0b00, // This value should be 0b01, but this is to prevent used it.
    MSGP1_RESERVED_VALUE2 = 0b00, // This value should be 0b11, but this is to prevent used it.
} msgp1_value;

// Possible values of the msgp2 field.
typedef enum {
    MSGP2_DEFAULT_MODE = 0b00,
    MSGP2_RXLED = 0b01,           // The output is asserted when the receiver is on, and stays
                                  // on for a brief period after the receiver is turned off.
    MSGP2_RESERVED_VALUE1 = 0b00, // This value should be 0b01, but this is to prevent used it.
    MSGP2_RESERVED_VALUE2 = 0b00, // This value should be 0b11, but this is to prevent used it.
} msgp2_value;

// Possible values of the msgp3 field.
typedef enum {
    MSGP3_DEFAULT_MODE = 0b00,
    MSGP3_TXLED = 0b01,           // The output is asserted briefly when the transmitter
                                  // completes sending a frame.
    MSGP3_RESERVED_VALUE1 = 0b00, // This value should be 0b01, but this is to prevent used it.
    MSGP3_RESERVED_VALUE2 = 0b00, // This value should be 0b11, but this is to prevent used it.
} msgp3_value;

// Possible values of the msgp4 field.
typedef enum {
    MSGP4_DEFAULT_MODE = 0b00,
    MSGP4_EXTPA = 0b01,           // The output of the external power amplification.
    MSGP4_RESERVED_VALUE1 = 0b00, // This value should be 0b01, but this is to prevent used it.
    MSGP4_RESERVED_VALUE2 = 0b00, // This value should be 0b11, but this is to prevent used it.
} msgp4_value;

// Possible values of the msgp5 field.
typedef enum {
    MSGP5_DEFAULT_MODE = 0b00,
    MSGP5_EXTTXE = 0b01,          // The output of the external power amplification.
    MSGP5_RESERVED_VALUE1 = 0b00, // This value should be 0b01, but this is to prevent used it.
    MSGP5_RESERVED_VALUE2 = 0b00, // This value should be 0b11, but this is to prevent used it.
} msgp5_value;

// Possible values of the msgp6 field.
typedef enum {
    MSGP6_DEFAULT_MODE = 0b00,
    MSGP6_EXTRXE = 0b01,          // The output of the external power amplification.
    MSGP6_RESERVED_VALUE1 = 0b00, // This value should be 0b01, but this is to prevent used it.
    MSGP6_RESERVED_VALUE2 = 0b00, // This value should be 0b11, but this is to prevent used it.
} msgp6_value;

// Possible values of the msgp7 field.
typedef enum {
    MSGP7_DEFAULT_MODE_SYNC = 0b00, // The pin operates as the SYNC input.
    MSGP7_GPIO7 = 0b01,             // The pin operates as GPIO7.
    MSGP7_RESERVED_VALUE1 = 0b00,   // This value should be 0b01, but this is to prevent used it.
    MSGP7_RESERVED_VALUE2 = 0b00,   // This value should be 0b11, but this is to prevent used it.
} msgp7_value;

// Possible values of the msgp8 field.
typedef enum {
    MSGP8_DEFAULT_MODE_IRQ = 0b00, // The pin operates as the IRQ output.
    MSGP8_GPIO8 = 0b01,            // The pin operates as GPIO8.
    MSGP8_RESERVED_VALUE1 = 0b00,  // This value should be 0b01, but this is to prevent used it.
    MSGP8_RESERVED_VALUE2 = 0b00,  // This value should be 0b11, but this is to prevent used it.
} msgp8_value;

// Structure of the GPIO mode control sub-register.
typedef struct {
    msgp8_value msgp8; // Mode Selection for IRQ/GPIO8.
    msgp7_value msgp7; // Mode Selection for SYNC/GPIO7.
    msgp6_value msgp6; // Mode Selection for GPIO6/EXTRXE.
    msgp5_value msgp5; // Mode Selection for GPIO5/EXTTXE.
    msgp4_value msgp4; // Mode Selection for GPIO4/EXTPA.
    msgp3_value msgp3; // Mode Selection for GPIO3/TXLED.
    msgp2_value msgp2; // Mode Selection for GPIO2/RXLED.
    msgp1_value msgp1; // Mode Selection for GPIO1/SFDLED.
    msgp0_value msgp0; // Mode Selection for GPIO0/RXOKLED.
} gpio_mode_ctrl_format;

// Possibles modes of the GPIOS.
typedef enum {
    INPUT = 0b1,
    OUTPUT = 0b0,
} input_output_mode;

// Structure of the GPIO direction control sub-register.
typedef struct {
    input_output_mode gdm8; // Mask for setting the direction of GPIO8.
    input_output_mode gdp8; // Reading this bit shows the direction setting for GPIO8.
    input_output_mode gdm7; // Mask for setting the direction of GPIO7.
    input_output_mode gdm6; // Mask for setting the direction of GPIO6.
    input_output_mode gdm5; // Mask for setting the direction of GPIO5.
    input_output_mode gdm4; // Mask for setting the direction of GPIO4.
    input_output_mode gdp7; // Reading this bit shows the direction setting for GPIO7.
    input_output_mode gdp6; // Reading this bit shows the direction setting for GPIO6.
    input_output_mode gdp5; // Reading this bit shows the direction setting for GPIO5.
    input_output_mode gdp4; // Reading this bit shows the direction setting for GPIO4.
    input_output_mode gdm3; // Mask for setting the direction of GPIO3.
    input_output_mode gdm2; // Mask for setting the direction of GPIO2.
    input_output_mode gdm1; // Mask for setting the direction of GPIO1.
    input_output_mode gdm0; // Mask for setting the direction of GPIO0.
    input_output_mode gdp3; // Reading this bit shows the direction setting for GPIO3.
    input_output_mode gdp2; // Reading this bit shows the direction setting for GPIO2.
    input_output_mode gdp1; // Reading this bit shows the direction setting for GPIO1.
    input_output_mode gdp0; // Reading this bit shows the direction setting for GPIO0.
} gpio_direction_ctrl_format;

// Possibles outputs of the GPIOS that are configured as output.
typedef enum {
    HIGH = 0b1,
    LOW = 0b0
} output_state;

// Structure of the GPIO data output control sub-register.
typedef struct {
    output_state gom8; // Mask for setting GPIO8 output state.
    output_state gop8; // Reading this bit shows the current setting for GPIO8.
    output_state gom7; // Mask for setting GPIO7 output state.
    output_state gom6; // Mask for setting GPIO6 output state.
    output_state gom5; // Mask for setting GPIO5 output state.
    output_state gom4; // Mask for setting GPIO4 output state.
    output_state gop7; // Reading this bit shows the current setting for GPIO7.
    output_state gop6; // Reading this bit shows the current setting for GPIO6.
    output_state gop5; // Reading this bit shows the current setting for GPIO5.
    output_state gop4; // Reading this bit shows the current setting for GPIO4.
    output_state gom3; // Mask for setting GPIO3 output state.
    output_state gom2; // Mask for setting GPIO2 output state.
    output_state gom1; // Mask for setting GPIO1 output state.
    output_state gom0; // Mask for setting GPIO0 output state.
    output_state gop3; // Reading this bit shows the current setting for GPIO3.
    output_state gop2; // Reading this bit shows the current setting for GPIO2.
    output_state gop1; // Reading this bit shows the current setting for GPIO1.
    output_state gop0; // Reading this bit shows the current setting for GPIO0.
} gpio_data_output_ctrl_format;

// Possibles modes of the IRQ GPIOS.
typedef enum {
    IRQ_ENEABLE = 0b1,
    IRQ_DISABLE = 0b0,
} gpio_irq_value;

// Structure of the GPIO interrupt enable sub-register.
typedef struct {
    gpio_irq_value girqe8; // GPIO IRQ Enable for GPIO8 input.
    gpio_irq_value girqe7; // GPIO IRQ Enable for GPIO7 input.
    gpio_irq_value girqe6; // GPIO IRQ Enable for GPIO6 input.
    gpio_irq_value girqe5; // GPIO IRQ Enable for GPIO5 input.
    gpio_irq_value girqe4; // GPIO IRQ Enable for GPIO4 input.
    gpio_irq_value girqe3; // GPIO IRQ Enable for GPIO3 input.
    gpio_irq_value girqe2; // GPIO IRQ Enable for GPIO2 input.
    gpio_irq_value girqe1; // GPIO IRQ Enable for GPIO1 input.
    gpio_irq_value girqe0; // GPIO IRQ Enable for GPIO0 input.
} gpio_irq_ctrl_format;

// Possibles senses of the IRQ GPIOS.
typedef enum {
    IRQ_RISING_EDGE = 0b0,  // Active in rising-edge.
    IRQ_FALLING_EDGE = 0b1, // Active in falling-edge.
} gpio_irq_sense_value;

// Structure of the GPIO Interrupt sense selection sub-register.
typedef struct {
    gpio_irq_sense_value gisen8; // GPIO IRQ sense selection GPIO8 input.
    gpio_irq_sense_value gisen7; // GPIO IRQ sense selection GPIO7 input.
    gpio_irq_sense_value gisen6; // GPIO IRQ sense selection GPIO6 input.
    gpio_irq_sense_value gisen5; // GPIO IRQ sense selection GPIO5 input.
    gpio_irq_sense_value gisen4; // GPIO IRQ sense selection GPIO4 input.
    gpio_irq_sense_value gisen3; // GPIO IRQ sense selection GPIO3 input.
    gpio_irq_sense_value gisen2; // GPIO IRQ sense selection GPIO2 input.
    gpio_irq_sense_value gisen1; // GPIO IRQ sense selection GPIO1 input.
    gpio_irq_sense_value gisen0; // GPIO IRQ sense selection GPIO0 input.
} gpio_irq_sense_ctrl_format;

// Possibles interrupt modes of the IRQ GPIOS.
typedef enum {
    IRQ_LEVEL = 0b0,
    IRQ_EDGE = 0b1,
} gpio_irq_interrupt_mode_value;

// Structure of the GPIO Interrupt Mode (Level/Edge) sub-register.
typedef struct {
    gpio_irq_interrupt_mode_value gimod8; // GPIO IRQ mode selection for GPIO8 input.
    gpio_irq_interrupt_mode_value gimod7; // GPIO IRQ mode selection for GPIO7 input.
    gpio_irq_interrupt_mode_value gimod6; // GPIO IRQ mode selection for GPIO6 input.
    gpio_irq_interrupt_mode_value gimod5; // GPIO IRQ mode selection for GPIO5 input.
    gpio_irq_interrupt_mode_value gimod4; // GPIO IRQ mode selection for GPIO4 input.
    gpio_irq_interrupt_mode_value gimod3; // GPIO IRQ mode selection for GPIO3 input.
    gpio_irq_interrupt_mode_value gimod2; // GPIO IRQ mode selection for GPIO2 input.
    gpio_irq_interrupt_mode_value gimod1; // GPIO IRQ mode selection for GPIO1 input.
    gpio_irq_interrupt_mode_value gimod0; // GPIO IRQ mode selection for GPIO0 input.
} gpio_irq_mode_ctrl_format;

// Possibles interrupt both edge modes of the IRQ GPIOS.
typedef enum {
    IRQ_NO_BOTH_EDGES = 0b0, // GPIO_IMODE sub-register selects the edge.
    IRQ_BOTH_EDGES = 0b1,    // Both edges trigger the interrupt.
} gpio_irq_both_edges_mode_value;

// Structure of the GPIO Interrupt both edge select sub-register.
typedef struct {
    gpio_irq_both_edges_mode_value gibes8; // GPIO IRQ both edge selection for GPIO8 input.
    gpio_irq_both_edges_mode_value gibes7; // GPIO IRQ both edge selection for GPIO7 input.
    gpio_irq_both_edges_mode_value gibes6; // GPIO IRQ both edge selection for GPIO6 input.
    gpio_irq_both_edges_mode_value gibes5; // GPIO IRQ both edge selection for GPIO5 input.
    gpio_irq_both_edges_mode_value gibes4; // GPIO IRQ both edge selection for GPIO4 input.
    gpio_irq_both_edges_mode_value gibes3; // GPIO IRQ both edge selection for GPIO3 input.
    gpio_irq_both_edges_mode_value gibes2; // GPIO IRQ both edge selection for GPIO2 input.
    gpio_irq_both_edges_mode_value gibes1; // GPIO IRQ both edge selection for GPIO1 input.
    gpio_irq_both_edges_mode_value gibes0; // GPIO IRQ both edge selection for GPIO0 input.
} gpio_irq_both_edges_mode_format;

// Possibles GPIO Interrupt latch clear modes.
typedef enum {
    IRQ_LATCH_CLEAR = 0b1,
    IRQ_NO_LATCH_CLEAR = 0b0,
} gpio_irq_latch_clear_mode_value;

// Structure of the GPIO interrupt latch clear sub-register.
typedef struct {
    gpio_irq_latch_clear_mode_value giclr8; // GPIO IRQ latch clear for GPIO8 input.
    gpio_irq_latch_clear_mode_value giclr7; // GPIO IRQ latch clear for GPIO7 input.
    gpio_irq_latch_clear_mode_value giclr6; // GPIO IRQ latch clear for GPIO6 input.
    gpio_irq_latch_clear_mode_value giclr5; // GPIO IRQ latch clear for GPIO5 input.
    gpio_irq_latch_clear_mode_value giclr4; // GPIO IRQ latch clear for GPIO4 input.
    gpio_irq_latch_clear_mode_value giclr3; // GPIO IRQ latch clear for GPIO3 input.
    gpio_irq_latch_clear_mode_value giclr2; // GPIO IRQ latch clear for GPIO2 input.
    gpio_irq_latch_clear_mode_value giclr1; // GPIO IRQ latch clear for GPIO1 input.
    gpio_irq_latch_clear_mode_value giclr0; // GPIO IRQ latch clear for GPIO0 input.
} gpio_irq_latch_clear_mode_format;

// Possibles GPIO interrupt de-bounce enable modes.
typedef enum {
    IRQ_DE_BOUNCE_ENABLE = 0b1,
    IRQ_DE_BOUNCE_DISABLE = 0b0,
} gpio_irq_de_bounce_mode_value;

// Structure of the GPIO interrupt de-bounce sub-register.
typedef struct {
    gpio_irq_de_bounce_mode_value gidbe8; // GPIO8 IRQ de-bounce configuration.
    gpio_irq_de_bounce_mode_value gidbe7; // GPIO7 IRQ de-bounce configuration.
    gpio_irq_de_bounce_mode_value gidbe6; // GPIO6 IRQ de-bounce configuration.
    gpio_irq_de_bounce_mode_value gidbe5; // GPIO5 IRQ de-bounce configuration.
    gpio_irq_de_bounce_mode_value gidbe4; // GPIO4 IRQ de-bounce configuration.
    gpio_irq_de_bounce_mode_value gidbe3; // GPIO3 IRQ de-bounce configuration.
    gpio_irq_de_bounce_mode_value gidbe2; // GPIO2 IRQ de-bounce configuration.
    gpio_irq_de_bounce_mode_value gidbe1; // GPIO1 IRQ de-bounce configuration.
    gpio_irq_de_bounce_mode_value gidbe0; // GPIO0 IRQ de-bounce configuration.
} gpio_irq_de_bounce_mode_format;

// Possibles GPIO raw states.
typedef enum {
    GPIO_HAS_RAW = 0b1,
    GPIO_HAS_NO_RAW = 0b0,
} gpio_raw_state_value;

// Structure of the GPIO raw state sub-register.
typedef struct {
    gpio_raw_state_value grawp8; // This bit reflects the raw state of GPIO8.
    gpio_raw_state_value grawp7; // This bit reflects the raw state of GPIO7.
    gpio_raw_state_value grawp6; // This bit reflects the raw state of GPIO6.
    gpio_raw_state_value grawp5; // This bit reflects the raw state of GPIO5.
    gpio_raw_state_value grawp4; // This bit reflects the raw state of GPIO4.
    gpio_raw_state_value grawp3; // This bit reflects the raw state of GPIO3.
    gpio_raw_state_value grawp2; // This bit reflects the raw state of GPIO2.
    gpio_raw_state_value grawp1; // This bit reflects the raw state of GPIO1.
    gpio_raw_state_value grawp0; // This bit reflects the raw state of GPIO0.
} gpio_raw_state_format;

/**
 * @brief Formats a spi_frame to a gpio_ctrl_format.
 *
 * @param[in] fr: spi_frame to initialize the gpio_ctrl_format(format).
 * @param[out] format: Structure which will contains the spi_frame formatted.
 * @param[in] sub_register: Enumerate that indicates the sub-register.
 *
 * @note: This function must receive an gpio_ctrl_format to work.
 *
 */
void gpio_ctrl_formater(spi_frame fr, void *format, const size_t sub_register);

/**
 * @brief Unformats a gpio_ctrl_format to a spi_frame.
 *
 * @param[in] format: Structure which contains the values of the fields of the GPIO_CTRL register.
 * @param[out] fr: spi_frame where this function will store the gpio_ctrl_format structure.
 * @param[in] sub_register: Enumerate that indicates the sub-register.
 *
 * @return size_t: Size of the spi_frame formed with the given gpio_ctrl_format structure.
 *
 * @note: This function must receive a gpio_ctrl_format value to work.
 *
 */
size_t gpio_ctrl_unformater(void *format, spi_frame fr, const size_t sub_register);

/******* DRX_CONF *******/

// Sub-registers of the digital receiver configuration register.
typedef enum {
    DRX_TUNE0B = 0x02,
    DRX_TUNE1A = 0x04,
    DRX_TUNE1B = 0x06,
    DRX_TUNE2 = 0x08,
    DRX_SFDTOC = 0x20,
    DRX_PRETOC = 0x24,
    DRX_TUNE4H = 0x26,
    DRX_CAR_INT = 0x28,
    RXPACC_NOSAT = 0x2C,
} drx_conf_subregister;

// Structure of the digital receiver configuration register.
typedef struct {
    uint16_t rxpacc_nosat;        // Digital debug register. (RO)
    unsigned int drx_car_int :21; // Carrier recovery integrator. (RO)
    uint16_t drx_tune4h;          // Digital tuning register (RW)
    uint16_t drx_pretoc;          // Preamble detection timeout count. (RW)
    uint16_t drx_sfdtoc;          // SFD detection timeout count. (RW)
    uint32_t drx_tune2;           // Digital tuning register 2. (RW)
    uint16_t drx_tune1b;          // Digital tuning register 1b. (RW)
    uint16_t drx_tune1a;          // Digital tuning register 1a. (RW)
    uint16_t drx_tune0b;          // Digital tuning register 0b. (RW)
} drx_conf_format;

/**
 * @brief Formats a spi_frame to a drx_conf_format.
 *
 * @param[in] fr: spi_frame to initialize the drx_conf_format(format).
 * @param[out] format: Structure which will contains the spi_frame formatted.
 * @param[in] sub_register: Enumerate that indicates the sub-register.
 *
 * @note: This function must receive an drx_conf_format to work.
 *
 */
void drx_conf_formater(spi_frame fr, void *format, const size_t sub_register);

/**
 * @brief Unformats a drx_conf_format to a spi_frame.
 *
 * @param[in] format: Structure which contains the values of the fields of the DRX_CONF register.
 * @param[out] fr: spi_frame where this function will store the gpio_ctrl_format structure.
 * @param[in] sub_register: Enumerate that indicates the sub-register.
 *
 * @return size_t: Size of the spi_frame formed with the given drx_conf_format structure.
 *
 * @note: This function must receive a drx_conf_format value to work.
 *
 */
size_t drx_conf_unformater(void *format, spi_frame fr, const size_t sub_register);

/******* RF_CONF *******/

// Sub-registers of the analog RF configuration register.
typedef enum {
    SRF_CONF = 0x00,
    RF_RXCTRLH = 0x0B,
    RF_TXCTRL = 0x0C,
    RF_STATUS = 0x2C,
    LDOTUNE = 0x30,
} rf_conf_subregister;

// Possible values of the txrxsw field.
typedef enum {
    RF_TX = 0x2,
    RF_RX = 0X1,
} txrxsw_value;

// Possible values of the ldofen field.
typedef enum {
    ENEABLE_LDOS = 0x1F,
    DISABLE_LDOS = 0X00,
} ldofen_value;

// Possible values of the pllfen field.
typedef enum {
    ENEABLE_CLK_PLL = 0x5,
    ENEABLE_CLK_PLL_AND_RF_PLL = 0x7,
} pllfen_value;

// Possible values of the txfen field.
typedef enum {
    FORCE_ALL_TX_BLOCKS_ON = 0x1F,
    FORCE_ALL_TX_BLOCKS_OFF = 0x00,
} txfen_value;

// Structure of the analog RF configuration register.
typedef struct {
    uint64_t ldotune;          // Controls the output voltage levels of the on chip LDOs. (RW)
    unsigned int rfplllock: 1; // RF PLL lock status. (RO)
    unsigned int cpllhigh: 1;  // Clock PLL high flag status bit. (RO)
    unsigned int cplllow: 1;   // Clock PLL low flag status bit. (RO)
    unsigned int cplllock: 1;  // Clock PLL lock status. (RO)
    unsigned int txmq :3;      // Transmit mixer Q-factor tuning register. (RW)
    unsigned int txmtune :4;   // Transmit mixer tuning register. (RW)
    uint8_t rfrxctrlh;         // Analog RX control register. (RW)
    txrxsw_value txrxsw;       // Force the TX/RX switch. (RW)
    ldofen_value ldofen;       // Force the enable to all LDO’s. (RW)
    pllfen_value pllfen;       // PLL block force enables. (RW)
    txfen_value txfen;         // Transmit block force enable. (RW)
} rf_conf_format;

/**
 * @brief Formats a spi_frame to a rf_conf_format.
 *
 * @param[in] fr: spi_frame to initialize the rf_conf_format(format).
 * @param[out] format: Structure which will contains the spi_frame formatted.
 * @param[in] sub_register: Enumerate that indicates the sub-register.
 *
 * @note: This function must receive an rf_conf_format to work.
 *
 */
void rf_conf_formater(spi_frame fr, void *format, const size_t sub_register);

/**
 * @brief Unformats a rf_conf_format to a spi_frame.
 *
 * @param[in] format: Structure which contains the values of the fields of the RF_CONF register.
 * @param[out] fr: spi_frame where this function will store the rf_conf_format structure.
 * @param[in] sub_register: Enumerate that indicates the sub-register.
 *
 * @return size_t: Size of the spi_frame formed with the given rf_conf_format structure.
 *
 * @note: This function must receive a rf_conf_format value to work.
 *
 */
size_t rf_conf_unformater(void *format, spi_frame fr, const size_t sub_register);

/******* TX_CAL *******/

// Sub-registers of the transmitter calibration block register.
typedef enum {
    TC_SARC = 0x00,
    TC_SARL = 0x03,
    TC_SARW = 0x06,
    TC_PG_CTRL = 0x08,
    TC_PG_STATUS = 0x09,
    TC_PG_DELAY = 0x0B,
    TC_PG_TEST = 0x0C,
} tx_cal_subregister;

#define SAR_TEMP_LEAST_BIT_VALUE 1.14 // (degrees centigrade)
#define calculate_temperature(register_value) register_value * SAR_TEMP_LEAST_BIT_VALUE

#define SAR_BAT_LEAST_BIT_VALUE 0.006 // (volts)
#define calculate_voltage(register_value) register_value * SAR_BAT_LEAST_BIT_VALUE

// Structure of the transmitter calibration block register.
typedef struct {
    uint8_t tc_pgtest;          // Pulse generator test. (RW)
    uint8_t tc_pgdelay;         // Pulse generator delay. (RW)
    unsigned int delay_cnt: 12; // PG status. (RO)
    unsigned int pg_start: 1;   // Start the pulse generator calibration. Note: This bit is self clearing. (RW)
    unsigned int pg_tmeas: 4;   // Number of clock cycles over which to run the pulse generator cal counter. (RW)
    double sar_wbat;            // SAR reading of Voltage level taken at last wakeup event. (RO)
    double sar_wtemp;           // SAR reading of temperature level taken at last wakeup event. (RO)
    double sar_lvbat;           // Latest SAR reading for Voltage level. (RO)
    double sar_ltemp;           // Latest SAR reading for Temperature level. (RO)
    unsigned int sar_ctrl :1;   // Writing 1 sets SAR enable and writing 0 clears the enable. (RW)
} tx_cal_format;

/**
 * @brief Formats a spi_frame to a tx_cal_format.
 *
 * @param[in] fr: spi_frame to initialize the tx_cal_format(format).
 * @param[out] format: Structure which will contains the spi_frame formatted.
 * @param[in] sub_register: Enumerate that indicates the sub-register.
 *
 * @note: This function must receive an tx_cal_format to work.
 *
 */
void tx_cal_formater(spi_frame fr, void *format, const size_t sub_register);

/**
 * @brief Unformats a tx_cal_format to a spi_frame.
 *
 * @param[in] format: Structure which contains the values of the fields of the TX_CAL register.
 * @param[out] fr: spi_frame where this function will store the tx_cal_format structure.
 * @param[in] sub_register: Enumerate that indicates the sub-register.
 *
 * @return size_t: Size of the spi_frame formed with the given tx_cal_format structure.
 *
 * @note: This function must receive a tx_cal_format value to work.
 *
 */
size_t tx_cal_unformater(void *format, spi_frame fr, const size_t sub_register);

/******* FS_CTRL *******/

// Sub-registers of the frequency synthesizer control block register.
typedef enum {
    FS_PLLCFG = 0x07,
    FS_PLLTUNE = 0x0B,
    FS_XTALT = 0x0E,
} fs_ctrl_subregister;

// Structure of the frequency synthesizer control block register.
typedef struct {
    unsigned int xtalt :5; // Crystal trim. (RW)
    uint8_t fs_plltune;    // PLL Tuning. (RW)
    uint32_t fs_pllcfg;    // PLL configuration. (RW)
} fs_ctrl_format;

/**
 * @brief Formats a spi_frame to a fs_ctrl_format.
 *
 * @param[in] fr: spi_frame to initialize the fs_ctrl_format(format).
 * @param[out] format: Structure which will contains the spi_frame formatted.
 * @param[in] sub_register: Enumerate that indicates the sub-register.
 *
 * @note: This function must receive an fs_ctrl_format to work.
 *
 */
void fs_ctrl_formater(spi_frame fr, void *format, const size_t sub_register);

/**
 * @brief Unformats a fs_ctrl_format to a spi_frame.
 *
 * @param[in] format: Structure which contains the values of the fields of the FS_CTRL register.
 * @param[out] fr: spi_frame where this function will store the fs_ctrl_format structure.
 * @param[in] sub_register: Enumerate that indicates the sub-register.
 *
 * @return size_t: Size of the spi_frame formed with the given fs_ctrl_format structure.
 *
 * @note: This function must receive a fs_ctrl_format value to work.
 *
 */
size_t fs_ctrl_unformater(void *format, spi_frame fr, const size_t sub_register);

#endif // _FORMAT_H_
