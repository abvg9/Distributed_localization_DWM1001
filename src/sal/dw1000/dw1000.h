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

#ifndef _DW1000_H_
#define _DW1000_H_

#include "register.h"
#include "params.h"

#define dw_power_on  palSetPad(IOPORT1, DW_RST)
#define dw_power_off palClearPad(IOPORT1, DW_RST)

/**
 * @brief Calculates the clock offset between two dw1000 devices.
 *
 * @param[in] rx_ttcko_f: Structure which contains the receiver time tracking offset.
 * @param[in] rx_ttcki: Value that contains the receiver time tracking interval.
 *
 * @return double: Clock offset in ppm units.
 *
 */
double calc_clock_offset(const rx_ttcko_format rx_ttcko_f, const rx_ttcki_value rx_ttcki);

/**
 * @brief Calculates the estimated signal power of a received message.
 *
 * @param[in] rx_fqual_f: Structure which contains the fp_ampl2 and fp_ampl3 values.
 * @param[in] rx_finfo_f: Structure which contains the rxpacc and rxprfr values.
 * @param[in] usr_sfd_f: Structure which contains the the sfd_length value(SFD symbol count).
 *
 * @note: Estimated power level = 10 * log10( cir_pwr * 2¹⁷/ rxpacc-sfd_length) - rxprfr
 * @note: This function may be used to check the deviation of the calculate_signal_power()
 *        function.
 *
 * @return double: Estimated signal power in units of dBm.
 *
 */
double calc_estimated_signal_power(const rx_fqual_format rx_fqual_f,
        const rx_finfo_format rx_finfo_f, const usr_sfd_format usr_sfd_f);

/**
 * @brief Calculates the frequency error of a packet received.
 *
 * @param[in] drx_conf_f: Structure which contains the drx_car_int value.
 * @param[in] rx_finfo_f: Structure which contains the transmit_receive_bit_rate value,
 *            with this value we can calculate the n_samples constant.
 *
 * @note: Frequency error = (drx_car_int * 2⁻¹⁷)/2(n_samples/Fs)
 *
 * @return double: Frequency error in Hz.
 *
 */
double calc_freq_error(const drx_conf_format drx_conf_f, const rx_finfo_format rx_finfo_f);

/**
 * @brief Calculates the noise energy level.
 *
 * @param[in] agc_ctrl_f: Structure which contains the edv2 and edg1 values.
 * @param[in] chan_ctrl_f: Structure which contains the ch value.
 *
 * @note: Noise energy level = (EVD2 - 40) * 10^EDG1 * S
 * @note: if ch = 1 to 4 => S = 1.3335
 *           ch = 5 or 7 => S = 1.0000
 *
 * @note: This function does not give an absolute level but instead gives a relative level that allows
 *        comparison between channels in order to select the channel with least noise.
 *
 * @return double: Estimated noise energy level.
 *
 */
double calc_noise_energy_level(const agc_ctrl_format agc_ctrl_f, const chan_ctrl_format chan_ctrl_f);

/**
 * @brief Calculates the signal power of the received message.
 *
 * @param[in] rx_time_f: Structure which contains the fp_ampl1 value.
 * @param[in] rx_fqual_f: Structure which contains the fp_ampl2 and fp_ampl3 values.
 * @param[in] rx_finfo_f: Structure which contains the rxpacc and rxprfr values.
 * @param[in] usr_sfd_f: Structure which contains the sfd_length value(SFD symbol count).
 *
 * @note: Power level = 10 * log10( (fp_ampl1² + fp_ampl2² + fp_ampl3²)/ rxpacc-sfd_length) - rxprfr
 * @note: The resultant power level may be compared with the estimated signal power.
 *
 * @return double: Signal power in units of dBm.
 *
 */
double calc_signal_power(const rx_time_format rx_time_f, const rx_fqual_format rx_fqual_f,
        const rx_finfo_format rx_finfo_f, const usr_sfd_format usr_sfd_f);

/**
 * @brief Calculates the distance between two dw1000 devices.
 *
 * @param[in] dev_id: Device identifier with which you want to calculate distance.
 * @param[out] distance: Voltage of the dw1000.
 * @param[in] attempts: Number of attempts to calculate distance.
 *
 * @return bool: Returns true if the distance can be calculated, otherwise false.
 *
 */
bool dw_calculate_distance(const uint64_t dev_id, double distance, const int attempts, const int wait_timer);

/**
 * @brief This function provides the main API for the configuration of the
 *        DW1000 and this low-level driver.
 *
 * @return bool: True if the device can be configured, otherwise false.
 */
bool dw_configure(void);

/**
 * @brief Disables SPI driver and turn off the dwm1000.
 *
 */
void dw_disable(void);

/**
 * @brief Resets the dw1000, enables SPI driver and initializes the dw1000.
 *
 * @param config_flags: Indicates which options of the dw_local_data_t should be initialized.
 *
 * @return bool: True if the device can be enabled, otherwise false.
 *
 * @note: It is mandatory to call this function before any other function call of this API.
 *
 */
bool dw_eneable(const int config_flags);

/**
 * @brief This is used to read the OTP data from given address.
 *
 * input parameters
 * @param[in] address: This is the OTP address to read from.
 * @param[out] read_value: This is the value of the OTP address.
 *
 * @return bool: True if the memory direction can be read, otherwise false
 */
bool dw_get_otp_value(const otp_memory_direction mem_dir, uint32_t* read_value);

/**
 * @brief Gets the temperature and voltage of the dw1000.
 *
 * @param[out] temperature: Temperature of the dw1000.
 * @param[out] voltage: Voltage of the dw1000.
 *
 * @return bool: Returns true if the temperature and voltage can be gotten, otherwise false.
 *
 * @note: To use this function is mandatory to call dw_initialise function with DW_READ_OTP_BAT | DW_READ_OTP_TMP flags.
 */
bool dw_get_voltage_bat_and_temp(double* temperature, double* voltage);

// Initialize flags.
#define DW_LOADNONE 0x00         // No loading of micro-code or reading of OTP values.
#define DW_LOADUCODE 0x01        // This can be called on power up or after wake up to load ucode.
#define DW_DW_WAKE_UP 0x02       // Init after wake up - will not load ucode / ucode will not run.
#define DW_DW_WUP_NO_UCODE  0x04 // Init after wake up - ucode has not already been loaded / ucode is not used.
#define DW_DW_WUP_RD_OTPREV 0x08 // Init after wakeup - read OTP rev after wake up.
#define DW_READ_OTP_PID 0x10     // Read part ID from OTP.
#define DW_READ_OTP_LID 0x20     // Read lot ID from OTP.
#define DW_READ_OTP_BAT 0x40     // Read ref voltage from OTP.
#define DW_READ_OTP_TMP 0x80     // Read ref temperature from OTP.

/**
 * @brief Initializes the dw1000.
 *
 * @param config_flags: Indicates which options of the dw_local_data_t should be initialized.
 *
 * @return bool: True if the device can be initialized, otherwise false.
 *
 */
bool dw_initialise(const int config_flags);

// Defined constants for "mode" bitmask parameter passed into dw_receive_message() function.
#define DW_START_RX_IMMEDIATE  0
#define DW_START_RX_DELAYED    1 // Set up delayed RX, if "late" error triggers, then the RX will be enabled immediately
#define DW_IDLE_ON_DLY_ERR     2 // If delayed RX failed due to "late" error then if this
                                 // flag is set the RX will not be re-enabled immediately, and device will be in IDLE when function exits
#define DW_NO_SYNC_PTRS        4 // Do not try to sync IC side and Host side buffer pointers when enabling RX. This is used to perform manual RX
                                 // re-enabling when receiving a frame in double buffer mode.

/**
 * @brief Waits for receive a message.
 *
 * @param frame[in]: Will contain the received message.
 * @param mode[in]: DWT_START_RX_IMMEDIATE used to enable receiver immediately.
 *                  DWT_START_RX_DELAYED used to set up delayed RX, if "late" error triggers, then the RX will be enabled immediately.
 *                  DWT_START_RX_DELAYED | DWT_IDLE_ON_DLY_ERR used to disable re-enabling of receiver if delayed RX failed due to "late" error.
 *                  DWT_START_RX_IMMEDIATE | DWT_NO_SYNC_PTRS used to re-enable RX without trying to sync IC and host side buffer pointers, typically when
 *                  performing manual RX re-enabling in double buffering mode.
 * @param wait_timer[in]: Timer that determines the number of seconds(aprox) that will be waiting for a message.
 *                        If wait_timer < 0, it will wait until the expected message be received(infinitely).
 *                        If wait_timer < SDF_TO, it will wait until the SDF time out expires.
 * @param dev_id[in]: Identifier of the sender. If this value is equal to zero, means that it will
 *                    wait for any sender.
 *
 * @note: See function get_rx_buffer to understand better this function.
 *
 * @return bool: Returns true if the message was received, otherwise false.
 *
 */
bool dw_receive_message(uwb_frame_format* frame, const uint8_t mode, const int wait_timer, const uint64_t dev_id);

/**
 * @brief Resets the dw1000.
 *
 */
void dw_reset(void);

// Defined constants for "mode" bitmask parameter passed into dw_send_message() function.
#define DW_START_TX_IMMEDIATE 0
#define DW_START_TX_DELAYED 1
#define DW_RESPONSE_EXPECTED 2

/**
 * @brief Send a message through UWB.
 *
 * @param frame[in]: Message to send.
 * @param frame_size[in]: This is the length of TX message (including the 2 byte CRC) - max is 1023.
 * @param ranging[in]: True if this is a ranging frame, else false.
 * @param mode[in]: DWT_START_TX_IMMEDIATE immediate TX (no response expected).
 *                  DWT_START_TX_DELAYED delayed TX (no response expected).
 *                  DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED - immediate TX (response expected - so the receiver will be automatically turned on after TX is done).
 *                  DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED - delayed TX (response expected - so the receiver will be automatically turned on after TX is done).
 *
 * @note: Standard PHR mode allows up to 127 bytes
 *        if > 127 is programmed, DWT_PHRMODE_EXT needs to be set in the phrMode configuration
 *        see dw_configure function.
 *
 * @note: See function set_tx_buffer to understand better this function.
 *
 * @return bool: Returns true if the message can be sent, otherwise false.
 *
 */
bool dw_send_message(uwb_frame_format* frame, const bool ranging, const uint8_t mode);

/**
 * @brief Sets the rate SPI communication of the dwm1000(8MBPS).
 *
 */
void dw_set_fast_spi_rate(void);

/**
 * @brief This is used to set up TX/RX GPIOs which could be used to control LEDs.
 *
 * @param enable[in] Enable LEDs.
 *
 * @return bool: Returns true if the leds can be set, otherwise false.
 *
 */
bool dw_set_leds(const bool enable);

/**
 * @brief Sets the rate SPI communication of the dwm1000(2MBPS).
 *
 */
void dw_set_slow_spi_rate(void);

/**
 * @brief This is used to turn off the transceiver.
 *
 * @return bool: Returns true if the transceiver can turned off, otherwise false.
 *
 */
bool dw_turn_off_transceiver(void);

/**
 * @brief Waits until the pin DW_IRQ is HIGH and return which of the IRQ events is/are active.
 *
 * @param sys_evt_msk_f[in]: Interruptions to enable and check.
 *
 * @return sys_evt_sts_format: Structure that contains which IRQ events is/are active.
 *
 * @note: This function must be called before any other function.
 *
 */
sys_evt_sts_format dw_wait_irq_event(sys_evt_msk_format sys_evt_msk_f);

/**
 * @brief Initializes the uwb_frame_format container.
 *
 * @param buffer[in]: Message to store in the container.
 * @param buffer_size[in]: Size of the message.
 * @param messagge_type[in]: Message type according to the IEEE 802.15.4 standard.
 * @param addr_mod[in]: Address mode (short address or extended address).
 * @param uwb_frame_f[out]: uwb_frame_format container initialized.
 *
 * @return bool: If the container could be initialized, otherwise false.
 *
 * @note: buffer_size must be lower or equal than TX_RX_BUFFER_MAX_SIZE.
 * @note: This function only fill the minimum required fields. It is mandatory to externally
 *        set other fields after calling this function, see uwb_frame_format struct definition
 *        to better understand.
 *
 */
bool init_uwb_frame_format(uint8_t* buffer, const size_t buffer_size,
        const frame_type_value messagge_type, const dest_sour_addr_mod_value addr_mod,
        uwb_frame_format* uwb_frame_f);

#endif /* _DWM1000_H_ */
