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

#ifndef _DW1000_H_
#define _DW1000_H_

#include "register.h"

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
 * @brief Disables SPI driver and turn off the dwm1000.
 *
 */
void dw_disable(void);

/**
 * @brief Resets the dw1000 and enables SPI driver.
 *
 * @return bool: Size of the spi_frame formed with the given eui_format structure.
 *
 * @note: This function must be called before any other function.
 *
 */
bool dw_eneable(void);

/**
 * @brief Resets the dw1000.
 *
 */
void dw_reset(void);

#endif /* _DWM1000_H_ */
