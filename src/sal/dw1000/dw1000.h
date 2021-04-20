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

#ifndef _DW1000_H_
#define _DW1000_H_

#include "register.h"

#define DW_POWER_ON  palSetPad(IOPORT1, DW_RST);
#define DW_POWER_OFF palClearPad(IOPORT1, DW_RST);

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

/**
 * @brief Gets the device id register value.
 *
 * @param[out] dev_id_f: Structure in will be stored the value of the register.
 *
 * @return bool: Returns true if the register can be gotten, otherwise false.
 *
 * @note: dev_id_f must be equal to DEFINED_DEV_ID =
 *        { .ridtag = 0XDECA, .model = 0X01, .ver = 0X03, .rev = 0x00} variable.
 *
 */
bool get_dev_id(dev_id_format* dev_id_f);

/**
 * @brief Gets the extended unique identifier register value.
 *
 * @param[out] eui_f: Structure in will be stored the value of the register.
 *
 * @return bool: Returns true if the register can be gotten, otherwise false.
 *
 */
bool get_eui(eui_format* eui_f);

/**
 * @brief Sets the extended unique identifier register value.
 *
 * @param[out] eui_f: Structure that contains the value that will be written in the register.
 *
 * @return bool: Returns true if the register can be set, otherwise false.
 *
 */
bool set_eui(eui_format* eui_f);

/**
 * @brief Gets the pan address register value.
 *
 * @param[out] pan_adr_f: Structure in will be stored the value of the register.
 *
 * @return bool: Returns true if the register can be gotten, otherwise false.
 *
 */
bool get_pan_adr(pan_adr_format* pan_adr_f);

/**
 * @brief Sets the pan address register value.
 *
 * @param[out] pan_adr_f: Structure in will be stored the value of the register.
 *
 * @return bool: True if the register can be set, otherwise false.
 *
 */
bool set_pan_adr(pan_adr_format* pan_adr_f);

/**
 * @brief Gets the system configuration register value.
 *
 * @param[out] sys_cfg_f: Structure in will be stored the value of the register.
 *
 * @return bool: Returns true if the register can be gotten, otherwise false.
 *
 */
bool get_sys_cfg(sys_cfg_format* sys_cfg_f);

/**
 * @brief Sets the system configuration register value.
 *
 * @param[out] sys_cfg_f: Structure that contains the value that will be written in the register.
 *
 * @return bool: Returns true if the register can be set, otherwise false.
 *
 */
bool set_sys_cfg(sys_cfg_format* sys_cfg_f);

/**
 * @brief Gets the system time register value.
 *
 * @param[out] seconds: Variable in will be stored the value of the register.
 *
 * @return bool: Returns true if the register can be gotten, otherwise false.
 *
 * @note: This register saves a limited amount of time, defined in COUNTER_WRAP_PERIOD = 17.2074.
 *        That means that after 17.2074 seconds, the register restarts.
 *
 */
bool get_sys_time(double* seconds);

/**
 * @brief Gets the transmit frame control register value.
 *
 * @param[out] seconds: Structure in will be stored the value of the register.
 *
 * @return bool: Returns true if the register can be gotten, otherwise false.
 *
 */
bool get_tx_fctrl(tx_fctrl_format* tx_fctrl_f);

/**
 * @brief Sets the transmit frame control register value.
 *
 * @param[out] pan_adr_f: Structure in will be stored the value of the register.
 *
 * @return bool: True if the register can be set, otherwise false.
 *
 */
bool set_tx_fctrl(tx_fctrl_format* tx_fctrl_f);

/**
 * @brief Sets the transmit buffer register value.
 *
 * @param[out] frame: Message to store in the transmit buffer register.
 * @param[in] frame_size: Size in bytes of the given uwb_frame.
 *
 * @return bool: True if the register can be set, otherwise false.
 *
 */
bool set_tx_buffer(uwb_frame frame, const uint16_t frame_size);

/**
 * @brief Gets the delayed send/receiver register value.
 *
 * @param[out] seconds: Variable in will be stored the value of the register.
 *
 * @return bool: Returns true if the register can be gotten, otherwise false.
 *
 */
bool get_dx_time(double* seconds);

/**
 * @brief Sets the delayed send/receiver register value.
 *
 * @param[in] seconds: Value to set to the register.
 *
 * @return bool: Returns true if the register can be set, otherwise false.
 * @note: seconds must be < COUNTER_WRAP_PERIOD(17.2074)
 *
 */
bool set_dx_time(double* seconds);

/**
 * @brief Gets the receive frame wait timeout period register value.
 *
 * @param[out] seconds: Variable in will be stored the value of the register.
 *
 * @return bool: Returns true if the register can be gotten, otherwise false.
 *
 */
bool get_rx_fwto(double* seconds);

/**
 * @brief Sets the receive frame wait timeout period register value.
 *
 * @param[in] seconds: Value to set to the register.
 *
 * @return bool: Returns true if the register can be set, otherwise false.
 *
 */
bool set_rx_fwto(double* seconds);

/**
 * @brief Gets the system control register value.
 *
 * @param[in] sys_ctrl_f: Structure in will be stored the value of the register.
 *
 * @return bool: Returns true if the register can be set, otherwise false.
 *
 */
bool get_sys_ctrl(sys_ctrl_format* sys_ctrl_f);

/**
 * @brief Sets the system control register value.
 *
 * @param[in] sys_ctrl_f: Structure that contains the value that will be written in the register.
 *
 * @return bool: Returns true if the register can be set, otherwise false.
 *
 */
bool set_sys_ctrl(sys_ctrl_format* sys_ctrl_f);

/**
 * @brief Gets the system event mask register value.
 *
 * @param[in] sys_evt_msk_f: Structure in will be stored the value of the register.
 *
 * @return bool: Returns true if the register can be set, otherwise false.
 *
 */
bool get_sys_event_msk(sys_evt_msk_format* sys_evt_msk_f);

/**
 * @brief Sets the system event mask register value.
 *
 * @param[in] sys_evt_msk_f: Structure that contains the value that will be written in the register.
 *
 * @return bool: Returns true if the register can be set, otherwise false.
 *
 */
bool set_sys_event_msk(sys_evt_msk_format* sys_evt_msk_f);

/**
 * @brief Gets the system event status register register value.
 *
 * @param[in] sys_evt_sts_f: Structure in will be stored the value of the register.
 *
 * @return bool: Returns true if the register can be set, otherwise false.
 *
 */
bool get_sys_event_sts(sys_evt_sts_format* sys_evt_sts_f);

/**
 * @brief Sets the system event status register value.
 *
 * @param[in] sys_evt_sts_f: Structure that contains the value that will be written in the register.
 *
 * @return bool: Returns true if the register can be set, otherwise false.
 *
 */
bool set_sys_event_sts(sys_evt_sts_format* sys_evt_sts_f);

#endif /* _DWM1000_H_ */
