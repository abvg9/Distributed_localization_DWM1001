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
    RESERVED_8  = 0X20,
    USR_SFD     = 0X21,
    RESERVED_9  = 0X22,
    AGC_CTRL    = 0X23,
    EXT_SYNC    = 0X24,
    ACC_MEM     = 0X25,
    GPIO_CTRL   = 0X26,
    DRX_CONF    = 0X27,
    RF_CONF     = 0x28,
    RESERVED_10 = 0X29,
    TX_CAL      = 0X2A,
    FS_CTRL     = 0X2B,
    AON         = 0X2C,
    OTP_IF      = 0X2D,
    LDE_CTRL    = 0X2E,
    DIG_DIAG    = 0X2F,
    RESERVED_11 = 0X30,
    RESERVED_12 = 0X31,
    RESERVED_13 = 0X32,
    RESERVED_14 = 0X33,
    RESERVED_15 = 0X34,
    RESERVED_16 = 0X35,
    PMSC        = 0X36,
    RESERVED_17 = 0X37,
    RESERVED_18 = 0X38,
    RESERVED_19 = 0X39,
    RESERVED_20 = 0X3A,
    RESERVED_21 = 0X3B,
    RESERVED_22 = 0X3C,
    RESERVED_23 = 0X3D,
    RESERVED_24 = 0X3E,
    RESERVED_25 = 0X3F
} register_id;

// Factory value of the dev_id register.
extern const dev_id_format DEFINED_DEV_ID;

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
 * @note: Depend on the phr_mode selected in the system configuration register
 *        this buffer can be from 127 bytes of size(phr_mode = standard) or
 *        1024 bytes(phr_mode = long_size).
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
 * @return bool: Returns true if the register can be gotten, otherwise false.
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
 * @return bool: Returns true if the register can be gotten, otherwise false.
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
 * @return bool: Returns true if the register can be gotten, otherwise false.
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

/**
 * @brief Gets the rx frame information register value.
 *
 * @param[in] rx_finfo_f: Structure in will be stored the value of the register.
 *
 * @return bool: Returns true if the register can be gotten, otherwise false.
 *
 */
bool get_rx_finfo(rx_finfo_format* rx_finfo_f);

/**
 * @brief Gets the receive buffer register value.
 *
 * @param[out] frame: Array in which will be load the transmit buffer register.
 * @param[in] frame_size: Number of bytes to read from the transmit buffer register.
 *
 * @note: Depend on the phr_mode selected in the system configuration register
 *        this buffer can be from 127 bytes of size(phr_mode = standard) or
 *        1024 bytes(phr_mode = long_size).
 *
 * @return bool: True if the register can be set, otherwise false.
 *
 */
bool get_rx_buffer(uwb_frame frame, const uint16_t frame_size);

/**
 * @brief Gets the rx frame quality information register value.
 *
 * @param[in] rx_fqual_f: Structure in will be stored the value of the register.
 *
 * @return bool: Returns true if the register can be gotten, otherwise false.
 *
 */
bool get_rx_fqual(rx_fqual_format* rx_fqual_f);

/**
 * @brief Gets the receiver time tracking interval register value.
 *
 * @param[in] rx_ttcki: Enumerate in will be stored the value of the register.
 *
 * @return bool: Returns true if the register can be gotten, otherwise false.
 *
 */
bool get_rx_ttcki(rx_ttcki_value* rx_ttcki);

/**
 * @brief Gets the receiver time tracking offset register value.
 *
 * @param[in] rx_ttcko_f: Structure in will be stored the value of the register.
 *
 * @return bool: Returns true if the register can be gotten, otherwise false.
 *
 */
bool get_rx_ttcko(rx_ttcko_format* rx_ttcko_f);

/**
 * @brief Gets the receiver time stamp register value.
 *
 * @param[in] rx_time_f: Structure in will be stored the value of the register.
 *
 * @return bool: Returns true if the register can be gotten, otherwise false.
 *
 */
bool get_rx_time(rx_time_format* rx_time_f);

/**
 * @brief Gets the transmitter time stamp register value.
 *
 * @param[in] tx_time_f: Structure in will be stored the value of the register.
 *
 * @return bool: Returns true if the register can be gotten, otherwise false.
 *
 */
bool get_tx_time(tx_time_format* tx_time_f);

/**
 * @brief Gets the transmitter antenna delay register value.
 *
 * @param[in] seconds: Value in will be stored the value of the register.
 *
 * @return bool: Returns true if the register can be gotten, otherwise false.
 *
 */
bool get_tx_antd(double* seconds);

/**
 * @brief Sets the transmitter antenna delay register value.
 *
 * @param[in] seconds: Value that contains the value that will be written in the register.
 *
 * @return bool: Returns true if the register can be set, otherwise false.
 *
 */
bool set_tx_antd(double* seconds);

/**
 * @brief Gets the system status register value.
 *
 * @param[in] sys_status_f: Structure in will be stored the value of the register.
 *
 * @return bool: Returns true if the register can be gotten, otherwise false.
 *
 */
bool get_sys_status(sys_status_format* sys_status_f);

/**
 * @brief Gets the acknowledgment time and response time register value.
 *
 * @param[in] ack_resp_t_f: Structure in will be stored the value of the register.
 *
 * @return bool: Returns true if the register can be gotten, otherwise false.
 *
 */
bool get_ack_resp_t(ack_resp_t_format* ack_resp_t_f);

/**
 * @brief Sets the acknowledgment time and response time register value.
 *
 * @param[in] ack_resp_t_f: Structure that contains the value that will be written in the register.
 *
 * @return bool: Returns true if the register can be set, otherwise false.
 *
 */
bool set_ack_resp_t(ack_resp_t_format* ack_resp_t_f);

/**
 * @brief Gets the sniff mode configuration register value.
 *
 * @param[in] rx_sniff_f: Structure in will be stored the value of the register.
 *
 * @return bool: Returns true if the register can be gotten, otherwise false.
 *
 */
bool get_rx_sniff(rx_sniff_format* rx_sniff_f);

/**
 * @brief Sets the sniff mode configuration register value.
 *
 * @param[in] rx_sniff_f: Structure that contains the value that will be written in the register.
 *
 * @return bool: Returns true if the register can be set, otherwise false.
 *
 */
bool set_rx_sniff(rx_sniff_format* rx_sniff_f);

/**
 * @brief Gets the tx power control register value.
 *
 * @param[in] tx_power_f: Structure in will be stored the value of the register.
 *
 * @return bool: Returns true if the register can be gotten, otherwise false.
 *
 */
bool get_tx_power(tx_power_format* tx_power_f);

/**
 * @brief Sets the tx power control register value.
 *
 * @param[in] tx_power_f: Structure that contains the value that will be written in the register.
 *
 * @return bool: Returns true if the register can be set, otherwise false.
 *
 */
bool set_tx_power(tx_power_format* tx_power_f);

/**
 * @brief Gets the channel control register value.
 *
 * @param[in] chan_ctrl_f: Structure in will be stored the value of the register.
 *
 * @return bool: Returns true if the register can be gotten, otherwise false.
 *
 */
bool get_chan_ctrl_power(chan_ctrl_format* chan_ctrl_f);

/**
 * @brief Sets the channel control register value.
 *
 * @param[in] chan_ctrl_f: Structure that contains the value that will be written in the register.
 *
 * @return bool: Returns true if the register can be set, otherwise false.
 *
 */
bool set_chan_ctrl_power(chan_ctrl_format* chan_ctrl_f);

/**
 * @brief Gets the user-specified short/long TX/RX SFD sequences register value.
 *
 * @param[in] usr_sfd_f: Structure in will be stored the value of the register.
 *
 * @return bool: Returns true if the register can be gotten, otherwise false.
 *
 */
bool get_usr_sfd(usr_sfd_format* usr_sfd_f);

/**
 * @brief Sets the user-specified short/long TX/RX SFD sequences register value.
 *
 * @param[in] usr_sfd_f: Structure that contains the value that will be written in the register.
 *
 * @return bool: Returns true if the register can be set, otherwise false.
 *
 */
bool set_usr_sfd(usr_sfd_format* usr_sfd_f);

/**
 * @brief Gets the user-specified short/long TX/RX SFD sequences register value.
 *
 * @param[in] usr_sfd_f: Structure in will be stored the value of the register.
 *
 * @return bool: Returns true if the register can be gotten, otherwise false.
 *
 */
bool get_agc_ctrl(agc_ctrl_format* agc_ctrl_f);

/**
 * @brief Sets the user-specified short/long TX/RX SFD sequences register value.
 *
 * @param[in] agc_ctrl_f: Structure that contains the value that will be written in the register.
 *
 * @return bool: Returns true if the register can be set, otherwise false.
 *
 */
bool set_agc_ctrl(agc_ctrl_format* agc_ctrl_f);

/**
 * @brief Gets the external clock synchronization counter configuration register value.
 *
 * @param[in] ext_sync_f: Structure in will be stored the value of the register.
 *
 * @return bool: Returns true if the register can be gotten, otherwise false.
 *
 */
bool get_ext_sync(ext_sync_format* ext_sync_f);

/**
 * @brief Sets the external clock synchronization counter configuration register value.
 *
 * @param[in] ext_sync_f: Structure that contains the value that will be written in the register.
 *
 * @return bool: Returns true if the register can be set, otherwise false.
 *
 */
bool set_ext_sync(ext_sync_format* ext_sync_f);

/**
 * @brief Gets one of the accumulator samples of the read access to accumulator data memory register value.
 *
 * @param[in] acc_mem_f: Structure in will be stored the value of the register.
 * @param[in] offset: Indicates which accumulator sample will be got.
 *
 * @return bool: Returns true if the register can be gotten, otherwise false.
 *
 */
bool get_acc_mem(acc_mem_field* acc_mem_f, const size_t offset);

/**
 * @brief Gets the GPIO mode sub-register value.
 *
 * @param[in] gpio_mode_ctrl_f: Structure in will be stored the value of the register.
 *
 * @return bool: Returns true if the register can be gotten, otherwise false.
 *
 */
bool get_gpio_mode_ctrl(gpio_mode_ctrl_format* gpio_mode_ctrl_f);

/**
 * @brief Sets the GPIO mode sub-register value.
 *
 * @param[in] gpio_mode_ctrl_f: Structure that contains the value that will be written in the register.
 *
 * @return bool: Returns true if the register can be set, otherwise false.
 *
 */
bool set_gpio_mode_ctrl(gpio_mode_ctrl_format* gpio_mode_ctrl_f);

/**
 * @brief Gets the GPIO mode sub-register value.
 *
 * @param[in] gpio_direction_ctrl_f: Structure in will be stored the value of the register.
 *
 * @return bool: Returns true if the register can be gotten, otherwise false.
 *
 */
bool get_gpio_direction_ctrl(gpio_direction_ctrl_format* gpio_direction_ctrl_f);

/**
 * @brief Sets the GPIO mode sub-register value.
 *
 * @param[in] gpio_direction_ctrl_f: Structure that contains the value that will be written in the register.
 *
 * @return bool: Returns true if the register can be set, otherwise false.
 *
 */
bool set_gpio_direction_ctrl(gpio_direction_ctrl_format* gpio_direction_ctrl_f);

/**
 * @brief Gets the GPIO data output control sub-register value.
 *
 * @param[in] gpio_data_output_ctrl_f: Structure in will be stored the value of the register.
 *
 * @return bool: Returns true if the register can be gotten, otherwise false.
 *
 */
bool get_gpio_data_output_ctrl(gpio_data_output_ctrl_format* gpio_data_output_ctrl_f);

/**
 * @brief Sets the GPIO data output control sub-register value.
 *
 * @param[in] gpio_data_output_ctrl_f: Structure that contains the value that will be written in the register.
 *
 * @return bool: Returns true if the register can be set, otherwise false.
 *
 */
bool set_gpio_data_output_ctrl(gpio_data_output_ctrl_format* gpio_data_output_ctrl_f);

/**
 * @brief Gets the GPIO interrupt enable sub-register value.
 *
 * @param[in] gpio_irq_ctrl_f: Structure in will be stored the value of the register.
 *
 * @return bool: Returns true if the register can be gotten, otherwise false.
 *
 */
bool get_gpio_irq_ctrl(gpio_irq_ctrl_format* gpio_irq_ctrl_f);

/**
 * @brief Sets the GPIO interrupt enable sub-register value.
 *
 * @param[in] gpio_irq_ctrl_f: Structure that contains the value that will be written in the register.
 *
 * @return bool: Returns true if the register can be set, otherwise false.
 *
 */
bool set_gpio_irq_ctrl(gpio_irq_ctrl_format* gpio_irq_ctrl_f);

/**
 * @brief Gets the GPIO Interrupt sense selection sub-register value.
 *
 * @param[in] gpio_irq_sense_ctrl_f: Structure in will be stored the value of the register.
 *
 * @return bool: Returns true if the register can be gotten, otherwise false.
 *
 */
bool get_gpio_irq_sense_ctrl(gpio_irq_sense_ctrl_format* gpio_irq_sense_ctrl_f);

/**
 * @brief Sets the GPIO Interrupt sense selection sub-register value.
 *
 * @param[in] gpio_irq_sense_ctrl_f: Structure that contains the value that will be written in the register.
 *
 * @return bool: Returns true if the register can be set, otherwise false.
 *
 */
bool set_gpio_irq_sense_ctrl(gpio_irq_sense_ctrl_format* gpio_irq_sense_ctrl_f);

/**
 * @brief Gets the GPIO Interrupt Mode (Level/Edge) sub-register value.
 *
 * @param[in] gpio_irq_mode_ctrl_f: Structure in will be stored the value of the register.
 *
 * @return bool: Returns true if the register can be gotten, otherwise false.
 *
 */
bool get_gpio_irq_mode_ctrl(gpio_irq_mode_ctrl_format* gpio_irq_mode_ctrl_f);

/**
 * @brief Sets the GPIO Interrupt Mode (Level/Edge) sub-register value.
 *
 * @param[in] gpio_irq_mode_ctrl_f: Structure that contains the value that will be written in the register.
 *
 * @return bool: Returns true if the register can be set, otherwise false.
 *
 */
bool set_gpio_irq_mode_ctrl(gpio_irq_mode_ctrl_format* gpio_irq_mode_ctrl_f);

/**
 * @brief Gets the GPIO Interrupt both edge select sub-register value.
 *
 * @param[in] gpio_irq_both_edges_mode_f: Structure in will be stored the value of the register.
 *
 * @return bool: Returns true if the register can be gotten, otherwise false.
 *
 */
bool get_gpio_irq_both_edges_mode_ctrl(gpio_irq_both_edges_mode_format* gpio_irq_both_edges_mode_f);

/**
 * @brief Sets the GPIO Interrupt both edge select sub-register value.
 *
 * @param[in] gpio_irq_both_edges_mode_f: Structure that contains the value that will be written in the register.
 *
 * @return bool: Returns true if the register can be set, otherwise false.
 *
 */
bool set_gpio_irq_both_edges_mode_ctrl(gpio_irq_both_edges_mode_format* gpio_irq_both_edges_mode_f);

/**
 * @brief Gets the GPIO interrupt latch clear sub-register value.
 *
 * @param[in] gpio_irq_latch_clear_mode_f: Structure in will be stored the value of the register.
 *
 * @return bool: Returns true if the register can be gotten, otherwise false.
 *
 */
bool get_gpio_irq_latch_clear_mode_ctrl(gpio_irq_latch_clear_mode_format* gpio_irq_latch_clear_mode_f);

/**
 * @brief Sets the GPIO interrupt latch clear sub-register value.
 *
 * @param[in] gpio_irq_latch_clear_mode_f: Structure that contains the value that will be written in the register.
 *
 * @return bool: Returns true if the register can be set, otherwise false.
 *
 */
bool set_gpio_irq_latch_clear_mode_ctrl(gpio_irq_latch_clear_mode_format* gpio_irq_latch_clear_mode_f);

/**
 * @brief Gets the GPIO interrupt de-bounce sub-register value.
 *
 * @param[in] gpio_irq_de_bounce_mode_f: Structure in will be stored the value of the register.
 *
 * @return bool: Returns true if the register can be gotten, otherwise false.
 *
 */
bool get_gpio_irq_de_bounce_mode_ctrl(gpio_irq_de_bounce_mode_format* gpio_irq_de_bounce_mode_f);

/**
 * @brief Sets the GPIO interrupt de-bounce sub-register value.
 *
 * @param[in] gpio_irq_de_bounce_mode_f: Structure that contains the value that will be written in the register.
 *
 * @return bool: Returns true if the register can be set, otherwise false.
 *
 */
bool set_gpio_irq_de_bounce_mode_ctrl(gpio_irq_de_bounce_mode_format* gpio_irq_de_bounce_mode_f);

/**
 * @brief Gets the GPIO raw state sub-register value.
 *
 * @param[in] gpio_raw_state_f: Structure in will be stored the value of the register.
 *
 * @return bool: Returns true if the register can be gotten, otherwise false.
 *
 */
bool get_gpio_raw_state_ctrl(gpio_raw_state_format* gpio_raw_state_f);

/**
 * @brief Sets the GPIO raw state sub-register value.
 *
 * @param[in] gpio_raw_state_f: Structure that contains the value that will be written in the register.
 *
 * @return bool: Returns true if the register can be set, otherwise false.
 *
 */
bool set_ggpio_raw_state_ctrl(gpio_raw_state_format* gpio_raw_state_f);

#endif // _REGISTER_H_
