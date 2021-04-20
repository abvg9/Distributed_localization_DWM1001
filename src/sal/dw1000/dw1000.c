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

#include "dw1000.h"

static const SPIConfig spi_cfg = { .end_cb = NULL, .ssport = IOPORT1, .sspad = SPI_SS,
        .freq = NRF5_SPI_FREQ_2MBPS, .sckpad = SPI_SCK, .mosipad = SPI_MOSI,
        .misopad = SPI_MISO, .lsbfirst = false, .mode = 2};

void dw_disable(void) {
    spiStop(&SPID1);
    DW_POWER_OFF;
}

bool dw_eneable(void) {

    dw_reset();

    spiStart(&SPID1, &spi_cfg);

    dev_id_format dev_id_f;
    bool ret = get_dev_id(&dev_id_f);

    if (ret) {
        ret &= DEFINED_DEV_ID.ridtag == dev_id_f.ridtag;
        ret &= DEFINED_DEV_ID.model == dev_id_f.model;
        ret &= DEFINED_DEV_ID.ver == dev_id_f.ver;
        ret &= DEFINED_DEV_ID.rev == dev_id_f.rev;
    }

    return ret;
}

void dw_reset(void) {
    DW_POWER_OFF;
    chThdSleepMicroseconds(10);
    DW_POWER_ON;
}

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

    const uint16_t offset = TX_BUFFER_MAX_SIZE - frame_size;
    bool ret = false;

    if(offset + frame_size <= TX_BUFFER_MAX_SIZE) {
        ret = dw_write_reg(TX_BUFFER, offset, (void*) frame);
    }

    return ret;

}

bool get_dx_time(double* seconds) {
    return dw_read_reg(DX_TIME, 0, (void*) seconds);
}

bool set_dx_time(double* seconds) {
    return (*seconds <= DTR_COUNTER_WRAP_PERIOD) &&
            dw_write_reg(DX_TIME, 0, (void*) seconds);
}

bool get_rx_fwto(double* seconds) {
    return dw_read_reg(RX_FWTO, 0, (void*) seconds);
}

bool set_rx_fwto(double* seconds) {
    return (*seconds <= RFR_COUNTER_WRAP_PERIOD) &&
            dw_write_reg(RX_FWTO, 0, (void*) seconds);
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
    return dw_read_reg(SYS_STATUS, OCT_0_TO_3, (void*) sys_evt_sts_f) &&
           dw_read_reg(SYS_STATUS, OCT_4, (void*) sys_evt_sts_f);
}

bool set_sys_event_sts(sys_evt_sts_format* sys_evt_sts_f) {
    return dw_write_reg(SYS_STATUS, OCT_0_TO_3, (void*) sys_evt_sts_f) &&
           dw_write_reg(SYS_STATUS, OCT_4, (void*) sys_evt_sts_f);
}
