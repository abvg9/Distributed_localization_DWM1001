/*
 * This file is part of the UCM-237 distribution (https://github.com/UCM-237/Distributed_localization_DWM1001).
 * Copyright (c) 2021 UCM, Madrid, Spain.
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

#include "format.h"

const double COUNTER_WRAP_PERIOD = 17.2074;
const double COUNTER_RESOLUTION = pow(2, 40);

void dev_id_formater(spi_frame fr, void *format) {

    dev_id_format *dev_id_f = (dev_id_format*) format;

    dev_id_f->ridtag = fr[2] | ((uint16_t) fr[3]) << 8;
    dev_id_f->model = fr[1];
    dev_id_f->ver = (fr[0] & 0XF0) >> 4;
    dev_id_f->rev = (fr[0] & 0XF);
}

void eui_formater(spi_frame fr, void *format) {

    eui_format *eui_f = (eui_format*) format;

    eui_f->mc_ID = fr[5] | (uint16_t) fr[6] << 8 | (uint32_t) fr[7] << 16;
    eui_f->ext_ID = fr[0] | (uint16_t) fr[1] << 8 | (uint32_t) fr[2] << 16
            | (uint32_t) fr[3] << 24 | (uint64_t) fr[4] << 32;
}

size_t eui_unformater(void *format, spi_frame fr) {

    eui_format *eui_f = ((eui_format*) format);

    fr[4] = (uint8_t) ((eui_f->ext_ID & 0xFF00000000) >> 32);
    fr[3] = (uint8_t) ((eui_f->ext_ID & 0x00FF000000) >> 24);
    fr[2] = (uint8_t) ((eui_f->ext_ID & 0x0000FF0000) >> 16);
    fr[1] = (uint8_t) ((eui_f->ext_ID & 0x000000FF00) >> 8);
    fr[0] = (uint8_t) (eui_f->ext_ID & 0x00000000FF);

    fr[7] = (uint8_t) ((eui_f->mc_ID & 0xFF0000) >> 16);
    fr[6] = (uint8_t) ((eui_f->mc_ID & 0x00FF00) >> 8);
    fr[5] = (uint8_t) (eui_f->mc_ID & 0x0000FF);

    return 8;
}

void pan_addr_formater(spi_frame fr, void *format) {

    pan_adr_format *pan_adr_f = ((pan_adr_format*) format);

    pan_adr_f->pan_id = fr[2] | ((uint16_t) fr[3]) << 8;
    pan_adr_f->short_addr = fr[0] | ((uint16_t) fr[1]) << 8;
}

size_t pan_addr_unformater(void *format, spi_frame fr) {

    uint16_t *subregister = ((uint16_t*) format);

    fr[1] = (*subregister & 0xFF00) >> 8;
    fr[0] = *subregister & 0xFF;

    return 2;
}

void sys_cfg_formater(spi_frame fr, void *format) {

    sys_cfg_format *sys_cfg_f = ((sys_cfg_format*) format);

    sys_cfg_f->aackpend = (fr[3] & 0b10000000) >> 7;
    sys_cfg_f->autoack = (fr[3] & 0b01000000) >> 6;
    sys_cfg_f->rxautr = (fr[3] & 0b00100000) >> 5;
    sys_cfg_f->rxwtoe = (fr[3] & 0b00010000) >> 4;

    sys_cfg_f->rxm110k = (fr[2] & 0b01000000) >> 6;
    sys_cfg_f->dis_stxp = (fr[2] & 0b00000100) >> 3;
    sys_cfg_f->phr_mode = fr[2] & 0b00000011;

    sys_cfg_f->fcs_init2f = (fr[1] & 0b10000000) >> 7;
    sys_cfg_f->dis_rsde = (fr[1] & 0b01000000) >> 6;
    sys_cfg_f->dis_phe = (fr[1] & 0b00100000) >> 5;
    sys_cfg_f->dis_drxb = (fr[1] & 0b00010000) >> 4;
    sys_cfg_f->dis_fce = (fr[1] & 0b00001000) >> 3;
    sys_cfg_f->spi_edge = (fr[1] & 0b00000100) >> 2;
    sys_cfg_f->hirq_pol = (fr[1] & 0b00000010) >> 1;
    sys_cfg_f->ffa5 = fr[1] & 0b00000001;

    sys_cfg_f->ffa4 = (fr[0] & 0b10000000) >> 7;
    sys_cfg_f->ffar = (fr[0] & 0b01000000) >> 6;
    sys_cfg_f->ffam = (fr[0] & 0b00100000) >> 5;
    sys_cfg_f->ffaa = (fr[0] & 0b00010000) >> 4;
    sys_cfg_f->ffad = (fr[0] & 0b00001000) >> 3;
    sys_cfg_f->ffab = (fr[0] & 0b00000100) >> 2;
    sys_cfg_f->ffbc = (fr[0] & 0b00000010) >> 1;
    sys_cfg_f->ffen = fr[0] & 0b00000001;

}

size_t sys_cfg_unformater(void *format, spi_frame fr) {

    sys_cfg_format *sys_cfg_f = ((sys_cfg_format*) format);

    fr[3] = sys_cfg_f->aackpend << 7;
    fr[3] |= sys_cfg_f->autoack << 6;
    fr[3] |= sys_cfg_f->rxautr << 6;
    fr[3] |= sys_cfg_f->rxwtoe << 6;

    fr[2] = sys_cfg_f->rxm110k << 6;
    fr[2] |= sys_cfg_f->dis_stxp << 3;
    fr[2] |= sys_cfg_f->phr_mode;

    fr[1] = sys_cfg_f->fcs_init2f << 7;
    fr[1] |= sys_cfg_f->dis_rsde << 6;
    fr[1] |= sys_cfg_f->dis_phe << 5;
    fr[1] |= sys_cfg_f->dis_drxb << 4;
    fr[1] |= sys_cfg_f->dis_fce << 3;
    fr[1] |= sys_cfg_f->spi_edge << 2;
    fr[1] |= sys_cfg_f->hirq_pol << 1;
    fr[1] |= sys_cfg_f->ffa5;

    fr[0] = sys_cfg_f->ffa4 << 7;
    fr[0] |= sys_cfg_f->ffar << 6;
    fr[0] |= sys_cfg_f->ffam << 5;
    fr[0] |= sys_cfg_f->ffaa << 4;
    fr[0] |= sys_cfg_f->ffad << 3;
    fr[0] |= sys_cfg_f->ffab << 2;
    fr[0] |= sys_cfg_f->ffbc << 1;
    fr[0] |= sys_cfg_f->ffen;

    return 4;
}

void sys_time_formater(spi_frame fr, void* format) {

    uint64_t register_value  = (uint16_t) fr[1] << 8 | (uint32_t) fr[2] << 16
            | (uint32_t) fr[3] << 24 | (uint64_t) fr[4] << 32;

    double* seconds = ((double*) format);
    *seconds = calculate_seconds(register_value);
}

void tx_fctrl_formater(spi_frame fr, void* format) {

    tx_fctrl_format *tx_fctrl_f = ((tx_fctrl_format*) format);

    /* TODO */
    /*
    tx_fctrl_f->txboffs = ((fr[3] & 0b11000000) >> 6) | ((uint16_t) fr[4]) << 8;
    tx_fctrl_f->pe_txpsr = fr[3] & 0b00111100;
    tx_fctrl_f->tr = fr[2] & 0b100000000;
    tx_fctrl_f->txbr = fr[2] & 0b011000000;
    tx_fctrl_f->tfle = (fr[1] & 0b10000000) | (fr[2] & 0b00000011) << 6;
    tx_fctrl_f->tflen = fr[1] & 0b01111111;
    tx_fctrl_f->ifsdelay = fr[0];
    */

}

size_t tx_fctrl_unformater(void* format, spi_frame fr) {

    tx_fctrl_format *tx_fctrl_f = ((tx_fctrl_format*) format);

    /* TODO */
    /*
    fr[4] = tx_fctrl_f->txboffs >> 2;
    fr[3] = ((tx_fctrl_f->txboffs & 0b00000011) << 6) | tx_fctrl_f->pe_txpsr | ;
    fr[2] = tx_fctrl_f->txboffs;
    fr[1] = tx_fctrl_f->txboffs;
    fr[0] = tx_fctrl_f->txboffs;
    */

    return 5;
}
