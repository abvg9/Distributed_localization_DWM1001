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

#include "format.h"

void dev_id_formater(spi_frame fr, void *format, const size_t sub_register) {

    dev_id_format *dev_id_f = (dev_id_format*) format;

    dev_id_f->ridtag = fr[2] | ((uint16_t) fr[3]) << 8;
    dev_id_f->model = fr[1];
    dev_id_f->ver = (fr[0] & 0XF0) >> 4;
    dev_id_f->rev = (fr[0] & 0XF);
}

void eui_formater(spi_frame fr, void *format, const size_t sub_register) {

    eui_format *eui_f = (eui_format*) format;

    eui_f->mc_ID = fr[5] | (uint16_t) fr[6] << 8 | (uint32_t) fr[7] << 16;
    eui_f->ext_ID = fr[0] | (uint16_t) fr[1] << 8 | (uint32_t) fr[2] << 16
            | (uint32_t) fr[3] << 24 | (uint64_t) fr[4] << 32;
}

size_t eui_unformater(void *format, spi_frame fr, const size_t sub_register) {

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

void pan_addr_formater(spi_frame fr, void *format, const size_t sub_register) {

    pan_adr_format *pan_adr_f = ((pan_adr_format*) format);

    pan_adr_f->pan_id = fr[2] | ((uint16_t) fr[3]) << 8;
    pan_adr_f->short_addr = fr[0] | ((uint16_t) fr[1]) << 8;
}

size_t pan_addr_unformater(void *format, spi_frame fr, const size_t sub_register) {

    pan_adr_format *pan_adr_f = ((pan_adr_format*) format);

    if(sub_register == PAN_ID) {
        fr[1] = (pan_adr_f->pan_id & 0xFF00) >> 8;
        fr[0] = pan_adr_f->pan_id & 0xFF;
    } else if(sub_register == SHORT_ADR) {
        fr[1] = (pan_adr_f->short_addr & 0xFF00) >> 8;
        fr[0] = pan_adr_f->short_addr & 0xFF;
    } else {
        return 0;
    }

    return 2;
}

void sys_cfg_formater(spi_frame fr, void *format, const size_t sub_register) {

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

size_t sys_cfg_unformater(void *format, spi_frame fr, const size_t sub_register) {

    sys_cfg_format *sys_cfg_f = ((sys_cfg_format*) format);

    fr[3] = sys_cfg_f->aackpend << 7;
    fr[3] |= sys_cfg_f->autoack << 6;
    fr[3] |= sys_cfg_f->rxautr << 5;
    fr[3] |= sys_cfg_f->rxwtoe << 4;

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

void sys_time_formater(spi_frame fr, void* format, const size_t sub_register) {

    uint64_t register_value  = (uint16_t) fr[1] << 8 | (uint32_t) fr[2] << 16
            | (uint32_t) fr[3] << 24 | (uint64_t) fr[4] << 32;

    double* seconds = ((double*) format);
    *seconds = str_calculate_seconds(register_value);
}

void tx_fctrl_formater(spi_frame fr, void* format, const size_t sub_register) {

    tx_fctrl_format *tx_fctrl_f = ((tx_fctrl_format*) format);

    if(sub_register == REST) {
        tx_fctrl_f->txboffs = (((uint16_t) fr[3]) << 2) | ((fr[2] & 0b11000000) >> 6);
        tx_fctrl_f->pe_txpsr = (fr[2] & 0b00111100) >> 2;
        tx_fctrl_f->txprf = fr[2] & 0b00000011;

        tx_fctrl_f->tr = (fr[1] & 0b10000000) >> 7;
        tx_fctrl_f->txbr = (fr[1] & 0b011000000) >> 5;

        tx_fctrl_f->tfle = ((fr[0] & 0b10000000) >> 7) | (fr[1] & 0b00000011) << 1;
        tx_fctrl_f->tflen = fr[0] & 0b01111111;

    } else if(sub_register == IFSDELAY) {

        tx_fctrl_f->ifsdelay = fr[0];

    }

}

size_t tx_fctrl_unformater(void* format, spi_frame fr, const size_t sub_register) {

    tx_fctrl_format *tx_fctrl_f = ((tx_fctrl_format*) format);

    if(sub_register == REST) {
        fr[3] = tx_fctrl_f->txboffs >> 2;

        fr[2] = ((tx_fctrl_f->txboffs & 0b0000000011) << 6) | tx_fctrl_f->pe_txpsr << 2 | tx_fctrl_f->txprf;

        fr[1] = tx_fctrl_f->tr << 7 | tx_fctrl_f->txbr << 5 | ((tx_fctrl_f->tfle & 0b110) >> 1);

        fr[0] = ((tx_fctrl_f->tfle & 0b001) << 7) | tx_fctrl_f->tflen;

        return 4;
    } else if(sub_register == IFSDELAY) {

        fr[0] = tx_fctrl_f->ifsdelay;

        return 1;
    }

    return 0;
}

size_t tx_buffer_unformater(void *format, spi_frame fr, const size_t sub_register) {

    const size_t buffer_size = TX_BUFFER_MAX_SIZE - sub_register;

    uint8_t* buffer = (uint8_t*) format;

    int i;
    for( i = buffer_size-1; i >= 0; --i) {
        fr[i] = buffer[i];
    }


    return buffer_size;
}

void dx_time_formater(spi_frame fr, void *format, const size_t sub_register) {

    uint64_t register_value  = (uint16_t) fr[1] << 8 | (uint32_t) fr[2] << 16
            | (uint32_t) fr[3] << 24 | (uint64_t) fr[4] << 32;

    double* seconds = ((double*) format);
    *seconds = dtr_calculate_seconds(register_value);

}

size_t dx_time_unformater(void *format, spi_frame fr, const size_t sub_register) {

    double* seconds = ((double*) format);
    uint64_t reg_value = dtr_calculate_register_val(*seconds);

    fr[4] = (reg_value & 0xFF00000000) >> 32;
    fr[3] = (reg_value & 0x00FF000000) >> 24;
    fr[2] = (reg_value & 0x0000FF0000) >> 16;
    fr[1] = (reg_value & 0x000000FF00) >> 8;
    fr[0] = reg_value & 0x00000000FF;

    return 5;
}

void rx_fwto_formater(spi_frame fr, void *format, const size_t sub_register) {

    uint16_t register_value  = fr[0] | (uint16_t) fr[1] << 8;

    double* seconds = ((double*) format);
    *seconds = rfr_calculate_seconds(register_value);

}

size_t rx_fwto_unformater(void *format, spi_frame fr, const size_t sub_register) {

    double* seconds = ((double*) format);
    uint16_t register_value = rfr_calculate_register_val(*seconds);

    fr[1] = (register_value & 0xFF00) >> 8;
    fr[0] = register_value & 0x00FF;

    return 2;
}

void tx_ctrl_formater(spi_frame fr, void *format, const size_t sub_register) {

    sys_ctrl_format *sys_ctrl_f = ((sys_ctrl_format*) format);

    sys_ctrl_f->hrbpt = fr[3] & 0b00000001;

    sys_ctrl_f->rxdlye = (fr[1] & 0b00000010) >> 1;
    sys_ctrl_f->rxenab = fr[1] & 0b00000001;

    sys_ctrl_f->wait4resp = (fr[0] & 0b10000000) >> 7;
    sys_ctrl_f->trxoff = (fr[0] & 0b01000000) >> 6;
    sys_ctrl_f->cansfcs = (fr[0] & 0b00001000) >> 3;
    sys_ctrl_f->txdlys = (fr[0] & 0b00000100) >> 2;
    sys_ctrl_f->txstrt = (fr[0] & 0b00000010) >> 1;
    sys_ctrl_f->sfcst = fr[0] & 0b00000001;

}

size_t tx_ctrl_unformater(void *format, spi_frame fr, const size_t sub_register) {

    sys_ctrl_format *sys_ctrl_f = ((sys_ctrl_format*) format);

    fr[3] = sys_ctrl_f->hrbpt;

    fr[2] = 0;

    fr[1] = sys_ctrl_f->rxdlye << 1;
    fr[1] |= sys_ctrl_f->rxenab;

    fr[0] = sys_ctrl_f->wait4resp << 7;
    fr[0] |= sys_ctrl_f->trxoff << 6;
    fr[0] |= sys_ctrl_f->cansfcs << 3;
    fr[0] |= sys_ctrl_f->txdlys << 2;
    fr[0] |= sys_ctrl_f->txstrt << 1;
    fr[0] |= sys_ctrl_f->sfcst;

    return 4;
}

void sys_evt_msk_formater(spi_frame fr, void *format, const size_t sub_register) {

    sys_evt_msk_format *sys_evt_msk_f = ((sys_evt_msk_format*) format);

    sys_evt_msk_f->maffrej = (fr[3] & 0b00100000) >> 5;
    sys_evt_msk_f->mtxberr = (fr[3] & 0b00010000) >> 4;
    sys_evt_msk_f->mhpdwarn = (fr[3] & 0b00001000) >> 3;
    sys_evt_msk_f->mrxsfdto = (fr[3] & 0b00000100) >> 2;
    sys_evt_msk_f->mcpllll = (fr[3] & 0b00000010) >> 1;
    sys_evt_msk_f->mrfpllll = fr[3] & 0b00000001;

    sys_evt_msk_f->mslp2init = (fr[2] & 0b10000000) >> 7;
    sys_evt_msk_f->mgpioirq = (fr[2] & 0b01000000) >> 6;
    sys_evt_msk_f->mrxpto = (fr[2] & 0b00100000) >> 5;
    sys_evt_msk_f->mrxovrr = (fr[2] & 0b00010000) >> 4;
    sys_evt_msk_f->mldeerr = (fr[2] & 0b00000100) >> 2;
    sys_evt_msk_f->mrxrfto = (fr[2] & 0b00000010) >> 1;
    sys_evt_msk_f->mrxrfsl = fr[2] & 0b00000001;

    sys_evt_msk_f->mrxfce = (fr[1] & 0b10000000) >> 7;
    sys_evt_msk_f->mrxfcg = (fr[1] & 0b01000000) >> 6;
    sys_evt_msk_f->mrxdfr = (fr[1] & 0b00100000) >> 5;
    sys_evt_msk_f->mrxphe = (fr[1] & 0b00010000) >> 4;
    sys_evt_msk_f->mrxphd = (fr[1] & 0b00001000) >> 3;
    sys_evt_msk_f->mldedone = (fr[1] & 0b00000100) >> 2;
    sys_evt_msk_f->mrxsfdd = (fr[1] & 0b00000010) >> 1;
    sys_evt_msk_f->mrxprd = fr[1] & 0b00000001;

    sys_evt_msk_f->mrxfce = (fr[1] & 0b10000000) >> 7;
    sys_evt_msk_f->mrxfcg = (fr[1] & 0b01000000) >> 6;
    sys_evt_msk_f->mrxdfr = (fr[1] & 0b00100000) >> 5;
    sys_evt_msk_f->mrxphe = (fr[1] & 0b00010000) >> 4;
    sys_evt_msk_f->mrxphd = (fr[1] & 0b00001000) >> 3;
    sys_evt_msk_f->mldedone = (fr[1] & 0b00000100) >> 2;
    sys_evt_msk_f->mrxsfdd = (fr[1] & 0b00000010) >> 1;
    sys_evt_msk_f->mrxprd = fr[1] & 0b00000001;

    sys_evt_msk_f->mtxfrs = (fr[0] & 0b10000000) >> 7;
    sys_evt_msk_f->mtxphs = (fr[0] & 0b01000000) >> 6;
    sys_evt_msk_f->mtxprs = (fr[0] & 0b00100000) >> 5;
    sys_evt_msk_f->mtxfrb = (fr[0] & 0b00010000) >> 4;
    sys_evt_msk_f->maat = (fr[0] & 0b00001000) >> 3;
    sys_evt_msk_f->mesyncr = (fr[0] & 0b00000100) >> 2;
    sys_evt_msk_f->mcplock = (fr[0] & 0b00000010) >> 1;

}

size_t sys_evt_msk_unformater(void *format, spi_frame fr, const size_t sub_register) {

    sys_evt_msk_format *sys_evt_msk_f = ((sys_evt_msk_format*) format);

    fr[3] = sys_evt_msk_f->maffrej << 5;
    fr[3] |= sys_evt_msk_f->mtxberr << 4;
    fr[3] |= sys_evt_msk_f->mhpdwarn << 3;
    fr[3] |= sys_evt_msk_f->mrxsfdto << 2;
    fr[3] |= sys_evt_msk_f->mcpllll << 1;
    fr[3] |= sys_evt_msk_f->mrfpllll;

    fr[2] = sys_evt_msk_f->mslp2init << 7;
    fr[2] |= sys_evt_msk_f->mgpioirq << 6;
    fr[2] |= sys_evt_msk_f->mrxpto << 5;
    fr[2] |= sys_evt_msk_f->mrxovrr << 4;
    fr[2] |= sys_evt_msk_f->mldeerr << 2;
    fr[2] |= sys_evt_msk_f->mrxrfto << 1;
    fr[2] |= sys_evt_msk_f->mrxrfsl;

    fr[1] = sys_evt_msk_f->mrxfce << 7;
    fr[1] |= sys_evt_msk_f->mrxfcg << 6;
    fr[1] |= sys_evt_msk_f->mrxdfr << 5;
    fr[1] |= sys_evt_msk_f->mrxphe << 4;
    fr[1] |= sys_evt_msk_f->mrxphd << 3;
    fr[1] |= sys_evt_msk_f->mldedone << 2;
    fr[1] |= sys_evt_msk_f->mrxsfdd << 1;
    fr[1] |= sys_evt_msk_f->mrxprd;

    fr[0] = sys_evt_msk_f->mtxfrs << 7;
    fr[0] |= sys_evt_msk_f->mtxphs << 6;
    fr[0] |= sys_evt_msk_f->mtxprs << 5;
    fr[0] |= sys_evt_msk_f->mtxfrb << 4;
    fr[0] |= sys_evt_msk_f->maat << 3;
    fr[0] |= sys_evt_msk_f->mesyncr << 2;
    fr[0] |= sys_evt_msk_f->mcplock << 1;

    return 4;
}

void sys_evt_sts_formater(spi_frame fr, void *format, const size_t sub_register) {

    sys_evt_sts_format *sys_evt_sts_f = ((sys_evt_sts_format*) format);

     if(sub_register == OCT_0_TO_3) {

         sys_evt_sts_f->icrbp = (fr[3] & 0b10000000) >> 7;
         sys_evt_sts_f->hsrbp = (fr[3] & 0b01000000) >> 6;
         sys_evt_sts_f->affrej = (fr[3] & 0b00100000) >> 5;
         sys_evt_sts_f->txberr = (fr[3] & 0b00010000) >> 4;
         sys_evt_sts_f->hpdwarn = (fr[3] & 0b00001000) >> 3;
         sys_evt_sts_f->rxsfdto = (fr[3] & 0b00000100) >> 2;
         sys_evt_sts_f->clkpll_ll = (fr[3] & 0b00000010) >> 1;
         sys_evt_sts_f->rfpll_ll = fr[3] & 0b00000001;

         sys_evt_sts_f->slp2init = (fr[2] & 0b10000000) >> 7;
         sys_evt_sts_f->gpioirq = (fr[2] & 0b01000000) >> 6;
         sys_evt_sts_f->rxpto = (fr[2] & 0b00100000) >> 5;
         sys_evt_sts_f->rxovrr = (fr[2] & 0b00010000) >> 4;
         sys_evt_sts_f->ldeerr = (fr[2] & 0b00000100) >> 2;
         sys_evt_sts_f->rxrfto = (fr[2] & 0b00000010) >> 1;
         sys_evt_sts_f->rxrfsl = fr[2] & 0b00000001;

         sys_evt_sts_f->rxfce = (fr[1] & 0b10000000) >> 7;
         sys_evt_sts_f->rxfcg = (fr[1] & 0b01000000) >> 6;
         sys_evt_sts_f->rxdfr = (fr[1] & 0b00100000) >> 5;
         sys_evt_sts_f->rxphe = (fr[1] & 0b00010000) >> 4;
         sys_evt_sts_f->rxphd = (fr[1] & 0b00001000) >> 3;
         sys_evt_sts_f->ldedone = (fr[1] & 0b00000100) >> 2;
         sys_evt_sts_f->rxsfdd = (fr[1] & 0b00000010) >> 1;
         sys_evt_sts_f->rxprd = fr[1] & 0b00000001;

         sys_evt_sts_f->txfrs = (fr[0] & 0b10000000) >> 7;
         sys_evt_sts_f->txphs = (fr[0] & 0b01000000) >> 6;
         sys_evt_sts_f->txprs = (fr[0] & 0b00100000) >> 5;
         sys_evt_sts_f->txfrb = (fr[0] & 0b00010000) >> 4;
         sys_evt_sts_f->aat = (fr[0] & 0b00001000) >> 3;
         sys_evt_sts_f->esyncr = (fr[0] & 0b00000100) >> 2;
         sys_evt_sts_f->cplock = (fr[0] & 0b00000010) >> 1;
         sys_evt_sts_f->irqs = fr[0] & 0b00000001;

     } else if(sub_register == OCT_4) {
         sys_evt_sts_f->txpute = (fr[0] & 0b00000100) >> 2;
         sys_evt_sts_f->rxprej = (fr[0] & 0b00000010) >> 1;
         sys_evt_sts_f->rxrscs = fr[0] & 0b00000001;
     }

}

size_t sys_evt_sts_unformater(void *format, spi_frame fr, const size_t sub_register) {

    sys_evt_sts_format *sys_evt_sts_f = ((sys_evt_sts_format*) format);

     if(sub_register == OCT_0_TO_3) {

         fr[3] = sys_evt_sts_f->icrbp << 7;
         fr[3] |= sys_evt_sts_f->hsrbp << 6;
         fr[3] |= sys_evt_sts_f->affrej << 5;
         fr[3] |= sys_evt_sts_f->txberr << 4;
         fr[3] |= sys_evt_sts_f->hpdwarn << 3;
         fr[3] |= sys_evt_sts_f->rxsfdto << 2;
         fr[3] |= sys_evt_sts_f->clkpll_ll << 1;
         fr[3] |= sys_evt_sts_f->rfpll_ll;

         fr[2] = sys_evt_sts_f->slp2init << 7;
         fr[2] |= sys_evt_sts_f->gpioirq << 6;
         fr[2] |= sys_evt_sts_f->rxpto << 5;
         fr[2] |= sys_evt_sts_f->rxovrr << 4;
         fr[2] |= sys_evt_sts_f->ldeerr << 2;
         fr[2] |= sys_evt_sts_f->rxrfto << 1;
         fr[2] |= sys_evt_sts_f->rxrfsl;

         fr[1] = sys_evt_sts_f->rxfce << 7;
         fr[1] |= sys_evt_sts_f->rxfcg << 6;
         fr[1] |= sys_evt_sts_f->rxdfr << 5;
         fr[1] |= sys_evt_sts_f->rxphe << 4;
         fr[1] |= sys_evt_sts_f->rxphd << 3;
         fr[1] |= sys_evt_sts_f->ldedone << 2;
         fr[1] |= sys_evt_sts_f->rxsfdd << 1;
         fr[1] |= sys_evt_sts_f->rxprd;

         fr[0] = sys_evt_sts_f->txfrs << 7;
         fr[0] |= sys_evt_sts_f->txphs << 6;
         fr[0] |= sys_evt_sts_f->txprs << 5;
         fr[0] |= sys_evt_sts_f->txfrb << 4;
         fr[0] |= sys_evt_sts_f->aat << 3;
         fr[0] |= sys_evt_sts_f->esyncr << 2;
         fr[0] |= sys_evt_sts_f->cplock << 1;
         fr[0] |= sys_evt_sts_f->irqs;

         return 4;

     } else if(sub_register == OCT_4) {

         fr[0] = sys_evt_sts_f->txpute << 2;
         fr[0] |= sys_evt_sts_f->rxprej << 1;
         fr[0] |= sys_evt_sts_f->rxrscs;

         return 1;
     }

     return 0;
}
