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

    const size_t buffer_size = TX_RX_BUFFER_MAX_SIZE - sub_register;

    uint8_t* buffer = (uint8_t*) format;

    int i;
    for( i = buffer_size-1; i >= 0; --i) {
        fr[i] = buffer[i];
    }


    return buffer_size;
}

void dx_time_formater(spi_frame fr, void *format, const size_t sub_register) {

    const uint64_t register_value  = (uint16_t) fr[1] << 8 | (uint32_t) fr[2] << 16
            | (uint32_t) fr[3] << 24 | (uint64_t) fr[4] << 32;

    double* seconds = ((double*) format);
    *seconds = dtr_calculate_seconds(register_value);

}

size_t dx_time_unformater(void *format, spi_frame fr, const size_t sub_register) {

    double* seconds = ((double*) format);
    const uint64_t reg_value = dtr_calculate_register_val(*seconds);

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

     if(sub_register == SES_OCT_0_TO_3) {

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

     } else if(sub_register == SES_OCT_4) {
         sys_evt_sts_f->txpute = (fr[0] & 0b00000100) >> 2;
         sys_evt_sts_f->rxprej = (fr[0] & 0b00000010) >> 1;
         sys_evt_sts_f->rxrscs = fr[0] & 0b00000001;
     }

}

size_t sys_evt_sts_unformater(void *format, spi_frame fr, const size_t sub_register) {

    sys_evt_sts_format *sys_evt_sts_f = ((sys_evt_sts_format*) format);

     if(sub_register == SES_OCT_0_TO_3) {

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

     } else if(sub_register == SES_OCT_4) {

         fr[0] = sys_evt_sts_f->txpute << 2;
         fr[0] |= sys_evt_sts_f->rxprej << 1;
         fr[0] |= sys_evt_sts_f->rxrscs;

         return 1;
     }

     return 0;
}

void rx_finfo_formater(spi_frame fr, void *format, const size_t sub_register) {

    rx_finfo_format *rx_finfo_f = ((rx_finfo_format*) format);

    rx_finfo_f->rxpacc = ((uint16_t)fr[3]) << 4 | ((fr[2] & 0b11110000) >> 4);
    rx_finfo_f->rxnspl_rxpsr =  ((fr[1] & 0b00011000) >> 1) | ((fr[2] & 0b00001100) >> 2);
    rx_finfo_f->rxprfr = fr[2] & 0b00000011;
    rx_finfo_f->rng = (fr[1] & 0b10000000) >> 7;
    rx_finfo_f->rxbr = (fr[1] & 0b01100000) >> 5;
    rx_finfo_f->rxfle = ((fr[1] && 0b00000011) << 1)  | ((fr[0] & 0b10000000) >> 7);
    rx_finfo_f->rxflen = fr[0] & 0b01111111;

}

void rx_buffer_formater(spi_frame fr, void *format, const size_t sub_register) {

    const size_t buffer_size = TX_RX_BUFFER_MAX_SIZE - sub_register;

    uint8_t* buffer = (uint8_t*) format;

    int i;
    for( i = buffer_size-1; i >= 0; --i) {
        buffer[i] = fr[i];
    }

}

void rx_fqual_formater(spi_frame fr, void *format, const size_t sub_register) {

    rx_fqual_format *rx_fqual_f = ((rx_fqual_format*) format);

    if(sub_register == FP_AMPL2_STD_NOISE) {

        rx_fqual_f->fp_ampl2 = ((uint16_t)fr[3] << 8) | fr[2];
        rx_fqual_f->std_noise = ((uint16_t)fr[1] << 8) | fr[0];

    } else if(sub_register == CIR_PWR_PP_AMPL3) {

        rx_fqual_f->cir_pwr = ((uint16_t)fr[3] << 8) | fr[2];
        rx_fqual_f->fp_ampl3 = ((uint16_t)fr[1] << 8) | fr[0];
    }
}

void rx_ttcki_formater(spi_frame fr, void *format, const size_t sub_register) {

    rx_ttcki_value* rx_ttcki = (rx_ttcki_value*) format;

    *rx_ttcki = (((uint32_t)fr[3]) << 24) | (((uint32_t)fr[2]) << 16) | (((uint32_t)fr[1]) << 8) | fr[0];

}

void rx_ttcko_formater(spi_frame fr, void *format, const size_t sub_register) {

    rx_ttcko_format* rx_ttcko_f = (rx_ttcko_format*) format;

    if(sub_register == RX_TTCKO_OCT_0_TO_3) {
        rx_ttcko_f->rsmpdel = fr[3];
        rx_ttcko_f->rxtofs = (((uint32_t)(fr[2] & 0b00000111)) << 16)  | (((uint16_t) fr[1]) << 8) | fr[0];
    } else if(sub_register == RX_TTCKO_OCT_4) {
        rx_ttcko_f->rcphase = calculate_phase( (fr[0] & 0b01111111) );
    }
}

void rx_time_formater(spi_frame fr, void *format, const size_t sub_register) {

    rx_time_format* rx_time_f = (rx_time_format*) format;

    if(sub_register == RX_TIME_OCT_0_TO_3) {

        const uint32_t rx_stamp_low_bits = (((uint32_t)fr[3]) << 24) | (((uint32_t)fr[2]) << 16) |
                (((uint16_t)fr[1]) << 8) | fr[0];
        rx_time_f->rx_stamp = calculate_stamp(rx_stamp_low_bits);

    } else if(sub_register == RX_TIME_OCT_4_TO_7) {

        rx_time_f->fp_ampl1 = fr[3];

        rx_time_f->fp_index = (((uint16_t)fr[2]) << 8) | fr[1];

        const uint64_t rx_stamp_high_bits = ((uint64_t)fr[0] << 32);
        rx_time_f->rx_stamp += calculate_stamp(rx_stamp_high_bits);

    } else if(sub_register == RX_TIME_OCT_8_TO_11) {

        const uint32_t rx_rawst_low_bits = (((uint32_t)fr[3]) << 16) | (((uint16_t)fr[2]) << 8) | fr[1];
        rx_time_f->rx_rawst = str_calculate_seconds(rx_rawst_low_bits);

        rx_time_f->fp_ampl1 |= ((uint16_t)fr[0]) << 8;

    } else if(sub_register == RX_TIME_OCT_12_TO_13) {

        const uint64_t rx_rawst_high_bits = ((uint64_t)fr[1] << 32) | ((uint32_t)fr[0] << 24);
        rx_time_f->rx_rawst += str_calculate_seconds(rx_rawst_high_bits);

    }

}

void tx_time_formater(spi_frame fr, void *format, const size_t sub_register) {

    tx_time_format* tx_time_f = (tx_time_format*) format;

    if(sub_register == TX_TIME_OCT_0_TO_3) {

        const uint32_t tx_stamp_low_bits = (((uint32_t)fr[3]) << 24) | (((uint32_t)fr[2]) << 16) |
                (((uint16_t)fr[1]) << 8) | fr[0];
        tx_time_f->tx_stamp = calculate_stamp(tx_stamp_low_bits);

    } else if(sub_register == TX_TIME_OCT_4_TO_7) {

        const uint32_t tx_rawst_low_bits = (((uint32_t)fr[3]) << 16) | (((uint16_t)fr[2]) << 8) | fr[1];
        tx_time_f->tx_rawst =  str_calculate_seconds(tx_rawst_low_bits);

        const uint64_t tx_stamp_high_bits = ((uint64_t)fr[0]) << 32;
        tx_time_f->tx_stamp += calculate_stamp(tx_stamp_high_bits);

    } else if(sub_register == TX_TIME_OCT_8_TO_9) {

        const uint64_t tx_rawst_high_bits = (((uint64_t)fr[1]) << 32) | (((uint32_t)fr[0]) << 24);
        tx_time_f->tx_rawst += str_calculate_seconds(tx_rawst_high_bits);

    }

}

void tx_antd_formater(spi_frame fr, void *format, const size_t sub_register) {

    uint16_t register_value  = ((uint16_t)fr[1]) << 8 | fr[0];

    double* seconds = (double*) format;
    *seconds = calculate_tx_antd(register_value);

}

size_t tx_antd_unformater(void *format, spi_frame fr, const size_t sub_register) {

    double* seconds = ((double*) format);
    const uint16_t reg_value = tx_antd_calculate_register_val(*seconds);

    fr[1] = (reg_value & 0xFF00) >> 8;
    fr[0] = reg_value & 0xFF;

    return 2;
}

void sys_state_formater(spi_frame fr, void *format, const size_t sub_register) {

    sys_status_format* sys_status_f = (sys_status_format*) format;

    sys_status_f->pmsc_state = fr[2] & 0b00001111;
    sys_status_f->rx_state = fr[1] & 0b00011111;
    sys_status_f->tx_state = fr[0] & 0b00001111;

}

void ack_resp_t_formater(spi_frame fr, void *format, const size_t sub_register) {

    ack_resp_t_format* ack_resp_t_f = (ack_resp_t_format*)format;

    ack_resp_t_f->ack_tim = fr[3];

    const uint32_t register_value = ((uint32_t)(fr[2] & 0b00001111) << 16) |
            ((uint16_t)(fr[1]) << 8) | fr[0];
    ack_resp_t_f->w4r_tim = calculate_w4r_tim(register_value);

}

size_t ack_resp_t_unformater(void *format, spi_frame fr, const size_t sub_register) {

    ack_resp_t_format* ack_resp_t_f = (ack_resp_t_format*)format;

    fr[3] = ack_resp_t_f->ack_tim;

    const uint32_t reg_value = w4r_tim_calculate_register_val(ack_resp_t_f->w4r_tim);

    fr[2] = (reg_value & 0b11110000000000000000) >> 16;

    fr[1] = (reg_value & 0b00001111111100000000) >> 8;

    fr[0] = reg_value & 0b00000000000011111111;

    return 4;
}

void rx_sniff_formater(spi_frame fr, void *format, const size_t sub_register) {

    rx_sniff_format* rx_sniff_f = (rx_sniff_format*)format;

    rx_sniff_f->sniff_offt = calculate_sniff_offt(fr[1]);

    rx_sniff_f->sniff_ont = fr[0] & 0b00001111;

}

size_t rx_sniff_unformater(void *format, spi_frame fr, const size_t sub_register) {

    rx_sniff_format* rx_sniff_f = (rx_sniff_format*)format;

    fr[1] = sniff_offt_calculate_register_val(rx_sniff_f->sniff_offt);

    fr[0] = rx_sniff_f->sniff_ont;

    return 2;
}

void tx_power_formater(spi_frame fr, void *format, const size_t sub_register) {

    tx_power_format* tx_power_f = (tx_power_format*)format;

    tx_power_f->field_32_24.coarse_da_setting = (fr[3] & 0b11100000) >> 5;
    tx_power_f->field_32_24.fine_mixer_setting = fr[3] & 0b00011111;

    tx_power_f->field_23_16.coarse_da_setting = (fr[2] & 0b11100000) >> 5;
    tx_power_f->field_23_16.fine_mixer_setting = fr[2] & 0b00011111;

    tx_power_f->field_15_8.coarse_da_setting = (fr[1] & 0b11100000) >> 5;
    tx_power_f->field_15_8.fine_mixer_setting = fr[1] & 0b00011111;

    tx_power_f->field_7_0.coarse_da_setting = (fr[0] & 0b11100000) >> 5;
    tx_power_f->field_7_0.fine_mixer_setting = fr[0] & 0b00011111;

}

size_t tx_power_unformater(void *format, spi_frame fr, const size_t sub_register) {

    tx_power_format* tx_power_f = (tx_power_format*)format;

    fr[3] = tx_power_f->field_32_24.coarse_da_setting << 5;
    fr[3] |= tx_power_f->field_32_24.fine_mixer_setting;

    fr[2] = tx_power_f->field_23_16.coarse_da_setting << 5;
    fr[2] |= tx_power_f->field_23_16.fine_mixer_setting;

    fr[1] = tx_power_f->field_15_8.coarse_da_setting << 5;
    fr[1] |= tx_power_f->field_15_8.fine_mixer_setting;

    fr[0] = tx_power_f->field_7_0.coarse_da_setting << 5;
    fr[0] |= tx_power_f->field_7_0.fine_mixer_setting;

    return 4;
}

void chan_ctrl_formater(spi_frame fr, void *format, const size_t sub_register) {

    chan_ctrl_format* chan_ctrl_f = (chan_ctrl_format*)format;

    chan_ctrl_f->rx_pcode = (fr[3] & 0b11111000) >> 3;

    chan_ctrl_f->tx_pcode = (fr[2] & 0b11000000) >> 6;
    chan_ctrl_f->tx_pcode |= (fr[3] & 0b00000111) << 2;

    chan_ctrl_f->rnssfd = (fr[2] & 0b00100000) >> 5;

    chan_ctrl_f->tnssfd = (fr[2] & 0b00010000) >> 4;

    chan_ctrl_f->rxprf = (fr[2] & 0b00001100) >> 2;

    chan_ctrl_f->dwsfd = (fr[2] & 0b00000010) >> 1;

    chan_ctrl_f->rx_chan = (fr[0] & 0b11110000) >> 4;

    chan_ctrl_f->tx_chan = fr[0] & 0b00001111;

}

size_t chan_ctrl_unformater(void *format, spi_frame fr, const size_t sub_register) {

    chan_ctrl_format* chan_ctrl_f = (chan_ctrl_format*)format;

    fr[3] = ((chan_ctrl_f->tx_pcode & 0b11100) >> 2);
    fr[3] |= ((uint8_t)chan_ctrl_f->rx_pcode) << 3;

    fr[2] = chan_ctrl_f->dwsfd << 1;
    fr[2] |= chan_ctrl_f->rxprf << 2;
    fr[2] |= chan_ctrl_f->tnssfd << 4;
    fr[2] |= chan_ctrl_f->rnssfd << 5;
    fr[2] |= ((uint8_t)chan_ctrl_f->tx_pcode & 0b00011) << 6;

    fr[1] = 0;

    fr[0] = chan_ctrl_f->tx_chan;
    fr[0] |= ((uint8_t)chan_ctrl_f->rx_chan) << 4;

    return 4;
}

void usr_sfd_formater(spi_frame fr, void *format, const size_t sub_register) {

    usr_sfd_format* usr_sfd_f = (usr_sfd_format*) format;

    if(sub_register == USR_SFD_OCT_0_TO_3) {
        usr_sfd_f->sfd_length = fr[0];
        usr_sfd_f->tx_ssfd_magl = fr[1];
        usr_sfd_f->tx_ssfd_magh = fr[2];
        usr_sfd_f->tx_ssfd_sgnl = fr[3];
    } else if(sub_register == USR_SFD_OCT_4_TO_7) {
        usr_sfd_f->tx_ssfd_sgnh = fr[0];
        usr_sfd_f->rx_ssfd_magl = fr[1];
        usr_sfd_f->rx_ssfd_magh = fr[2];
        usr_sfd_f->rx_ssfd_sgnl = fr[3];
    } else if(sub_register == USR_SFD_OCT_8_TO_11) {
        usr_sfd_f->rx_ssfd_sgnh = fr[0];
        usr_sfd_f->tx_lsfd_mag0 = fr[1];
        usr_sfd_f->tx_lsfd_mag1 = fr[2];
        usr_sfd_f->tx_lsfd_mag2 = fr[3];
    } else if(sub_register == USR_SFD_OCT_12_TO_15) {
        usr_sfd_f->tx_lsfd_mag3 = fr[0];
        usr_sfd_f->tx_lsfd_mag4 = fr[1];
        usr_sfd_f->tx_lsfd_mag5 = fr[2];
        usr_sfd_f->tx_lsfd_mag6 = fr[3];
    } else if(sub_register == USR_SFD_OCT_16_TO_19) {
        usr_sfd_f->tx_lsfd_mag7 = fr[0];
        usr_sfd_f->tx_lsfd_sgn0 = fr[1];
        usr_sfd_f->tx_lsfd_sgn1 = fr[2];
        usr_sfd_f->tx_lsfd_sgn2 = fr[3];
    } else if(sub_register == USR_SFD_OCT_20_TO_23) {
        usr_sfd_f->tx_lsfd_sgn3 = fr[0];
        usr_sfd_f->tx_lsfd_sgn4 = fr[1];
        usr_sfd_f->tx_lsfd_sgn5 = fr[2];
        usr_sfd_f->tx_lsfd_sgn6 = fr[3];
    } else if(sub_register == USR_SFD_OCT_24_TO_27) {
        usr_sfd_f->tx_lsfd_sgn7 = fr[0];
        usr_sfd_f->rx_lsfd_mag0 = fr[1];
        usr_sfd_f->rx_lsfd_mag1 = fr[2];
        usr_sfd_f->rx_lsfd_mag2 = fr[3];
    } else if(sub_register == USR_SFD_OCT_28_TO_31) {
        usr_sfd_f->rx_lsfd_mag3 = fr[0];
        usr_sfd_f->rx_lsfd_mag4 = fr[1];
        usr_sfd_f->rx_lsfd_mag5 = fr[2];
        usr_sfd_f->rx_lsfd_mag6 = fr[3];
    } else if(sub_register == USR_SFD_OCT_32_TO_35) {
        usr_sfd_f->rx_lsfd_mag7 = fr[0];
        usr_sfd_f->rx_lsfd_sgn0 = fr[1];
        usr_sfd_f->rx_lsfd_sgn1 = fr[2];
        usr_sfd_f->rx_lsfd_sgn2 = fr[3];
    } else if(sub_register == USR_SFD_OCT_36_TO_39) {
        usr_sfd_f->rx_lsfd_sgn2 = fr[0];
        usr_sfd_f->rx_lsfd_sgn3 = fr[1];
        usr_sfd_f->rx_lsfd_sgn4 = fr[2];
        usr_sfd_f->rx_lsfd_sgn5 = fr[3];
    } else if(sub_register == USR_SFD_OCT_40_TO_41) {
        usr_sfd_f->rx_lsfd_sgn6 = fr[0];
        usr_sfd_f->rx_lsfd_sgn7 = fr[1];
    }

}

size_t usr_sfd_unformater(void *format, spi_frame fr, const size_t sub_register) {

    usr_sfd_format* usr_sfd_f = (usr_sfd_format*) format;

    if(sub_register == USR_SFD_OCT_0_TO_3) {
        fr[0] = usr_sfd_f->sfd_length;
        fr[1] = usr_sfd_f->tx_ssfd_magl;
        fr[2] = usr_sfd_f->tx_ssfd_magh;
        fr[3] = usr_sfd_f->tx_ssfd_sgnl;
    } else if(sub_register == USR_SFD_OCT_4_TO_7) {
        fr[0] = usr_sfd_f->tx_ssfd_sgnh;
        fr[1] = usr_sfd_f->rx_ssfd_magl;
        fr[2] = usr_sfd_f->rx_ssfd_magh;
        fr[3] = usr_sfd_f->rx_ssfd_sgnl;
    } else if(sub_register == USR_SFD_OCT_8_TO_11) {
        fr[0] = usr_sfd_f->rx_ssfd_sgnh;
        fr[1] = usr_sfd_f->tx_lsfd_mag0;
        fr[2] = usr_sfd_f->tx_lsfd_mag1;
        fr[3] = usr_sfd_f->tx_lsfd_mag2;
    } else if(sub_register == USR_SFD_OCT_12_TO_15) {
        fr[0] = usr_sfd_f->tx_lsfd_mag3;
        fr[1] = usr_sfd_f->tx_lsfd_mag4;
        fr[2] = usr_sfd_f->tx_lsfd_mag5;
        fr[3] = usr_sfd_f->tx_lsfd_mag6;
    } else if(sub_register == USR_SFD_OCT_16_TO_19) {
        fr[0] = usr_sfd_f->tx_lsfd_mag7;
        fr[1] = usr_sfd_f->tx_lsfd_sgn0;
        fr[2] = usr_sfd_f->tx_lsfd_sgn1;
        fr[3] = usr_sfd_f->tx_lsfd_sgn2 ;
    } else if(sub_register == USR_SFD_OCT_20_TO_23) {
        fr[0] = usr_sfd_f->tx_lsfd_sgn3;
        fr[1] = usr_sfd_f->tx_lsfd_sgn4;
        fr[2] = usr_sfd_f->tx_lsfd_sgn5;
        fr[3] = usr_sfd_f->tx_lsfd_sgn6;
    } else if(sub_register == USR_SFD_OCT_24_TO_27) {
        fr[0] = usr_sfd_f->tx_lsfd_sgn7;
        fr[1] = usr_sfd_f->rx_lsfd_mag0;
        fr[2] = usr_sfd_f->rx_lsfd_mag1;
        fr[3] = usr_sfd_f->rx_lsfd_mag2;
    } else if(sub_register == USR_SFD_OCT_28_TO_31) {
        fr[0] = usr_sfd_f->rx_lsfd_mag3;
        fr[1] = usr_sfd_f->rx_lsfd_mag4;
        fr[2] = usr_sfd_f->rx_lsfd_mag5;
        fr[3] = usr_sfd_f->rx_lsfd_mag6;
    } else if(sub_register == USR_SFD_OCT_32_TO_35) {
        fr[0] = usr_sfd_f->rx_lsfd_mag7;
        fr[1] = usr_sfd_f->rx_lsfd_sgn0;
        fr[2] = usr_sfd_f->rx_lsfd_sgn1;
        fr[3] = usr_sfd_f->rx_lsfd_sgn2;
    } else if(sub_register == USR_SFD_OCT_36_TO_39) {
        fr[0] = usr_sfd_f->rx_lsfd_sgn2;
        fr[1] = usr_sfd_f->rx_lsfd_sgn3;
        fr[2] = usr_sfd_f->rx_lsfd_sgn4;
        fr[3] = usr_sfd_f->rx_lsfd_sgn5;
    } else if(sub_register == USR_SFD_OCT_40_TO_41) {
        fr[0] = usr_sfd_f->rx_lsfd_sgn6;
        fr[1] = usr_sfd_f->rx_lsfd_sgn7;

        return 2;
    }

    return 4;
}

void agc_ctrl_formater(spi_frame fr, void *format, const size_t sub_register) {

    agc_ctrl_format* agc_ctrl_f = (agc_ctrl_format*) format;

    if(sub_register == AGC_CTRL1) {

        agc_ctrl_f->dis_am = fr[0] & 0b00000001;

    } else if(sub_register == AGC_TUNE1) {

        agc_ctrl_f->agc_tune1 = (((uint16_t)fr[1]) << 8) | fr[0];

    } else if(sub_register == AGC_TUNE2) {

        agc_ctrl_f->agc_tune2 = (((uint32_t)fr[3]) << 24) | (((uint32_t)fr[2]) << 16) |
                (((uint16_t)fr[1]) << 8) | fr[0];

    } else if(sub_register == AGC_TUNE3) {

        agc_ctrl_f->agc_tune3 = (((uint16_t)fr[1]) << 8) | fr[0];

    } else if(sub_register == AGC_STAT1) {

        agc_ctrl_f->edv2 = (fr[1] & 0b11111000) >> 3;
        agc_ctrl_f->edv2 |= ((uint16_t)(fr[2] & 0b00001111)) << 5;

        agc_ctrl_f->edg1 = (fr[0] & 0b11000000) >> 6;
        agc_ctrl_f->edg1 |= (fr[1] & 0b00000111) << 2;
    }

}

size_t agc_ctrl_unformater(void *format, spi_frame fr, const size_t sub_register) {

    agc_ctrl_format* agc_ctrl_f = (agc_ctrl_format*) format;

    if(sub_register == AGC_CTRL1) {

        fr[0] = agc_ctrl_f->dis_am;

        return 1;

    } else if(sub_register == AGC_TUNE1) {

        agc_ctrl_f->agc_tune1 = (((uint16_t)fr[1]) << 8) | fr[0];

        fr[0] = agc_ctrl_f->agc_tune1 & 0x00FF;
        fr[1] = (agc_ctrl_f->agc_tune1 & 0xFF00) >> 8;

        return 2;

    } else if(sub_register == AGC_TUNE2) {

        agc_ctrl_f->agc_tune2 = (((uint32_t)fr[3]) << 24) | (((uint32_t)fr[2]) << 16) |
                (((uint16_t)fr[1]) << 8) | fr[0];

        fr[0] = agc_ctrl_f->agc_tune2 & 0x000000FF;
        fr[1] = (agc_ctrl_f->agc_tune2 & 0x0000FF00) >> 8;
        fr[2] = (agc_ctrl_f->agc_tune2 & 0x00FF0000) >> 16;
        fr[3] = (agc_ctrl_f->agc_tune2 & 0xFF000000) >> 24;

        return 4;

    } else if(sub_register == AGC_TUNE3) {

        fr[0] = agc_ctrl_f->agc_tune3 & 0x00FF;
        fr[1] = (agc_ctrl_f->agc_tune3 & 0xFF00) >> 8;

        return 2;

    } else if(sub_register == AGC_STAT1) {

        agc_ctrl_f->edv2 = (fr[1] & 0b11111000) >> 3;
        agc_ctrl_f->edv2 |= ((uint16_t)(fr[2] & 0b00001111)) << 5;

        agc_ctrl_f->edg1 = (fr[0] & 0b11000000) >> 6;
        agc_ctrl_f->edg1 |= (fr[1] & 0b00000111) << 2;

        fr[0] = (agc_ctrl_f->edg1 & 0b00011) << 6;

        fr[1] = (agc_ctrl_f->edv2 & 0b000011111) | ((agc_ctrl_f->edg1 & 0b11100) >> 2);

        fr[2] = (agc_ctrl_f->edv2 & 0b111100000) >> 5;

        return 3;
    }

    return 0;
}
