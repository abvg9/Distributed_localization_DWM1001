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

#include "format.h"

void dev_id_formatter(spi_frame fr, void *format, const size_t sub_register) {

    dev_id_format *dev_id_f = (dev_id_format*) format;

    dev_id_f->ridtag = fr[2] | ((uint16_t) fr[3]) << 8;
    dev_id_f->model = fr[1];
    dev_id_f->ver = (fr[0] & 0XF0) >> 4;
    dev_id_f->rev = (fr[0] & 0XF);
}

void eui_formatter(spi_frame fr, void *format, const size_t sub_register) {

    eui_format *eui_f = (eui_format*) format;

    eui_f->mc_ID = fr[5] | (uint16_t) fr[6] << 8 | (uint32_t) fr[7] << 16;
    eui_f->ext_ID = fr[0] | (uint16_t) fr[1] << 8 | (uint32_t) fr[2] << 16
            | (uint32_t) fr[3] << 24 | (uint64_t) fr[4] << 32;
}

size_t eui_unformatter(void *format, spi_frame fr, const size_t sub_register) {

    eui_format *eui_f = ((eui_format*) format);

    fr[4] = (eui_f->ext_ID & 0xFF00000000) >> 32;
    fr[3] = (eui_f->ext_ID & 0x00FF000000) >> 24;
    fr[2] = (eui_f->ext_ID & 0x0000FF0000) >> 16;
    fr[1] = (eui_f->ext_ID & 0x000000FF00) >> 8;
    fr[0] = eui_f->ext_ID & 0x00000000FF;

    fr[7] = (eui_f->mc_ID & 0xFF0000) >> 16;
    fr[6] = (eui_f->mc_ID & 0x00FF00) >> 8;
    fr[5] = eui_f->mc_ID & 0x0000FF;

    return 8;
}

void pan_addr_formatter(spi_frame fr, void *format, const size_t sub_register) {

    pan_adr_format *pan_adr_f = (pan_adr_format*) format;

    pan_adr_f->pan_id = fr[2] | ((uint16_t) fr[3]) << 8;
    pan_adr_f->short_addr = fr[0] | ((uint16_t) fr[1]) << 8;
}

size_t pan_addr_unformatter(void *format, spi_frame fr, const size_t sub_register) {

    pan_adr_format *pan_adr_f = ((pan_adr_format*) format);

    if(sub_register == PAN_ID) {

        fr[1] = (pan_adr_f->pan_id & 0xFF00) >> 8;
        fr[0] = pan_adr_f->pan_id & 0xFF;

        return 2;

    } else if(sub_register == SHORT_ADR) {

        fr[1] = (pan_adr_f->short_addr & 0xFF00) >> 8;
        fr[0] = pan_adr_f->short_addr & 0xFF;

        return 2;

    }

    return 0;
}

void sys_cfg_formatter(spi_frame fr, void *format, const size_t sub_register) {

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

size_t sys_cfg_unformatter(void *format, spi_frame fr, const size_t sub_register) {

    sys_cfg_format *sys_cfg_f = ((sys_cfg_format*) format);

    fr[3] = ((uint8_t)sys_cfg_f->aackpend) << 7;
    fr[3] |= ((uint8_t)sys_cfg_f->autoack) << 6;
    fr[3] |= ((uint8_t)sys_cfg_f->rxautr) << 5;
    fr[3] |= ((uint8_t)sys_cfg_f->rxwtoe) << 4;

    fr[2] = ((uint8_t)sys_cfg_f->rxm110k) << 6;
    fr[2] |= ((uint8_t)sys_cfg_f->dis_stxp) << 3;
    fr[2] |= sys_cfg_f->phr_mode;

    fr[1] = ((uint8_t)sys_cfg_f->fcs_init2f) << 7;
    fr[1] |= ((uint8_t)sys_cfg_f->dis_rsde) << 6;
    fr[1] |= ((uint8_t)sys_cfg_f->dis_phe) << 5;
    fr[1] |= ((uint8_t)sys_cfg_f->dis_drxb) << 4;
    fr[1] |= ((uint8_t)sys_cfg_f->dis_fce) << 3;
    fr[1] |= ((uint8_t)sys_cfg_f->spi_edge) << 2;
    fr[1] |= ((uint8_t)sys_cfg_f->hirq_pol) << 1;
    fr[1] |= sys_cfg_f->ffa5;

    fr[0] = ((uint8_t)sys_cfg_f->ffa4) << 7;
    fr[0] |= ((uint8_t)sys_cfg_f->ffar) << 6;
    fr[0] |= ((uint8_t)sys_cfg_f->ffam) << 5;
    fr[0] |= ((uint8_t)sys_cfg_f->ffaa) << 4;
    fr[0] |= ((uint8_t)sys_cfg_f->ffad) << 3;
    fr[0] |= ((uint8_t)sys_cfg_f->ffab) << 2;
    fr[0] |= ((uint8_t)sys_cfg_f->ffbc) << 1;
    fr[0] |= sys_cfg_f->ffen;

    return 4;
}

void sys_time_formatter(spi_frame fr, void* format, const size_t sub_register) {

    uint64_t register_value  = (uint16_t) fr[1] << 8 | (uint32_t) fr[2] << 16
            | (uint32_t) fr[3] << 24 | (uint64_t) fr[4] << 32;

    double* seconds = ((double*) format);
    *seconds = str_calculate_seconds(register_value);
}

void tx_fctrl_formatter(spi_frame fr, void* format, const size_t sub_register) {

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

size_t tx_fctrl_unformatter(void* format, spi_frame fr, const size_t sub_register) {

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

void payload_formatter_f(spi_frame fr, uwb_frame_format* format, size_t* payload_size) {

    int start_raw_payload_index = 0;

    // API flags.
    format->api_message_t = fr[start_raw_payload_index++];
    *payload_size = *payload_size + 1;

    switch(format->api_message_t) {
        case CALC_DISTANCE:
            break;
        case CALC_DISTANCE_RESP_RX: {

            memcpy(&format->rx_stamp, &fr[1], sizeof(double));
            start_raw_payload_index += sizeof(double);
            *payload_size = *payload_size + sizeof(double);
            break;
        }
        case CALC_DISTANCE_RESP_TX: {

            memcpy(&format->tx_stamp, &fr[start_raw_payload_index], sizeof(double));
            start_raw_payload_index += sizeof(double);
            *payload_size = *payload_size + sizeof(double);
            break;
        }
        default:
            break;
    }

    // Payload.
    int i;
    for(i = start_raw_payload_index; i < format->raw_payload_size; ++i) {
        format->raw_payload[i] = fr[i];
    }

}

void payload_unformatter_f(const uwb_frame_format format, spi_frame fr, size_t* payload_size) {

    int start_payload_index = 0;

    // API flags.
    fr[start_payload_index++] = format.api_message_t;
    *payload_size = *payload_size + 1;

    switch(format.api_message_t) {
        case CALC_DISTANCE:
            break;
        case CALC_DISTANCE_RESP_RX: {

            char* rx_stamp_bits = (char *) &format.rx_stamp;
            unsigned int i;

            for(i = 0; i < sizeof(double); ++i) {
                fr[start_payload_index] = rx_stamp_bits[i];
                start_payload_index++;
            }
            *payload_size = *payload_size + sizeof(double);
            break;
        }

        case CALC_DISTANCE_RESP_TX: {

            char* tx_stamp_bits = (char *) &format.tx_stamp;
            unsigned int i;

            for(i = 0; i < sizeof(double); ++i) {
                fr[start_payload_index] = tx_stamp_bits[i];
                start_payload_index++;
            }
            *payload_size = *payload_size + sizeof(double);
            break;
        }
        default:
            break;
    }

    // Payload.
    int i;
    for(i = start_payload_index; i < format.raw_payload_size; ++i) {
        fr[i] = format.raw_payload[i];
    }
}

size_t tx_buffer_unformatter(void *format, spi_frame fr, const size_t sub_register) {

    uwb_frame_format* uwb_frame_f = (uwb_frame_format*) format;

    // Frame control.
    fr[0] = (uint8_t)uwb_frame_f->frame_t;
    fr[0] |= ((uint8_t)uwb_frame_f->seq_enab) << 3;

    fr[0] |= ((uint8_t)uwb_frame_f->frame_pend) << 4;
    fr[0] |= ((uint8_t)uwb_frame_f->ack_req) << 5;
    fr[0] |= ((uint8_t)uwb_frame_f->intra_PAN) << 6;

    fr[1] = ((uint8_t)uwb_frame_f->dest_addr_mod) << 2;
    fr[1] |= ((uint8_t)uwb_frame_f->sour_addr_mod) << 6;

    // Sequence number.
    fr[2] = uwb_frame_f->seq_num;

    size_t start_payload_byte = 3;

    // Addressing fields.
    switch(uwb_frame_f->dest_addr_mod) {
        case SHORT_ADDRESS:
            fr[start_payload_byte++] = uwb_frame_f->dest_PAN_id & 0x00FF;
            fr[start_payload_byte++] = (uwb_frame_f->dest_PAN_id & 0xFF00) >> 8;

            fr[start_payload_byte++] = uwb_frame_f->dest_addr & 0x00FF;
            fr[start_payload_byte++] = (uwb_frame_f->dest_addr & 0xFF00) >> 8;

            break;
        case EXTENDED_ADDRESS:
            fr[start_payload_byte++] = uwb_frame_f->dest_PAN_id & 0x00FF;
            fr[start_payload_byte++] = (uwb_frame_f->dest_PAN_id & 0xFF00) >> 8;

            fr[start_payload_byte++] =  uwb_frame_f->dest_addr & 0x00000000000000FF;
            fr[start_payload_byte++] = (uwb_frame_f->dest_addr & 0x000000000000FF00) >> 8;
            fr[start_payload_byte++] = (uwb_frame_f->dest_addr & 0x0000000000FF0000) >> 16;
            fr[start_payload_byte++] = (uwb_frame_f->dest_addr & 0x00000000FF000000) >> 24;
            fr[start_payload_byte++] = (uwb_frame_f->dest_addr & 0x000000FF00000000) >> 32;
            fr[start_payload_byte++] = (uwb_frame_f->dest_addr & 0x0000FF0000000000) >> 40;
            fr[start_payload_byte++] = (uwb_frame_f->dest_addr & 0x00FF000000000000) >> 48;
            fr[start_payload_byte++] = (uwb_frame_f->dest_addr & 0xFF00000000000000) >> 56;
            break;
        default:
            break;
    }

    switch(uwb_frame_f->sour_addr_mod) {
        case SHORT_ADDRESS:
            fr[start_payload_byte++] = uwb_frame_f->sour_PAN_id & 0x00FF;
            fr[start_payload_byte++] = (uwb_frame_f->sour_PAN_id & 0xFF00) >> 8;

            fr[start_payload_byte++] = uwb_frame_f->sour_addr & 0x00FF;
            fr[start_payload_byte++] = (uwb_frame_f->sour_addr & 0xFF00) >> 8;
            break;
        case EXTENDED_ADDRESS:
            fr[start_payload_byte++] = uwb_frame_f->sour_PAN_id & 0x00FF;
            fr[start_payload_byte++] = (uwb_frame_f->sour_PAN_id & 0xFF00) >> 8;

            fr[start_payload_byte++] =  uwb_frame_f->sour_addr & 0x00000000000000FF;
            fr[start_payload_byte++] = (uwb_frame_f->sour_addr & 0x000000000000FF00) >> 8;
            fr[start_payload_byte++] = (uwb_frame_f->sour_addr & 0x0000000000FF0000) >> 16;
            fr[start_payload_byte++] = (uwb_frame_f->sour_addr & 0x00000000FF000000) >> 24;
            fr[start_payload_byte++] = (uwb_frame_f->sour_addr & 0x000000FF00000000) >> 32;
            fr[start_payload_byte++] = (uwb_frame_f->sour_addr & 0x0000FF0000000000) >> 40;
            fr[start_payload_byte++] = (uwb_frame_f->sour_addr & 0x00FF000000000000) >> 48;
            fr[start_payload_byte++] = (uwb_frame_f->sour_addr & 0xFF00000000000000) >> 56;
            break;
        default:
            break;
    }

    fr[start_payload_byte++] = uwb_frame_f->raw_payload_size & 0x00FF;
    fr[start_payload_byte++] = (uwb_frame_f->raw_payload_size & 0xFF00) >> 8;

    // Payload.
    if(uwb_frame_f->frame_t != ACKNOWLEDGMENT) {
        payload_unformatter_f(*uwb_frame_f, &fr[start_payload_byte], &start_payload_byte);
    }

    uwb_frame_f->frame_size = start_payload_byte + uwb_frame_f->raw_payload_size;

    // Check sum.
    fr[uwb_frame_f->frame_size++] = uwb_frame_f->check_sum & 0x00FF;
    fr[uwb_frame_f->frame_size++] = (uwb_frame_f->check_sum & 0xFF00) >> 8;

    return uwb_frame_f->frame_size;
}

void dx_time_formatter(spi_frame fr, void *format, const size_t sub_register) {

    const uint64_t register_value  = (uint16_t) fr[1] << 8 | (uint32_t) fr[2] << 16
            | (uint32_t) fr[3] << 24 | (uint64_t) fr[4] << 32;

    double* seconds = ((double*) format);
    *seconds = dtr_calculate_seconds(register_value);

}

size_t dx_time_unformatter(void *format, spi_frame fr, const size_t sub_register) {

    double* seconds = ((double*) format);
    const uint64_t reg_value = dtr_calculate_register_val(*seconds);

    fr[4] = (reg_value & 0xFF00000000) >> 32;
    fr[3] = (reg_value & 0x00FF000000) >> 24;
    fr[2] = (reg_value & 0x0000FF0000) >> 16;
    fr[1] = (reg_value & 0x000000FF00) >> 8;
    fr[0] = reg_value & 0x00000000FF;

    return 5;
}

void rx_fwto_formatter(spi_frame fr, void *format, const size_t sub_register) {

    uint16_t register_value  = fr[0] | (uint16_t) fr[1] << 8;

    double* seconds = ((double*) format);
    *seconds = rfr_calculate_seconds(register_value);

}

size_t rx_fwto_unformatter(void *format, spi_frame fr, const size_t sub_register) {

    double* seconds = ((double*) format);
    uint16_t register_value = rfr_calculate_register_val(*seconds);

    fr[1] = (register_value & 0xFF00) >> 8;
    fr[0] = register_value & 0x00FF;

    return 2;
}

void tx_ctrl_formatter(spi_frame fr, void *format, const size_t sub_register) {

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

size_t tx_ctrl_unformatter(void *format, spi_frame fr, const size_t sub_register) {

    sys_ctrl_format *sys_ctrl_f = ((sys_ctrl_format*) format);

    fr[3] = sys_ctrl_f->hrbpt;

    fr[2] = 0;

    fr[1] = ((uint8_t)sys_ctrl_f->rxdlye) << 1;
    fr[1] |= sys_ctrl_f->rxenab;

    fr[0] = ((uint8_t)sys_ctrl_f->wait4resp) << 7;
    fr[0] |= ((uint8_t)sys_ctrl_f->trxoff) << 6;
    fr[0] |= ((uint8_t)sys_ctrl_f->cansfcs) << 3;
    fr[0] |= ((uint8_t)sys_ctrl_f->txdlys) << 2;
    fr[0] |= ((uint8_t)sys_ctrl_f->txstrt) << 1;
    fr[0] |= sys_ctrl_f->sfcst;

    return 4;
}

void sys_evt_msk_formatter(spi_frame fr, void *format, const size_t sub_register) {

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

size_t sys_evt_msk_unformatter(void *format, spi_frame fr, const size_t sub_register) {

    sys_evt_msk_format *sys_evt_msk_f = ((sys_evt_msk_format*) format);

    fr[3] = ((uint8_t)sys_evt_msk_f->maffrej) << 5;
    fr[3] |= ((uint8_t)sys_evt_msk_f->mtxberr) << 4;
    fr[3] |= ((uint8_t)sys_evt_msk_f->mhpdwarn) << 3;
    fr[3] |= ((uint8_t)sys_evt_msk_f->mrxsfdto) << 2;
    fr[3] |= ((uint8_t)sys_evt_msk_f->mcpllll) << 1;
    fr[3] |= sys_evt_msk_f->mrfpllll;

    fr[2] = ((uint8_t)sys_evt_msk_f->mslp2init) << 7;
    fr[2] |= ((uint8_t)sys_evt_msk_f->mgpioirq) << 6;
    fr[2] |= ((uint8_t)sys_evt_msk_f->mrxpto) << 5;
    fr[2] |= ((uint8_t)sys_evt_msk_f->mldeerr) << 2;
    fr[2] |= ((uint8_t)sys_evt_msk_f->mrxrfto) << 1;
    fr[2] |= sys_evt_msk_f->mrxrfsl;

    fr[1] = ((uint8_t)sys_evt_msk_f->mrxfce) << 7;
    fr[1] |= ((uint8_t)sys_evt_msk_f->mrxfcg) << 6;
    fr[1] |= ((uint8_t)sys_evt_msk_f->mrxdfr) << 5;
    fr[1] |= ((uint8_t)sys_evt_msk_f->mrxphe) << 4;
    fr[1] |= ((uint8_t)sys_evt_msk_f->mrxphd) << 3;
    fr[1] |= ((uint8_t)sys_evt_msk_f->mldedone) << 2;
    fr[1] |= ((uint8_t)sys_evt_msk_f->mrxsfdd) << 1;
    fr[1] |= sys_evt_msk_f->mrxprd;

    fr[0] = ((uint8_t)sys_evt_msk_f->mtxfrs) << 7;
    fr[0] |= ((uint8_t)sys_evt_msk_f->mtxphs) << 6;
    fr[0] |= ((uint8_t)sys_evt_msk_f->mtxprs) << 5;
    fr[0] |= ((uint8_t)sys_evt_msk_f->mtxfrb) << 4;
    fr[0] |= ((uint8_t)sys_evt_msk_f->maat) << 3;
    fr[0] |= ((uint8_t)sys_evt_msk_f->mesyncr) << 2;
    fr[0] |= ((uint8_t)sys_evt_msk_f->mcplock) << 1;

    return 4;
}

void sys_evt_sts_formatter(spi_frame fr, void *format, const size_t sub_register) {

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

size_t sys_evt_sts_unformatter(void *format, spi_frame fr, const size_t sub_register) {

    sys_evt_sts_format *sys_evt_sts_f = ((sys_evt_sts_format*) format);

     if(sub_register == SES_OCT_0_TO_3) {

         fr[3] |= ((uint8_t)sys_evt_sts_f->affrej) << 5;
         fr[3] |= ((uint8_t)sys_evt_sts_f->txberr) << 4;
         fr[3] |= ((uint8_t)sys_evt_sts_f->rxsfdto) << 2;
         fr[3] |= ((uint8_t)sys_evt_sts_f->clkpll_ll) << 1;
         fr[3] |= sys_evt_sts_f->rfpll_ll;

         fr[2] = ((uint8_t)sys_evt_sts_f->slp2init) << 7;
         fr[2] |= ((uint8_t)sys_evt_sts_f->gpioirq) << 6;
         fr[2] |= ((uint8_t)sys_evt_sts_f->rxpto) << 5;
         fr[2] |= ((uint8_t)sys_evt_sts_f->rxovrr) << 4;
         fr[2] |= ((uint8_t)sys_evt_sts_f->ldeerr) << 2;
         fr[2] |= ((uint8_t)sys_evt_sts_f->rxrfto) << 1;
         fr[2] |= sys_evt_sts_f->rxrfsl;

         fr[1] = ((uint8_t)sys_evt_sts_f->rxfce) << 7;
         fr[1] |= ((uint8_t)sys_evt_sts_f->rxfcg) << 6;
         fr[1] |= ((uint8_t)sys_evt_sts_f->rxdfr) << 5;
         fr[1] |= ((uint8_t)sys_evt_sts_f->rxphe) << 4;
         fr[1] |= ((uint8_t)sys_evt_sts_f->rxphd) << 3;
         fr[1] |= ((uint8_t)sys_evt_sts_f->ldedone) << 2;
         fr[1] |= ((uint8_t)sys_evt_sts_f->rxsfdd) << 1;
         fr[1] |= sys_evt_sts_f->rxprd;

         fr[0] = ((uint8_t)sys_evt_sts_f->txfrs) << 7;
         fr[0] |= ((uint8_t)sys_evt_sts_f->txphs) << 6;
         fr[0] |= ((uint8_t)sys_evt_sts_f->txprs) << 5;
         fr[0] |= ((uint8_t)sys_evt_sts_f->txfrb) << 4;
         fr[0] |= ((uint8_t)sys_evt_sts_f->aat) << 3;
         fr[0] |= ((uint8_t)sys_evt_sts_f->esyncr) << 2;
         fr[0] |= ((uint8_t)sys_evt_sts_f->cplock) << 1;

         return 4;

     } else if(sub_register == SES_OCT_4) {

         fr[0] = ((uint8_t)sys_evt_sts_f->rxprej) << 1;
         fr[0] |= sys_evt_sts_f->rxrscs;

         return 1;
     }

     return 0;
}

void rx_finfo_formatter(spi_frame fr, void *format, const size_t sub_register) {

    rx_finfo_format *rx_finfo_f = ((rx_finfo_format*) format);

    rx_finfo_f->rxpacc = ((uint16_t)fr[3]) << 4 | ((fr[2] & 0b11110000) >> 4);
    rx_finfo_f->rxnspl_rxpsr =  ((fr[1] & 0b00011000) >> 1) | ((fr[2] & 0b00001100) >> 2);
    rx_finfo_f->rxprfr = fr[2] & 0b00000011;
    rx_finfo_f->rng = (fr[1] & 0b10000000) >> 7;
    rx_finfo_f->rxbr = (fr[1] & 0b01100000) >> 5;
    rx_finfo_f->rxfle = ((fr[1] && 0b00000011) << 1)  | ((fr[0] & 0b10000000) >> 7);
    rx_finfo_f->rxflen = fr[0] & 0b01111111;

}

void rx_buffer_formatter(spi_frame fr, void *format, const size_t sub_register) {

    uwb_frame_format* uwb_frame_f = (uwb_frame_format*) format;

    // Frame control.
    uwb_frame_f->frame_t = fr[0] & 0b00000111;
    uwb_frame_f->seq_enab = (fr[0] & 0b00001000) >> 3;
    uwb_frame_f->frame_pend = (fr[0] & 0b00010000) >> 4;
    uwb_frame_f->ack_req = (fr[0] & 0b00100000) >> 5;
    uwb_frame_f->intra_PAN = (fr[0] & 0b01000000) >> 6;

    uwb_frame_f->dest_addr_mod = (fr[1] & 0b00001100) >> 2;
    uwb_frame_f->sour_addr_mod = (fr[1] & 0b11000000) >> 6;

    // Sequence number.
    uwb_frame_f->seq_num = fr[2];

    size_t start_payload_byte = 3;

    // Addressing fields.
    switch(uwb_frame_f->dest_addr_mod) {
        case SHORT_ADDRESS:
            uwb_frame_f->dest_PAN_id = fr[start_payload_byte++];
            uwb_frame_f->dest_PAN_id |= ((uint16_t)fr[start_payload_byte++]) << 8;

            uwb_frame_f->dest_addr = fr[start_payload_byte++];
            uwb_frame_f->dest_addr |= ((uint16_t)fr[start_payload_byte++]) << 8;
            break;
        case EXTENDED_ADDRESS:

            uwb_frame_f->dest_PAN_id = fr[start_payload_byte++];
            uwb_frame_f->dest_PAN_id |= ((uint16_t)fr[start_payload_byte++]) << 8;

            uwb_frame_f->dest_addr = fr[start_payload_byte++];
            uwb_frame_f->dest_addr |= ((uint16_t)fr[start_payload_byte++]) << 8;
            uwb_frame_f->dest_addr |= ((uint32_t)fr[start_payload_byte++]) << 16;
            uwb_frame_f->dest_addr |= ((uint32_t)fr[start_payload_byte++]) << 24;
            uwb_frame_f->dest_addr |= ((uint64_t)fr[start_payload_byte++]) << 32;
            uwb_frame_f->dest_addr |= ((uint64_t)fr[start_payload_byte++]) << 40;
            uwb_frame_f->dest_addr |= ((uint64_t)fr[start_payload_byte++]) << 48;
            uwb_frame_f->dest_addr |= ((uint64_t)fr[start_payload_byte++]) << 56;

            break;
        case PAN_ID_AND_ADDRESS_ARE_NOT_PRESENT:
            uwb_frame_f->dest_PAN_id = 0;
            uwb_frame_f->dest_addr = 0;
            break;
        default:
            break;
    }

    switch(uwb_frame_f->sour_addr_mod) {
        case SHORT_ADDRESS:
            uwb_frame_f->sour_PAN_id = fr[start_payload_byte++];
            uwb_frame_f->sour_PAN_id |= ((uint16_t)fr[start_payload_byte++]) << 8;

            uwb_frame_f->sour_addr = fr[start_payload_byte++];
            uwb_frame_f->sour_addr |= ((uint16_t)fr[start_payload_byte++]) << 8;
            break;
        case EXTENDED_ADDRESS:

            uwb_frame_f->sour_PAN_id = fr[start_payload_byte++];
            uwb_frame_f->sour_PAN_id |= ((uint16_t)fr[start_payload_byte++]) << 8;

            uwb_frame_f->sour_addr = fr[start_payload_byte++];
            uwb_frame_f->sour_addr |= ((uint16_t)fr[start_payload_byte++]) << 8;
            uwb_frame_f->sour_addr |= ((uint32_t)fr[start_payload_byte++]) << 16;
            uwb_frame_f->sour_addr |= ((uint32_t)fr[start_payload_byte++]) << 24;
            uwb_frame_f->sour_addr |= ((uint64_t)fr[start_payload_byte++]) << 32;
            uwb_frame_f->sour_addr |= ((uint64_t)fr[start_payload_byte++]) << 40;
            uwb_frame_f->sour_addr |= ((uint64_t)fr[start_payload_byte++]) << 48;
            uwb_frame_f->sour_addr |= ((uint64_t)fr[start_payload_byte++]) << 56;
            break;
        case PAN_ID_AND_ADDRESS_ARE_NOT_PRESENT:
            uwb_frame_f->sour_PAN_id = 0;
            uwb_frame_f->sour_addr = 0;
            break;
        default:
            break;
    }

    uwb_frame_f->raw_payload_size = (fr[start_payload_byte++]);
    uwb_frame_f->raw_payload_size |= ((uint16_t)fr[start_payload_byte++]) << 8;

    // Payload.
    if(uwb_frame_f->frame_t != ACKNOWLEDGMENT) {
        payload_formatter_f(&fr[start_payload_byte], uwb_frame_f, &start_payload_byte);
    }

    uwb_frame_f->frame_size = start_payload_byte + uwb_frame_f->raw_payload_size;

    // Check sum.
    uwb_frame_f->check_sum = fr[uwb_frame_f->frame_size++];
    uwb_frame_f->check_sum |= ((uint16_t)fr[uwb_frame_f->frame_size++]) << 8;

}

void rx_fqual_formatter(spi_frame fr, void *format, const size_t sub_register) {

    rx_fqual_format *rx_fqual_f = ((rx_fqual_format*) format);

    if(sub_register == FP_AMPL2_STD_NOISE) {

        rx_fqual_f->fp_ampl2 = ((uint16_t)fr[3] << 8) | fr[2];
        rx_fqual_f->std_noise = ((uint16_t)fr[1] << 8) | fr[0];

    } else if(sub_register == CIR_PWR_PP_AMPL3) {

        rx_fqual_f->cir_pwr = ((uint16_t)fr[3] << 8) | fr[2];
        rx_fqual_f->fp_ampl3 = ((uint16_t)fr[1] << 8) | fr[0];
    }
}

void rx_ttcki_formatter(spi_frame fr, void *format, const size_t sub_register) {

    rx_ttcki_value* rx_ttcki = (rx_ttcki_value*) format;
    *rx_ttcki = (((uint32_t)fr[3]) << 24) | (((uint32_t)fr[2]) << 16) | (((uint32_t)fr[1]) << 8) | fr[0];

}

void rx_ttcko_formatter(spi_frame fr, void *format, const size_t sub_register) {

    rx_ttcko_format* rx_ttcko_f = (rx_ttcko_format*) format;

    if(sub_register == RX_TTCKO_OCT_0_TO_3) {
        rx_ttcko_f->rsmpdel = fr[3];
        rx_ttcko_f->rxtofs = (((uint32_t)(fr[2] & 0b00000111)) << 16)  | (((uint16_t) fr[1]) << 8) | fr[0];
    } else if(sub_register == RX_TTCKO_OCT_4) {
        rx_ttcko_f->rcphase = calculate_phase( (fr[0] & 0b01111111) );
    }
}

void rx_time_formatter(spi_frame fr, void *format, const size_t sub_register) {

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

void tx_time_formatter(spi_frame fr, void *format, const size_t sub_register) {

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

void tx_antd_formatter(spi_frame fr, void *format, const size_t sub_register) {

    uint16_t register_value  = ((uint16_t)fr[1]) << 8 | fr[0];

    double* seconds = (double*) format;
    *seconds = calculate_tx_antd(register_value);

}

size_t tx_antd_unformatter(void *format, spi_frame fr, const size_t sub_register) {

    double* seconds = ((double*) format);
    const uint16_t reg_value = tx_antd_calculate_register_val(*seconds);

    fr[1] = (reg_value & 0xFF00) >> 8;
    fr[0] = reg_value & 0xFF;

    return 2;
}

void sys_state_formatter(spi_frame fr, void *format, const size_t sub_register) {

    sys_status_format* sys_status_f = (sys_status_format*) format;

    sys_status_f->pmsc_state = fr[2] & 0b00001111;
    sys_status_f->rx_state = fr[1] & 0b00011111;
    sys_status_f->tx_state = fr[0] & 0b00001111;

}

void ack_resp_t_formatter(spi_frame fr, void *format, const size_t sub_register) {

    ack_resp_t_format* ack_resp_t_f = (ack_resp_t_format*)format;

    ack_resp_t_f->ack_tim = fr[3];

    const uint32_t register_value = ((uint32_t)(fr[2] & 0b00001111) << 16) |
            ((uint16_t)(fr[1]) << 8) | fr[0];
    ack_resp_t_f->w4r_tim = calculate_w4r_tim(register_value);

}

size_t ack_resp_t_unformatter(void *format, spi_frame fr, const size_t sub_register) {

    ack_resp_t_format* ack_resp_t_f = (ack_resp_t_format*)format;

    fr[3] = ack_resp_t_f->ack_tim;

    const uint32_t reg_value = w4r_tim_calculate_register_val(ack_resp_t_f->w4r_tim);

    fr[2] = (reg_value & 0b11110000000000000000) >> 16;
    fr[1] = (reg_value & 0b00001111111100000000) >> 8;
    fr[0] = reg_value & 0b00000000000011111111;

    return 4;
}

void rx_sniff_formatter(spi_frame fr, void *format, const size_t sub_register) {

    rx_sniff_format* rx_sniff_f = (rx_sniff_format*)format;
    rx_sniff_f->sniff_offt = calculate_sniff_offt(fr[1]);

    rx_sniff_f->sniff_ont = fr[0] & 0b00001111;

}

size_t rx_sniff_unformatter(void *format, spi_frame fr, const size_t sub_register) {

    rx_sniff_format* rx_sniff_f = (rx_sniff_format*)format;
    fr[1] = sniff_offt_calculate_register_val(rx_sniff_f->sniff_offt);

    fr[0] = rx_sniff_f->sniff_ont;

    return 2;
}

void tx_power_formatter(spi_frame fr, void *format, const size_t sub_register) {

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

size_t tx_power_unformatter(void *format, spi_frame fr, const size_t sub_register) {

    tx_power_format* tx_power_f = (tx_power_format*)format;

    fr[3] = ((uint8_t)tx_power_f->field_32_24.coarse_da_setting) << 5;
    fr[3] |= tx_power_f->field_32_24.fine_mixer_setting;

    fr[2] = ((uint8_t)tx_power_f->field_23_16.coarse_da_setting) << 5;
    fr[2] |= tx_power_f->field_23_16.fine_mixer_setting;

    fr[1] = ((uint8_t)tx_power_f->field_15_8.coarse_da_setting) << 5;
    fr[1] |= tx_power_f->field_15_8.fine_mixer_setting;

    fr[0] = ((uint8_t)tx_power_f->field_7_0.coarse_da_setting) << 5;
    fr[0] |= tx_power_f->field_7_0.fine_mixer_setting;

    return 4;
}

void chan_ctrl_formatter(spi_frame fr, void *format, const size_t sub_register) {

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

size_t chan_ctrl_unformatter(void *format, spi_frame fr, const size_t sub_register) {

    chan_ctrl_format* chan_ctrl_f = (chan_ctrl_format*)format;

    fr[3] = ((uint8_t)(chan_ctrl_f->tx_pcode & 0b11100)) >> 2;
    fr[3] |= ((uint8_t)chan_ctrl_f->rx_pcode) << 3;

    fr[2] = ((uint8_t)chan_ctrl_f->dwsfd) << 1;
    fr[2] |= ((uint8_t)chan_ctrl_f->rxprf) << 2;
    fr[2] |= ((uint8_t)chan_ctrl_f->tnssfd) << 4;
    fr[2] |= ((uint8_t)chan_ctrl_f->rnssfd) << 5;
    fr[2] |= ((uint8_t)(chan_ctrl_f->tx_pcode & 0b00011)) << 6;

    fr[1] = 0;

    fr[0] = chan_ctrl_f->tx_chan;
    fr[0] |= ((uint8_t)chan_ctrl_f->rx_chan) << 4;

    return 4;
}

void usr_sfd_formatter(spi_frame fr, void *format, const size_t sub_register) {

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

size_t usr_sfd_unformatter(void *format, spi_frame fr, const size_t sub_register) {

    usr_sfd_format* usr_sfd_f = (usr_sfd_format*) format;

    if(sub_register == USR_SFD_OCT_0_TO_3) {

        fr[0] = usr_sfd_f->sfd_length;
        fr[1] = usr_sfd_f->tx_ssfd_magl;
        fr[2] = usr_sfd_f->tx_ssfd_magh;
        fr[3] = usr_sfd_f->tx_ssfd_sgnl;

        return 4;

    } else if(sub_register == USR_SFD_OCT_4_TO_7) {

        fr[0] = usr_sfd_f->tx_ssfd_sgnh;
        fr[1] = usr_sfd_f->rx_ssfd_magl;
        fr[2] = usr_sfd_f->rx_ssfd_magh;
        fr[3] = usr_sfd_f->rx_ssfd_sgnl;

        return 4;

    } else if(sub_register == USR_SFD_OCT_8_TO_11) {

        fr[0] = usr_sfd_f->rx_ssfd_sgnh;
        fr[1] = usr_sfd_f->tx_lsfd_mag0;
        fr[2] = usr_sfd_f->tx_lsfd_mag1;
        fr[3] = usr_sfd_f->tx_lsfd_mag2;

        return 4;

    } else if(sub_register == USR_SFD_OCT_12_TO_15) {

        fr[0] = usr_sfd_f->tx_lsfd_mag3;
        fr[1] = usr_sfd_f->tx_lsfd_mag4;
        fr[2] = usr_sfd_f->tx_lsfd_mag5;
        fr[3] = usr_sfd_f->tx_lsfd_mag6;

        return 4;

    } else if(sub_register == USR_SFD_OCT_16_TO_19) {

        fr[0] = usr_sfd_f->tx_lsfd_mag7;
        fr[1] = usr_sfd_f->tx_lsfd_sgn0;
        fr[2] = usr_sfd_f->tx_lsfd_sgn1;
        fr[3] = usr_sfd_f->tx_lsfd_sgn2 ;

        return 4;

    } else if(sub_register == USR_SFD_OCT_20_TO_23) {

        fr[0] = usr_sfd_f->tx_lsfd_sgn3;
        fr[1] = usr_sfd_f->tx_lsfd_sgn4;
        fr[2] = usr_sfd_f->tx_lsfd_sgn5;
        fr[3] = usr_sfd_f->tx_lsfd_sgn6;

        return 4;

    } else if(sub_register == USR_SFD_OCT_24_TO_27) {

        fr[0] = usr_sfd_f->tx_lsfd_sgn7;
        fr[1] = usr_sfd_f->rx_lsfd_mag0;
        fr[2] = usr_sfd_f->rx_lsfd_mag1;
        fr[3] = usr_sfd_f->rx_lsfd_mag2;

        return 4;

    } else if(sub_register == USR_SFD_OCT_28_TO_31) {

        fr[0] = usr_sfd_f->rx_lsfd_mag3;
        fr[1] = usr_sfd_f->rx_lsfd_mag4;
        fr[2] = usr_sfd_f->rx_lsfd_mag5;
        fr[3] = usr_sfd_f->rx_lsfd_mag6;

        return 4;

    } else if(sub_register == USR_SFD_OCT_32_TO_35) {

        fr[0] = usr_sfd_f->rx_lsfd_mag7;
        fr[1] = usr_sfd_f->rx_lsfd_sgn0;
        fr[2] = usr_sfd_f->rx_lsfd_sgn1;
        fr[3] = usr_sfd_f->rx_lsfd_sgn2;

        return 4;

    } else if(sub_register == USR_SFD_OCT_36_TO_39) {

        fr[0] = usr_sfd_f->rx_lsfd_sgn2;
        fr[1] = usr_sfd_f->rx_lsfd_sgn3;
        fr[2] = usr_sfd_f->rx_lsfd_sgn4;
        fr[3] = usr_sfd_f->rx_lsfd_sgn5;

        return 4;

    } else if(sub_register == USR_SFD_OCT_40_TO_41) {

        fr[0] = usr_sfd_f->rx_lsfd_sgn6;
        fr[1] = usr_sfd_f->rx_lsfd_sgn7;

        return 2;
    }

    return 0;
}

void agc_ctrl_formatter(spi_frame fr, void *format, const size_t sub_register) {

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

size_t agc_ctrl_unformatter(void *format, spi_frame fr, const size_t sub_register) {

    agc_ctrl_format* agc_ctrl_f = (agc_ctrl_format*) format;

    if(sub_register == AGC_CTRL1) {

        fr[0] = agc_ctrl_f->dis_am;

        return 1;

    } else if(sub_register == AGC_TUNE1) {

        fr[0] = agc_ctrl_f->agc_tune1 & 0x00FF;
        fr[1] = (agc_ctrl_f->agc_tune1 & 0xFF00) >> 8;

        return 2;

    } else if(sub_register == AGC_TUNE2) {

        fr[0] = agc_ctrl_f->agc_tune2 & 0x000000FF;
        fr[1] = (agc_ctrl_f->agc_tune2 & 0x0000FF00) >> 8;
        fr[2] = (agc_ctrl_f->agc_tune2 & 0x00FF0000) >> 16;
        fr[3] = (agc_ctrl_f->agc_tune2 & 0xFF000000) >> 24;

        return 4;

    } else if(sub_register == AGC_TUNE3) {

        fr[0] = agc_ctrl_f->agc_tune3 & 0x00FF;
        fr[1] = (agc_ctrl_f->agc_tune3 & 0xFF00) >> 8;

        return 2;

    }

    return 0;
}

void ext_sync_formatter(spi_frame fr, void *format, const size_t sub_register) {

    ext_sync_format* ext_sync_f = (ext_sync_format*) format;

    if(sub_register == EC_CTRL) {

        ext_sync_f->ostrm  = (fr[1] & 0b00001000) >> 3;

        ext_sync_f->wait = ((fr[1] & 0b00000111) << 5) | (fr[0] & 0b11111000) >> 3;

        ext_sync_f->pllldt = (fr[0] & 0b00000100) >> 2;
        ext_sync_f->osrsm = (fr[0] & 0b00000010) >> 1;
        ext_sync_f->ostsm = fr[0] & 0b00000001;

    } else if(sub_register == EC_RXTC) {

        ext_sync_f->rx_ts_est = (((uint32_t)fr[3]) << 24) | (((uint32_t)fr[2]) << 16) | (((uint16_t)fr[1]) << 8) | fr[0];

    } else if(sub_register == EC_GOLP) {

        ext_sync_f->offset_ext = fr[0];
    }

}

size_t ext_sync_unformatter(void *format, spi_frame fr, const size_t sub_register) {

    ext_sync_format* ext_sync_f = (ext_sync_format*) format;

    if(sub_register == EC_CTRL) {

        fr[1] = (((uint8_t)ext_sync_f->ostrm) << 3) | ((ext_sync_f->wait & 0b11100000) >> 5);

        fr[0] = ext_sync_f->ostsm | (((uint8_t)ext_sync_f->osrsm) << 1) | (((uint8_t)ext_sync_f->pllldt) << 2)
                | ((ext_sync_f->wait & 0b00011111) << 3);

        return 2;

    }

    return 0;
}

void acc_mem_formatter(spi_frame fr, void *format, const size_t sub_register) {

    acc_mem_field* acc_mem_f = (acc_mem_field*) format;

    acc_mem_f->imaginary = (((uint16_t)fr[3]) << 8) | fr[2];
    acc_mem_f->real = (((uint16_t)fr[1]) << 8) | fr[0];

}

void gpio_ctrl_formatter(spi_frame fr, void *format, const size_t sub_register) {

    if(sub_register == GPIO_MODE) {

        gpio_mode_ctrl_format* gpio_mode_ctrl_f = (gpio_mode_ctrl_format*)format;

        gpio_mode_ctrl_f->msgp0 = fr[0] >> 6;

        gpio_mode_ctrl_f->msgp1 = fr[1] & 0b00000011;
        gpio_mode_ctrl_f->msgp2 = (fr[1] & 0b00001100) >> 2;
        gpio_mode_ctrl_f->msgp3 = (fr[1] & 0b00110000) >> 4;
        gpio_mode_ctrl_f->msgp4 = (fr[1] & 0b11000000) >> 6;

        gpio_mode_ctrl_f->msgp5 = fr[2] & 0b00000011;
        gpio_mode_ctrl_f->msgp6 = (fr[2] & 0b00001100) >> 2;
        gpio_mode_ctrl_f->msgp7 = (fr[2] & 0b00110000) >> 4;
        gpio_mode_ctrl_f->msgp8 = (fr[2] & 0b11000000) >> 6;

    } else if(sub_register == GPIO_DIR) {

        gpio_direction_ctrl_format* gpio_direction_ctrl_f = (gpio_direction_ctrl_format*)format;

        gpio_direction_ctrl_f->gdp0 = fr[0] & 0b00000001;
        gpio_direction_ctrl_f->gdp1 = (fr[0] & 0b00000010) >> 1;
        gpio_direction_ctrl_f->gdp2 = (fr[0] & 0b00000100) >> 2;
        gpio_direction_ctrl_f->gdp3 = (fr[0] & 0b00001000) >> 3;
        gpio_direction_ctrl_f->gdm0 = (fr[0] & 0b00010000) >> 4;
        gpio_direction_ctrl_f->gdm1 = (fr[0] & 0b00100000) >> 5;
        gpio_direction_ctrl_f->gdm2 = (fr[0] & 0b01000000) >> 6;
        gpio_direction_ctrl_f->gdm3 = (fr[0] & 0b10000000) >> 7;

        gpio_direction_ctrl_f->gdp4 = fr[1] & 0b00000001;
        gpio_direction_ctrl_f->gdp5 = (fr[1] & 0b00000010) >> 1;
        gpio_direction_ctrl_f->gdp6 = (fr[1] & 0b00000100) >> 2;
        gpio_direction_ctrl_f->gdp7 = (fr[1] & 0b00001000) >> 3;
        gpio_direction_ctrl_f->gdm4 = (fr[1] & 0b00010000) >> 4;
        gpio_direction_ctrl_f->gdm5 = (fr[1] & 0b00100000) >> 5;
        gpio_direction_ctrl_f->gdm6 = (fr[1] & 0b01000000) >> 6;
        gpio_direction_ctrl_f->gdm7 = (fr[1] & 0b10000000) >> 7;

        gpio_direction_ctrl_f->gdp8 = fr[2] & 0b00000001;
        gpio_direction_ctrl_f->gdm8 = (fr[2] & 0b00010000) >> 4;

    } else if(sub_register == GPIO_DOUT) {

        gpio_data_output_ctrl_format* gpio_data_output_ctrl_f = (gpio_data_output_ctrl_format*)format;

        gpio_data_output_ctrl_f->gop0 = fr[0] & 0b00000001;
        gpio_data_output_ctrl_f->gop1 = (fr[0] & 0b00000010) >> 1;
        gpio_data_output_ctrl_f->gop2 = (fr[0] & 0b00000100) >> 2;
        gpio_data_output_ctrl_f->gop3 = (fr[0] & 0b00001000) >> 3;
        gpio_data_output_ctrl_f->gom0 = (fr[0] & 0b00010000) >> 4;
        gpio_data_output_ctrl_f->gom1 = (fr[0] & 0b00100000) >> 5;
        gpio_data_output_ctrl_f->gom2 = (fr[0] & 0b01000000) >> 6;
        gpio_data_output_ctrl_f->gom3 = (fr[0] & 0b10000000) >> 7;

        gpio_data_output_ctrl_f->gop4 = fr[1] & 0b00000001;
        gpio_data_output_ctrl_f->gop5 = (fr[1] & 0b00000010) >> 1;
        gpio_data_output_ctrl_f->gop6 = (fr[1] & 0b00000100) >> 2;
        gpio_data_output_ctrl_f->gop7 = (fr[1] & 0b00001000) >> 3;
        gpio_data_output_ctrl_f->gom4 = (fr[1] & 0b00010000) >> 4;
        gpio_data_output_ctrl_f->gom5 = (fr[1] & 0b00100000) >> 5;
        gpio_data_output_ctrl_f->gom6 = (fr[1] & 0b01000000) >> 6;
        gpio_data_output_ctrl_f->gom7 = (fr[1] & 0b10000000) >> 7;

        gpio_data_output_ctrl_f->gop8 = fr[2] & 0b00000001;
        gpio_data_output_ctrl_f->gom8 = (fr[2] & 0b00010000) >> 4;

    } else if(sub_register == GPIO_IRQE) {

        gpio_irq_ctrl_format* gpio_irq_ctrl_f = (gpio_irq_ctrl_format*)format;

        gpio_irq_ctrl_f->girqe0 = fr[0] & 0b00000001;
        gpio_irq_ctrl_f->girqe1 = (fr[0] & 0b00000010) >> 1;
        gpio_irq_ctrl_f->girqe2 = (fr[0] & 0b00000100) >> 2;
        gpio_irq_ctrl_f->girqe3 = (fr[0] & 0b00001000) >> 3;
        gpio_irq_ctrl_f->girqe4 = (fr[0] & 0b00010000) >> 4;
        gpio_irq_ctrl_f->girqe5 = (fr[0] & 0b00100000) >> 5;
        gpio_irq_ctrl_f->girqe6 = (fr[0] & 0b01000000) >> 6;
        gpio_irq_ctrl_f->girqe7 = (fr[0] & 0b10000000) >> 7;

        gpio_irq_ctrl_f->girqe8 = fr[1] & 0b00000001;

    } else if(sub_register == GPIO_ISEN) {

        gpio_irq_sense_ctrl_format* gpio_irq_sense_ctrl_f = (gpio_irq_sense_ctrl_format*)format;

        gpio_irq_sense_ctrl_f->gisen0 = fr[0] & 0b00000001;
        gpio_irq_sense_ctrl_f->gisen1 = (fr[0] & 0b00000010) >> 1;
        gpio_irq_sense_ctrl_f->gisen2 = (fr[0] & 0b00000100) >> 2;
        gpio_irq_sense_ctrl_f->gisen3 = (fr[0] & 0b00001000) >> 3;
        gpio_irq_sense_ctrl_f->gisen4 = (fr[0] & 0b00010000) >> 4;
        gpio_irq_sense_ctrl_f->gisen5 = (fr[0] & 0b00100000) >> 5;
        gpio_irq_sense_ctrl_f->gisen6 = (fr[0] & 0b01000000) >> 6;
        gpio_irq_sense_ctrl_f->gisen7 = (fr[0] & 0b10000000) >> 7;

        gpio_irq_sense_ctrl_f->gisen8 = fr[1] & 0b00000001;

    } else if(sub_register == GPIO_IMODE) {

        gpio_irq_mode_ctrl_format* gpio_irq_mode_ctrl_f = (gpio_irq_mode_ctrl_format*)format;

        gpio_irq_mode_ctrl_f->gimod0 = fr[0] & 0b00000001;
        gpio_irq_mode_ctrl_f->gimod1 = (fr[0] & 0b00000010) >> 1;
        gpio_irq_mode_ctrl_f->gimod2 = (fr[0] & 0b00000100) >> 2;
        gpio_irq_mode_ctrl_f->gimod3 = (fr[0] & 0b00001000) >> 3;
        gpio_irq_mode_ctrl_f->gimod4 = (fr[0] & 0b00010000) >> 4;
        gpio_irq_mode_ctrl_f->gimod5 = (fr[0] & 0b00100000) >> 5;
        gpio_irq_mode_ctrl_f->gimod6 = (fr[0] & 0b01000000) >> 6;
        gpio_irq_mode_ctrl_f->gimod7 = (fr[0] & 0b10000000) >> 7;

        gpio_irq_mode_ctrl_f->gimod8 = fr[1] & 0b00000001;

    } else if(sub_register == GPIO_IBES) {

        gpio_irq_both_edges_mode_format* gpio_irq_both_edges_mode_f = (gpio_irq_both_edges_mode_format*)format;

        gpio_irq_both_edges_mode_f->gibes0 = fr[0] & 0b00000001;
        gpio_irq_both_edges_mode_f->gibes1 = (fr[0] & 0b00000010) >> 1;
        gpio_irq_both_edges_mode_f->gibes2 = (fr[0] & 0b00000100) >> 2;
        gpio_irq_both_edges_mode_f->gibes3 = (fr[0] & 0b00001000) >> 3;
        gpio_irq_both_edges_mode_f->gibes4 = (fr[0] & 0b00010000) >> 4;
        gpio_irq_both_edges_mode_f->gibes5 = (fr[0] & 0b00100000) >> 5;
        gpio_irq_both_edges_mode_f->gibes6 = (fr[0] & 0b01000000) >> 6;
        gpio_irq_both_edges_mode_f->gibes7 = (fr[0] & 0b10000000) >> 7;

        gpio_irq_both_edges_mode_f->gibes8 = fr[1] & 0b00000001;

    } else if(sub_register == GPIO_ICLR) {

        gpio_irq_latch_clear_mode_format* gpio_irq_latch_clear_mode_f = (gpio_irq_latch_clear_mode_format*)format;

        gpio_irq_latch_clear_mode_f->giclr0 = fr[0] & 0b00000001;
        gpio_irq_latch_clear_mode_f->giclr1 = (fr[0] & 0b00000010) >> 1;
        gpio_irq_latch_clear_mode_f->giclr2 = (fr[0] & 0b00000100) >> 2;
        gpio_irq_latch_clear_mode_f->giclr3 = (fr[0] & 0b00001000) >> 3;
        gpio_irq_latch_clear_mode_f->giclr4 = (fr[0] & 0b00010000) >> 4;
        gpio_irq_latch_clear_mode_f->giclr5 = (fr[0] & 0b00100000) >> 5;
        gpio_irq_latch_clear_mode_f->giclr6 = (fr[0] & 0b01000000) >> 6;
        gpio_irq_latch_clear_mode_f->giclr7 = (fr[0] & 0b10000000) >> 7;

        gpio_irq_latch_clear_mode_f->giclr8 = fr[1] & 0b00000001;

    } else if(sub_register == GPIO_IDBE) {

        gpio_irq_de_bounce_mode_format* gpio_irq_de_bounce_mode_f = (gpio_irq_de_bounce_mode_format*)format;

        gpio_irq_de_bounce_mode_f->gidbe0 = fr[0] & 0b00000001;
        gpio_irq_de_bounce_mode_f->gidbe1 = (fr[0] & 0b00000010) >> 1;
        gpio_irq_de_bounce_mode_f->gidbe2 = (fr[0] & 0b00000100) >> 2;
        gpio_irq_de_bounce_mode_f->gidbe3 = (fr[0] & 0b00001000) >> 3;
        gpio_irq_de_bounce_mode_f->gidbe4 = (fr[0] & 0b00010000) >> 4;
        gpio_irq_de_bounce_mode_f->gidbe5 = (fr[0] & 0b00100000) >> 5;
        gpio_irq_de_bounce_mode_f->gidbe6 = (fr[0] & 0b01000000) >> 6;
        gpio_irq_de_bounce_mode_f->gidbe7 = (fr[0] & 0b10000000) >> 7;

        gpio_irq_de_bounce_mode_f->gidbe8 = fr[1] & 0b00000001;

    } else if(sub_register == GPIO_RAW) {

        gpio_raw_state_format* gpio_raw_state_f = (gpio_raw_state_format*)format;

        gpio_raw_state_f->grawp0 = fr[0] & 0b00000001;
        gpio_raw_state_f->grawp1 = (fr[0] & 0b00000010) >> 1;
        gpio_raw_state_f->grawp2 = (fr[0] & 0b00000100) >> 2;
        gpio_raw_state_f->grawp3 = (fr[0] & 0b00001000) >> 3;
        gpio_raw_state_f->grawp4 = (fr[0] & 0b00010000) >> 4;
        gpio_raw_state_f->grawp5 = (fr[0] & 0b00100000) >> 5;
        gpio_raw_state_f->grawp6 = (fr[0] & 0b01000000) >> 6;
        gpio_raw_state_f->grawp7 = (fr[0] & 0b10000000) >> 7;

        gpio_raw_state_f->grawp8 = fr[1] & 0b00000001;
    }

}

size_t gpio_ctrl_unformatter(void *format, spi_frame fr, const size_t sub_register) {

    if(sub_register == GPIO_MODE) {

        gpio_mode_ctrl_format* gpio_mode_ctrl_f = (gpio_mode_ctrl_format*)format;

        fr[0] = gpio_mode_ctrl_f->msgp0;

        fr[1] = (((uint8_t)gpio_mode_ctrl_f->msgp4) << 6) | (((uint8_t)gpio_mode_ctrl_f->msgp3) << 4) |
                (((uint8_t)gpio_mode_ctrl_f->msgp2) << 2) | gpio_mode_ctrl_f->msgp1;

        fr[2] = (((uint8_t)gpio_mode_ctrl_f->msgp8) << 6) | (((uint8_t)gpio_mode_ctrl_f->msgp7) << 4) |
                (((uint8_t)gpio_mode_ctrl_f->msgp6) << 2) | gpio_mode_ctrl_f->msgp5;

        return 3;

    } else if(sub_register == GPIO_DIR) {

        gpio_direction_ctrl_format* gpio_direction_ctrl_f = (gpio_direction_ctrl_format*)format;

        fr[0] = (((uint8_t)gpio_direction_ctrl_f->gdm3) >> 7) |
                (((uint8_t)gpio_direction_ctrl_f->gdm2) >> 6) |
                (((uint8_t)gpio_direction_ctrl_f->gdm1) >> 5) |
                (((uint8_t)gpio_direction_ctrl_f->gdm0) >> 4) |
                (((uint8_t)gpio_direction_ctrl_f->gdp3) >> 3) |
                (((uint8_t)gpio_direction_ctrl_f->gdp2) >> 2) |
                (((uint8_t)gpio_direction_ctrl_f->gdp1) >> 1) |
                gpio_direction_ctrl_f->gdp0;

        fr[1] = (((uint8_t)gpio_direction_ctrl_f->gdm7) >> 7) |
                (((uint8_t)gpio_direction_ctrl_f->gdm6) >> 6) |
                (((uint8_t)gpio_direction_ctrl_f->gdm5) >> 5) |
                (((uint8_t)gpio_direction_ctrl_f->gdm4) >> 4) |
                (((uint8_t)gpio_direction_ctrl_f->gdp7) >> 3) |
                (((uint8_t)gpio_direction_ctrl_f->gdp6) >> 2) |
                (((uint8_t)gpio_direction_ctrl_f->gdp5) >> 1) |
                gpio_direction_ctrl_f->gdp4;

        fr[2] = (((uint8_t)gpio_direction_ctrl_f->gdm8) >> 4) | gpio_direction_ctrl_f->gdp8;

        return 3;

    } else if(sub_register == GPIO_DOUT) {

        gpio_data_output_ctrl_format* gpio_data_output_ctrl_f = (gpio_data_output_ctrl_format*)format;

        fr[0] = (((uint8_t)gpio_data_output_ctrl_f->gom3) >> 7) |
                (((uint8_t)gpio_data_output_ctrl_f->gom2) >> 6) |
                (((uint8_t)gpio_data_output_ctrl_f->gom1) >> 5) |
                (((uint8_t)gpio_data_output_ctrl_f->gom0) >> 4) |
                (((uint8_t)gpio_data_output_ctrl_f->gop3) >> 3) |
                (((uint8_t)gpio_data_output_ctrl_f->gop2) >> 2) |
                (((uint8_t)gpio_data_output_ctrl_f->gop1) >> 1) |
                gpio_data_output_ctrl_f->gop0;

        fr[1] = (((uint8_t)gpio_data_output_ctrl_f->gom7) >> 7) |
                (((uint8_t)gpio_data_output_ctrl_f->gom6) >> 6) |
                (((uint8_t)gpio_data_output_ctrl_f->gom5) >> 5) |
                (((uint8_t)gpio_data_output_ctrl_f->gom4) >> 4) |
                (((uint8_t)gpio_data_output_ctrl_f->gop7) >> 3) |
                (((uint8_t)gpio_data_output_ctrl_f->gop6) >> 2) |
                (((uint8_t)gpio_data_output_ctrl_f->gop5) >> 1) |
                gpio_data_output_ctrl_f->gop4;

        fr[2] = (((uint8_t)gpio_data_output_ctrl_f->gom8) >> 4) | gpio_data_output_ctrl_f->gop8;

        return 3;

    } else if(sub_register == GPIO_IRQE) {

        gpio_irq_ctrl_format* gpio_irq_ctrl_f = (gpio_irq_ctrl_format*)format;

        fr[0] = (((uint8_t)gpio_irq_ctrl_f->girqe7) >> 7) |
                (((uint8_t)gpio_irq_ctrl_f->girqe6) >> 6) |
                (((uint8_t)gpio_irq_ctrl_f->girqe5) >> 5) |
                (((uint8_t)gpio_irq_ctrl_f->girqe4) >> 4) |
                (((uint8_t)gpio_irq_ctrl_f->girqe3) >> 3) |
                (((uint8_t)gpio_irq_ctrl_f->girqe2) >> 2) |
                (((uint8_t)gpio_irq_ctrl_f->girqe1) >> 1) |
                gpio_irq_ctrl_f->girqe0;

        fr[1] = gpio_irq_ctrl_f->girqe8;

        return 2;

    } else if(sub_register == GPIO_ISEN) {

        gpio_irq_sense_ctrl_format* gpio_irq_sense_ctrl_f = (gpio_irq_sense_ctrl_format*)format;

        fr[0] = (((uint8_t)gpio_irq_sense_ctrl_f->gisen7) >> 7) |
                (((uint8_t)gpio_irq_sense_ctrl_f->gisen6) >> 6) |
                (((uint8_t)gpio_irq_sense_ctrl_f->gisen5) >> 5) |
                (((uint8_t)gpio_irq_sense_ctrl_f->gisen4) >> 4) |
                (((uint8_t)gpio_irq_sense_ctrl_f->gisen3) >> 3) |
                (((uint8_t)gpio_irq_sense_ctrl_f->gisen2) >> 2) |
                (((uint8_t)gpio_irq_sense_ctrl_f->gisen1) >> 1) |
                gpio_irq_sense_ctrl_f->gisen0;

        fr[1] = gpio_irq_sense_ctrl_f->gisen8;

        return 2;

    } else if(sub_register == GPIO_IMODE) {

        gpio_irq_mode_ctrl_format* gpio_irq_mode_ctrl_f = (gpio_irq_mode_ctrl_format*)format;

        fr[0] = (((uint8_t)gpio_irq_mode_ctrl_f->gimod7) >> 7) |
                (((uint8_t)gpio_irq_mode_ctrl_f->gimod6) >> 6) |
                (((uint8_t)gpio_irq_mode_ctrl_f->gimod5) >> 5) |
                (((uint8_t)gpio_irq_mode_ctrl_f->gimod4) >> 4) |
                (((uint8_t)gpio_irq_mode_ctrl_f->gimod3) >> 3) |
                (((uint8_t)gpio_irq_mode_ctrl_f->gimod2) >> 2) |
                (((uint8_t)gpio_irq_mode_ctrl_f->gimod1) >> 1) |
                gpio_irq_mode_ctrl_f->gimod0;

        fr[1] = gpio_irq_mode_ctrl_f->gimod8;

        return 2;

    } else if(sub_register == GPIO_IBES) {

        gpio_irq_both_edges_mode_format* gpio_irq_both_edges_mode_f = (gpio_irq_both_edges_mode_format*)format;

        fr[0] = (((uint8_t)gpio_irq_both_edges_mode_f->gibes7) >> 7) |
                (((uint8_t)gpio_irq_both_edges_mode_f->gibes6) >> 6) |
                (((uint8_t)gpio_irq_both_edges_mode_f->gibes5) >> 5) |
                (((uint8_t)gpio_irq_both_edges_mode_f->gibes4) >> 4) |
                (((uint8_t)gpio_irq_both_edges_mode_f->gibes3) >> 3) |
                (((uint8_t)gpio_irq_both_edges_mode_f->gibes2) >> 2) |
                (((uint8_t)gpio_irq_both_edges_mode_f->gibes1) >> 1) |
                gpio_irq_both_edges_mode_f->gibes0;

        fr[1] = gpio_irq_both_edges_mode_f->gibes8;

        return 2;

    } else if(sub_register == GPIO_ICLR) {

        gpio_irq_latch_clear_mode_format* gpio_irq_latch_clear_mode_f = (gpio_irq_latch_clear_mode_format*)format;

        fr[0] = (((uint8_t)gpio_irq_latch_clear_mode_f->giclr7) >> 7) |
                (((uint8_t)gpio_irq_latch_clear_mode_f->giclr6) >> 6) |
                (((uint8_t)gpio_irq_latch_clear_mode_f->giclr5) >> 5) |
                (((uint8_t)gpio_irq_latch_clear_mode_f->giclr4) >> 4) |
                (((uint8_t)gpio_irq_latch_clear_mode_f->giclr3) >> 3) |
                (((uint8_t)gpio_irq_latch_clear_mode_f->giclr2) >> 2) |
                (((uint8_t)gpio_irq_latch_clear_mode_f->giclr1) >> 1) |
                gpio_irq_latch_clear_mode_f->giclr0;

        fr[1] = gpio_irq_latch_clear_mode_f->giclr8;

        return 2;

    } else if(sub_register == GPIO_IDBE) {

        gpio_irq_de_bounce_mode_format* gpio_irq_de_bounce_mode_f = (gpio_irq_de_bounce_mode_format*)format;

        fr[0] = (((uint8_t)gpio_irq_de_bounce_mode_f->gidbe7) >> 7) |
                (((uint8_t)gpio_irq_de_bounce_mode_f->gidbe6) >> 6) |
                (((uint8_t)gpio_irq_de_bounce_mode_f->gidbe5) >> 5) |
                (((uint8_t)gpio_irq_de_bounce_mode_f->gidbe4) >> 4) |
                (((uint8_t)gpio_irq_de_bounce_mode_f->gidbe3) >> 3) |
                (((uint8_t)gpio_irq_de_bounce_mode_f->gidbe2) >> 2) |
                (((uint8_t)gpio_irq_de_bounce_mode_f->gidbe1) >> 1) |
                gpio_irq_de_bounce_mode_f->gidbe0;

        fr[1] = gpio_irq_de_bounce_mode_f->gidbe8;

        return 2;

    }

    return 0;
}

void drx_conf_formatter(spi_frame fr, void *format, const size_t sub_register) {

    drx_conf_format* drx_conf_f = (drx_conf_format*)format;

    if(sub_register == DRX_TUNE0B) {

        drx_conf_f->drx_tune0b = (((uint16_t)fr[1]) << 8) | fr[0];

    } else if(sub_register == DRX_TUNE1A) {

        drx_conf_f->drx_tune1a = (((uint16_t)fr[1]) << 8) | fr[0];

    } else if(sub_register == DRX_TUNE1B) {

        drx_conf_f->drx_tune1b = (((uint16_t)fr[1]) << 8) | fr[0];

    } else if(sub_register == DRX_TUNE2) {

        drx_conf_f->drx_tune2 = (((uint32_t)(fr[3] & 0b00011111)) << 24) | (((uint32_t)fr[2]) << 16) |
                (((uint16_t)fr[1]) << 8) | fr[0];

    } else if(sub_register == DRX_SFDTOC) {

        drx_conf_f->drx_sfdtoc = (((uint16_t)fr[1]) << 8) | fr[0];

    } else if(sub_register == DRX_PRETOC) {

        drx_conf_f->drx_pretoc = (((uint16_t)fr[1]) << 8) | fr[0];

    } else if(sub_register == DRX_TUNE4H) {

        drx_conf_f->drx_tune4h = (((uint16_t)fr[1]) << 8) | fr[0];

    } else if(sub_register == DRX_CAR_INT) {

        drx_conf_f->drx_car_int = (((uint32_t)fr[2]) << 16) | (((uint16_t)fr[1]) << 8) | fr[0];

    } else if(sub_register == RXPACC_NOSAT) {

        drx_conf_f->rxpacc_nosat = (((uint16_t)fr[1]) << 8) | fr[0];
    }

}

size_t drx_conf_unformatter(void *format, spi_frame fr, const size_t sub_register) {

    drx_conf_format* drx_conf_f = (drx_conf_format*)format;

    if(sub_register == DRX_TUNE0B) {

        fr[0] = drx_conf_f->drx_tune0b & 0x00FF;
        fr[1] = (drx_conf_f->drx_tune0b & 0xFF00) >> 8;

        return 2;

    } else if(sub_register == DRX_TUNE1A) {

        fr[0] = drx_conf_f->drx_tune1a & 0x00FF;
        fr[1] = (drx_conf_f->drx_tune1a & 0xFF00) >> 8;

        return 2;

    } else if(sub_register == DRX_TUNE1B) {

        fr[0] = drx_conf_f->drx_tune1b & 0x00FF;
        fr[1] = (drx_conf_f->drx_tune1b & 0xFF00) >> 8;

        return 2;

    } else if(sub_register == DRX_TUNE2) {

        fr[0] = drx_conf_f->drx_tune2 & 0x000000FF;
        fr[1] = (drx_conf_f->drx_tune2 & 0x0000FF00) >> 8;
        fr[2] = (drx_conf_f->drx_tune2 & 0x00FF0000) >> 16;
        fr[3] = (drx_conf_f->drx_tune2 & 0xFF000000) >> 24;

        return 4;

    } else if(sub_register == DRX_SFDTOC) {

        fr[0] = drx_conf_f->drx_sfdtoc & 0x00FF;
        fr[1] = (drx_conf_f->drx_sfdtoc & 0xFF00) >> 8;

        return 2;

    } else if(sub_register == DRX_PRETOC) {

        fr[0] = drx_conf_f->drx_pretoc & 0x00FF;
        fr[1] = (drx_conf_f->drx_pretoc & 0xFF00) >> 8;

        return 2;

    } else if(sub_register == DRX_TUNE4H) {

        fr[0] = drx_conf_f->drx_tune4h & 0x00FF;
        fr[1] = (drx_conf_f->drx_tune4h & 0xFF00) >> 8;

        return 2;

    }

    return 0;
}

void rf_conf_formatter(spi_frame fr, void *format, const size_t sub_register) {

    rf_conf_format* rf_conf_f = (rf_conf_format*) format;

    if(sub_register == SRF_CONF) {

        rf_conf_f->txrxsw = (fr[2] & 0b01100000) >> 5;
        rf_conf_f->ldofen = fr[2] & 0b00011111;

        rf_conf_f->pllfen = (fr[1] & 0b11100000) >> 5;
        rf_conf_f->txfen = fr[1] & 0b00011111;

    } else if(sub_register == RF_RXCTRLH) {

        rf_conf_f->rfrxctrlh = fr[0];

    } else if(sub_register == RF_TXCTRL) {

        rf_conf_f->txmq = (fr[1] & 0b00001110) >> 1;
        rf_conf_f->txmtune = ((fr[1] & 0b00000001) << 3) | ((fr[0] & 0b11100000) >> 5);

    } else if(sub_register == RF_STATUS) {

        rf_conf_f->rfplllock = (fr[0] & 0b00001000) >> 3;
        rf_conf_f->cpllhigh = (fr[0] & 0b00000100) >> 2;
        rf_conf_f->cplllow = (fr[0] & 0b00000010) >> 1;
        rf_conf_f->cplllock = fr[0] & 0b00000001;

    } else if(sub_register == LDOTUNE) {

        rf_conf_f->ldotune = (((uint64_t)fr[4]) << 32) |
                             (((uint32_t)fr[3]) << 24) |
                             (((uint32_t)fr[2]) << 16) |
                             (((uint16_t)fr[1]) << 8) |
                             fr[0];
    }


}

size_t rf_conf_unformatter(void *format, spi_frame fr, const size_t sub_register) {

    rf_conf_format* rf_conf_f = (rf_conf_format*) format;

    if(sub_register == SRF_CONF) {

        fr[0] = 0;
        fr[1] = (((uint8_t)rf_conf_f->pllfen) << 5) | rf_conf_f->txfen;
        fr[2] = (((uint8_t)rf_conf_f->txrxsw) << 5) | rf_conf_f->ldofen;
        fr[3] = 0;

        return 4;

    } else if(sub_register == RF_RXCTRLH) {

        fr[0] = rf_conf_f->rfrxctrlh;

        return 1;

    } else if(sub_register == RF_TXCTRL) {

        fr[0] = ((uint8_t)rf_conf_f->txmtune) << 5;
        fr[1] = 0b00110000 | (((uint8_t)rf_conf_f->txmq) << 1) | ((rf_conf_f->txmtune & 0b1000) >> 3);
        fr[2] = 0b00011110;
        fr[3] = 0;

        return 4;

    } else if(sub_register == LDOTUNE) {

        fr[4] = (rf_conf_f->ldotune & 0x000000FF00000000) >> 32;
        fr[3] = (rf_conf_f->ldotune & 0x00000000FF000000) >> 24;
        fr[2] = (rf_conf_f->ldotune & 0x0000000000FF0000) >> 16;
        fr[1] = (rf_conf_f->ldotune & 0x000000000000FF00) >> 8;
        fr[0] = rf_conf_f->ldotune & 0x00000000000000FF;

        return 5;

    } else if(sub_register == RF_TLD_BIAS) {

        fr[0] = rf_conf_f->rf_tld_bias;

        return 1;

    } else if(sub_register == RF_TLD_ADC_BIAS) {

        fr[0] = rf_conf_f->rf_tld_adc_bias;

        return 1;
    }

    return 0;
}

void tx_cal_formatter(spi_frame fr, void *format, const size_t sub_register) {

    tx_cal_format* tx_cal_f = (tx_cal_format*)format;

    if(sub_register == TC_SARC) {

        tx_cal_f->sar_ctrl = fr[0] & 0b00000001;

    } else if(sub_register == TC_SARL) {

        tx_cal_f->sar_lvbat = fr[0];
        tx_cal_f->sar_ltemp = fr[1];

    } else if(sub_register == TC_SARW) {

        tx_cal_f->sar_wbat = fr[0];
        tx_cal_f->sar_wtemp = fr[1];

    } else if(sub_register == TC_PG_CTRL) {

        tx_cal_f->pg_start = fr[0] & 0b00000001;
        tx_cal_f->pg_tmeas = (fr[0] & 0b00111100) >> 2;

    }  else if(sub_register == TC_PG_STATUS) {

        tx_cal_f->delay_cnt = (((uint16_t)(fr[1] & 0b00001111)) << 8) | fr[0];

    } else if(sub_register == TC_PG_DELAY) {

        tx_cal_f->tc_pgdelay = fr[0];

    } else if(sub_register == TC_PG_TEST) {

        tx_cal_f->tc_pgtest = fr[0];
    }

}

size_t tx_cal_unformatter(void *format, spi_frame fr, const size_t sub_register) {

    tx_cal_format* tx_cal_f = (tx_cal_format*)format;

    if(sub_register == TC_SARC) {

        fr[0] = tx_cal_f->sar_ctrl;
        fr[1] = 0;

        return 2;

    } else if(sub_register == TC_PG_CTRL) {

        fr[0] = (((uint8_t)tx_cal_f->pg_tmeas) << 2) | tx_cal_f->pg_start;

        return 1;

    }  else if(sub_register == TC_PG_DELAY) {

        fr[0] = tx_cal_f->tc_pgdelay;

        return 1;

    } else if(sub_register == TC_PG_TEST) {

        fr[0] = tx_cal_f->tc_pgtest;

        return 1;
    }

    return 0;
}

void fs_ctrl_formatter(spi_frame fr, void *format, const size_t sub_register) {

    fs_ctrl_format* fs_ctrl_f = (fs_ctrl_format*)format;

    if(sub_register == FS_PLLCFG) {

        fs_ctrl_f->fs_pllcfg = (((uint32_t)fr[3]) << 24) | (((uint32_t)fr[2]) << 16)
                | (((uint16_t)fr[1]) << 8) | fr[0];

    } else if(sub_register == FS_PLLTUNE) {

        fs_ctrl_f->fs_plltune = fr[0];

    } else if(sub_register == FS_XTALT) {

        fs_ctrl_f->xtalt = fr[0] & 0b00011111;

    }
}

size_t fs_ctrl_unformatter(void *format, spi_frame fr, const size_t sub_register) {

    fs_ctrl_format* fs_ctrl_f = (fs_ctrl_format*)format;

    if(sub_register == FS_PLLCFG) {

        fr[3] = (fs_ctrl_f->fs_pllcfg & 0xFF000000) >> 24;
        fr[2] = (fs_ctrl_f->fs_pllcfg & 0x00FF0000) >> 16;
        fr[1] = (fs_ctrl_f->fs_pllcfg & 0x0000FF00) >> 8;
        fr[0] = fs_ctrl_f->fs_pllcfg & 0x000000FF;

        return 4;

    } else if(sub_register == FS_PLLTUNE) {

        fr[0] = fs_ctrl_f->fs_plltune;

        return 1;

    } else if(sub_register == FS_XTALT) {

        fr[0] = 0b01100000 | fs_ctrl_f->xtalt;

        return 1;

    }

    return 0;
}

void aon_formatter(spi_frame fr, void *format, const size_t sub_register) {

    aon_format* aon_f = (aon_format*)format;

    if(sub_register == AON_CFG1) {

        aon_f->sleep_cen = fr[0] & 0b00000001;
        aon_f->smxx = (fr[0] & 0b00000010) >> 1;
        aon_f->lposc_cal = (fr[0] & 0b00000100) >> 2;

    } else if(sub_register == AON_CFG0) {

        aon_f->sleep_en = fr[0] & 0b00000001;
        aon_f->wake_pin = (fr[0] & 0b00000010) >> 1;
        aon_f->wake_spi = (fr[0] & 0b00000100) >> 2;
        aon_f->wake_cnt = (fr[0] & 0b00001000) >> 3;
        aon_f->lpdiv_en = (fr[0] & 0b00010000) >> 4;

        aon_f->lpclkdiva = (((uint16_t)fr[1]) << 3) | (fr[0] & 0b11100000) >> 5;
        aon_f->sleep_tim = (((uint16_t)fr[3]) << 8) | fr[2];

    } else if(sub_register == AON_ADDR) {

        aon_f->aon_addr = fr[0];

    } else if(sub_register == AON_RDAT) {

        aon_f->aon_rdat = fr[0];

    } else if(sub_register == AON_CTRL) {

        aon_f->dca_enab = (fr[0] & 0b10000000) >> 7;
        aon_f->dca_read = (fr[0] & 0b00001000) >> 3;
        aon_f->upl_cfg = (fr[0] & 0b00000100) >> 2;
        aon_f->save = (fr[0] & 0b00000010) >> 1;
        aon_f->restore = fr[0] & 0b00000001;

    } else if(sub_register == AON_WCFG) {

        aon_f->onw_l64p = (fr[0] & 0b10000000) >> 7;
        aon_f->onw_ldc = (fr[0] & 0b01000000) >> 6;
        aon_f->onw_leui = (fr[0] & 0b00001000) >> 3;
        aon_f->onw_rx = (fr[0] & 0b00000010) >> 1;
        aon_f->onw_rad = fr[0] & 0b00000001;

        aon_f->onw_lld0 = (fr[1] & 0b00010000) >> 4;
        aon_f->onw_llde = (fr[1] & 0b00001000) >> 3;
        aon_f->pres_sleep = fr[1] & 0b00000001;

    }

}

size_t aon_unformatter(void *format, spi_frame fr, const size_t sub_register) {

    aon_format* aon_f = (aon_format*)format;

    if(sub_register == AON_CFG1) {

        fr[0] = (((uint8_t)aon_f->sleep_cen) << 2) | (((uint8_t)aon_f->smxx) << 1) | aon_f->lposc_cal;
        fr[1] = 0;

        return 2;

    }  else if(sub_register == AON_CFG0) {

        fr[0] =  (((uint8_t)(aon_f->lpclkdiva & 0b00000000111)) << 5) |
                 (((uint8_t)aon_f->lpdiv_en) << 4) |
                 (((uint8_t)aon_f->wake_cnt) << 3) |
                 (((uint8_t)aon_f->wake_spi) << 2) |
                 (((uint8_t)aon_f->wake_pin) << 1) |
                 aon_f->sleep_en;


        fr[1] = (aon_f->lpclkdiva & 0b11111111000) >> 3;

        fr[2] = aon_f->sleep_tim & 0x00FF;

        fr[3] = (aon_f->sleep_tim & 0xFF00) >> 8;

        return 4;

    } else if(sub_register == AON_ADDR) {

        fr[0] = aon_f->aon_addr;

        return 1;

    } else if(sub_register == AON_RDAT) {

        fr[0] = aon_f->aon_rdat;

        return 1;

    } else if(sub_register == AON_CTRL) {

        fr[0] = (((uint8_t)aon_f->dca_enab) << 7) |
                (((uint8_t)aon_f->dca_read) << 3) |
                (((uint8_t)aon_f->upl_cfg) << 2) |
                (((uint8_t)aon_f->save) << 1) | aon_f->restore;

        return 1;

    } else if(sub_register == AON_WCFG) {

        fr[0] = (((uint8_t)aon_f->onw_l64p) << 7) |
                (((uint8_t)aon_f->onw_ldc) << 6) |
                (((uint8_t)aon_f->onw_leui) << 3) |
                (((uint8_t)aon_f->onw_rx) << 1) |
                aon_f->onw_rad;


        fr[1] = (((uint8_t)aon_f->onw_lld0) << 4) |
                (((uint8_t)aon_f->onw_llde) << 3) |
                aon_f->pres_sleep;


        return 2;
    }

    return 0;
}

void otp_if_formatter(spi_frame fr, void *format, const size_t sub_register) {

    otp_if_format* otp_if_f = (otp_if_format*)format;

    if(sub_register == OTP_WDAT) {

        otp_if_f->otp_wdat = (((uint32_t)fr[3]) << 24) | (((uint32_t)fr[2]) << 16)
                | (((uint16_t)fr[1]) << 8) | fr[0];

    } else if(sub_register == OTP_ADDR) {

        otp_if_f->otp_address = (((uint16_t)fr[1]) << 8) | fr[0];

    } else if(sub_register == OTP_CTRL) {

        otp_if_f->otp_rden = fr[0] & 0b00000001;
        otp_if_f->otp_read = (fr[0] & 0b00000010) >> 1;
        otp_if_f->otp_mrwr = (fr[0] & 0b00001000) >> 3;
        otp_if_f->otp_prog = (fr[0] & 0b01000000) >> 6;
        otp_if_f->otp_mr = ((fr[1] & 0b00000111) << 1) | ((fr[0] & 0b10000000) >> 7);
        otp_if_f->lde_load = (fr[1] & 0b10000000) >> 7;

    } else if(sub_register == OTP_STAT) {

        otp_if_f->otp_prgd = fr[0] & 0b00000001;
        otp_if_f->otp_vpok = (fr[0] & 0b00000010) >> 1;

    } else if(sub_register == OTP_RDAT) {

        otp_if_f->otp_rdat = (((uint32_t)fr[3]) << 24) | (((uint32_t)fr[2]) << 16)
                | (((uint16_t)fr[1]) << 8) | fr[0];

    } else if(sub_register == OTP_SRDAT) {

        otp_if_f->otp_srdat = (((uint32_t)fr[3]) << 24) | (((uint32_t)fr[2]) << 16)
                | (((uint16_t)fr[1]) << 8) | fr[0];

    } else if(sub_register == OTP_SF) {

        otp_if_f->ops_kick = fr[0] & 0b00000001;
        otp_if_f->ldo_kick = (fr[0] & 0b00000010) >> 1;
        otp_if_f->ops_sel = (fr[0] & 0b01100000) >> 5;

    }

}

size_t otp_if_unformatter(void *format, spi_frame fr, const size_t sub_register) {

    otp_if_format* otp_if_f = (otp_if_format*)format;

    if(sub_register == OTP_WDAT) {

        fr[0] = otp_if_f->otp_wdat && 0x000000FF;
        fr[1] = (otp_if_f->otp_wdat && 0x0000FF00) >> 8;
        fr[2] = (otp_if_f->otp_wdat && 0x00FF0000) >> 16;
        fr[3] = (otp_if_f->otp_wdat && 0xFF000000) >> 24;

        return 4;

    } else if(sub_register == OTP_ADDR) {

        fr[0] = otp_if_f->otp_address & 0b000011111111;
        fr[1] = (otp_if_f->otp_address & 0b111100000000) >> 8;

        return 2;

    } else if(sub_register == OTP_CTRL) {

        fr[0] = (((uint8_t)(otp_if_f->otp_mr & 0b0001)) << 7) |
                (((uint8_t)otp_if_f->otp_prog) << 6) |
                (((uint8_t)otp_if_f->otp_mrwr) << 3) |
                (((uint8_t)otp_if_f->otp_read) << 1) |
                otp_if_f->otp_rden;

        fr[1] = (((uint8_t)otp_if_f->lde_load) << 7) |
                (((uint8_t)(otp_if_f->otp_mr & 0b1110)) >> 1);


        return 2;

    } else if(sub_register == OTP_STAT) {

        fr[0] = (((uint8_t)otp_if_f->otp_vpok) << 1) |
                otp_if_f->otp_prgd;

        fr[1] = 0;

        return 2;

    } else if(sub_register == OTP_SRDAT) {

        fr[0] = otp_if_f->otp_srdat && 0x000000FF;
        fr[1] = (otp_if_f->otp_srdat && 0x0000FF00) >> 8;
        fr[2] = (otp_if_f->otp_srdat && 0x00FF0000) >> 16;
        fr[3] = (otp_if_f->otp_srdat && 0xFF000000) >> 24;

        return 4;

    } else if(sub_register == OTP_SF) {


        fr[0] = (((uint8_t)otp_if_f->ops_sel) << 5) |
                (((uint8_t)otp_if_f->ldo_kick) << 1) |
                otp_if_f->ops_kick;

        return 1;

    }

    return 0;
}

void lde_if_formatter(spi_frame fr, void *format, const size_t sub_register) {

    lde_if_format* lde_if_f = (lde_if_format*)format;

    if(sub_register == LDE_REPC) {

        lde_if_f->lde_repc = (((uint16_t)fr[1]) << 8) | fr[0];

    } else if(sub_register == LDE_CFG2) {

        lde_if_f->lde_cfg2 = (((uint16_t)fr[1]) << 8) | fr[0];

    } else if(sub_register == LDE_RXANTD) {

        uint16_t register_value = ((uint16_t)fr[1]) << 8 | fr[0];

        lde_if_f->lde_rxantd = calculate_tx_antd(register_value);

    } else if(sub_register == LDE_PPAMPL) {

        lde_if_f->lde_ppampl = (((uint16_t)fr[1]) << 8) | fr[0];

    } else if(sub_register == LDE_PPINDX) {

        lde_if_f->lde_ppindx = (((uint16_t)fr[1]) << 8) | fr[0];

    } else if(sub_register == LDE_CFG1) {

        lde_if_f->ntm = fr[0] & 0b00011111;
        lde_if_f->pmult = (fr[0] & 0b11100000) >> 5;

    } else if(sub_register == LDE_THRESH) {

        lde_if_f->lde_thresh = (((uint16_t)fr[1]) << 8) | fr[0];

    }

}

size_t lde_if_unformatter(void *format, spi_frame fr, const size_t sub_register) {

    lde_if_format* lde_if_f = (lde_if_format*)format;

    if(sub_register == LDE_REPC) {

        fr[0] = lde_if_f->lde_repc && 0xFF;
        fr[1] = (lde_if_f->lde_repc  && 0xFF00) >> 8;

        return 2;

    } else if(sub_register == LDE_CFG2) {

        fr[0] = lde_if_f->lde_cfg2 && 0xFF;
        fr[1] = (lde_if_f->lde_cfg2  && 0xFF00) >> 8;

        return 2;

    } else if(sub_register == LDE_RXANTD) {

        uint16_t reg_value = tx_antd_calculate_register_val(lde_if_f->lde_rxantd);

        fr[1] = (reg_value & 0xFF00) >> 8;
        fr[0] = reg_value & 0xFF;

        return 2;

    } else if(sub_register == LDE_CFG1) {

        fr[0] = (((uint8_t)lde_if_f->pmult) << 5) |
                lde_if_f->ntm;

        return 1;

    }

    return 0;
}

void dig_diag_formatter(spi_frame fr, void *format, const size_t sub_register) {

    dig_diag_format* dig_diag_f = (dig_diag_format*)format;

    if(sub_register == DIAG_TMC) {

        dig_diag_f->tx_pstm = (fr[0] & 0b00010000) >> 4;

    } else if(sub_register == EVC_TPW) {

        dig_diag_f->evc_tpw =  (((uint16_t)(fr[1] & 0b00001111)) << 8) | fr[0];

    } else if(sub_register == EVC_HPW) {

        dig_diag_f->evc_hpw =  (((uint16_t)(fr[1] & 0b00001111)) << 8) | fr[0];

    } else if(sub_register == EVC_TXFS) {

        dig_diag_f->evc_txfs =  (((uint16_t)(fr[1] & 0b00001111)) << 8) | fr[0];

    } else if(sub_register == EVC_FWTO){

        dig_diag_f->evc_fwto =  (((uint16_t)(fr[1] & 0b00001111)) << 8) | fr[0];

    } else if(sub_register == EVC_PTO) {

        dig_diag_f->evc_pto =  (((uint16_t)(fr[1] & 0b00001111)) << 8) | fr[0];

    } else if(sub_register == EVC_STO) {

        dig_diag_f->evc_sto =  (((uint16_t)(fr[1] & 0b00001111)) << 8) | fr[0];

    } else if(sub_register == EVC_OVR) {

        dig_diag_f->evc_ovr =  (((uint16_t)(fr[1] & 0b00001111)) << 8) | fr[0];

    } else if(sub_register == EVC_FFR) {

        dig_diag_f->evc_ffr =  (((uint16_t)(fr[1] & 0b00001111)) << 8) | fr[0];

    } else if(sub_register == EVC_FCE) {

        dig_diag_f->evc_fce =  (((uint16_t)(fr[1] & 0b00001111)) << 8) | fr[0];

    } else if(sub_register == EVC_FCG) {

        dig_diag_f->evc_fcg =  (((uint16_t)(fr[1] & 0b00001111)) << 8) | fr[0];

    } else if(sub_register == EVC_RSE) {

        dig_diag_f->evc_rse =  (((uint16_t)(fr[1] & 0b00001111)) << 8) | fr[0];

    } else if(sub_register == EVC_PHE) {

        dig_diag_f->evc_phe =  (((uint16_t)(fr[1] & 0b00001111)) << 8) | fr[0];

    } else if(sub_register == EVC_CTRL) {

        dig_diag_f->evc_en = fr[0] & 0b00000001;

        dig_diag_f->evc_clr = (fr[0] & 0b00000010) >> 1;

    }

}

size_t dig_diag_unformatter(void *format, spi_frame fr, const size_t sub_register) {

    dig_diag_format* dig_diag_f = (dig_diag_format*)format;

    if(sub_register == DIAG_TMC) {

        fr[0] = ((uint8_t)dig_diag_f->tx_pstm) << 4;
        fr[1] = 0;

        return 2;

    } else if(sub_register == EVC_CTRL) {


        fr[0] = (((uint8_t)dig_diag_f->evc_clr) << 1) | dig_diag_f->evc_en;

        fr[1] = 0;
        fr[2] = 0;
        fr[3] = 0;

        return 4;
    }

    return 0;
}

void pmsc_formatter(spi_frame fr, void *format, const size_t sub_register) {

    pmsc_format* pmsc_f = (pmsc_format*)format;

    if(sub_register == PMSC_LEDC) {

        pmsc_f->blink_tim = calculate_blink_time(fr[0]);
        pmsc_f->blnk_en = fr[1] & 0b00000001;
        pmsc_f->blnk_now = fr[2] & 0b00001111;

    } else if(sub_register == PMSC_TXFSEQ) {

        pmsc_f->txfseq =  (((uint16_t)fr[1]) << 8) | fr[0];

    } else if(sub_register == PMSC_SNOZT) {

        pmsc_f->snoz_tim =  calculate_snoz_time(fr[0]);

    } else if(sub_register == PMSC_CTRL1) {

        pmsc_f->arx2init = (fr[0] & 0b00000010) >> 1;

        pmsc_f->pktseq = ((fr[1] & 0b00000111) << 5) | ((fr[0] & 0b11111000) >> 3);

        pmsc_f->atxslp = (fr[1] & 0b00001000) >> 3;
        pmsc_f->arxslp = (fr[1] & 0b00010000) >> 4;
        pmsc_f->snoze = (fr[1] & 0b00100000) >> 5;
        pmsc_f->snozr = (fr[1] & 0b01000000) >> 6;
        pmsc_f->pllsyn = (fr[1] & 0b10000000) >> 7;

        pmsc_f->lderune = (fr[2] & 0b00000010) >> 1;

        pmsc_f->khzclkdiv = (fr[3] & 0b11111100) >> 2;

    } else if(sub_register == PMSC_CTRL0) {

        pmsc_f->sysclks = fr[0] & 0b00000011;
        pmsc_f->rxclks = (fr[0] & 0b00001100) >> 2;
        pmsc_f->txclks = (fr[0] & 0b00110000) >> 4;
        pmsc_f->face = (fr[0] & 0b01000000) >> 6;

        pmsc_f->adcce = (fr[1] & 0b00000100) >> 2;
        pmsc_f->amce = (fr[1] & 0b10000000) >> 7;

        pmsc_f->gpce = fr[2] & 0b00000001;
        pmsc_f->gprn = (fr[2] & 0b00000010) >> 1;
        pmsc_f->gpdce = (fr[2] & 0b00000100) >> 2;
        pmsc_f->gpdrn = (fr[2] & 0b00001000) >> 3;
        pmsc_f->khzclken = (fr[2] & 0b10000000) >> 7;

        pmsc_f->pll2_seq_en = fr[3] & 0b00000001;
        pmsc_f->softreset = (fr[3] & 0b11110000) >> 4;
    }

}

size_t pmsc_unformatter(void *format, spi_frame fr, const size_t sub_register) {

    pmsc_format* pmsc_f = (pmsc_format*)format;

    if(sub_register == PMSC_LEDC) {

        fr[0] = blink_time_calculate_seconds(pmsc_f->blink_tim);

        fr[1] = pmsc_f->blnk_en;

        fr[2] = pmsc_f->blnk_now;

        fr[3] = 0;

        return 4;

    } else if(sub_register == PMSC_TXFSEQ) {

        fr[0] = pmsc_f->txfseq & 0x00FF;

        fr[1] = (pmsc_f->txfseq & 0xFF00) >> 8;

        return 2;

    } else if(sub_register == PMSC_SNOZT) {

        fr[0] = snoz_time_calculate_seconds(pmsc_f->snoz_tim);

        return 1;

    } else if(sub_register == PMSC_CTRL1) {

        fr[0] = ((pmsc_f->pktseq & 0b00011111) << 3) |
                (((uint8_t)pmsc_f->arx2init) << 1);

        fr[1] = (((uint8_t)pmsc_f->pllsyn) << 7) |
                (((uint8_t)pmsc_f->snozr) << 6) |
                (((uint8_t)pmsc_f->snoze) << 5) |
                (((uint8_t)pmsc_f->arxslp) << 4) |
                (((uint8_t)pmsc_f->atxslp) << 3) |
                ((pmsc_f->pktseq & 0b11100000) >> 5);

        fr[2] = (((uint8_t)pmsc_f->lderune) << 1);

        fr[3] = (((uint8_t)pmsc_f->khzclkdiv) << 2) |
                0b00000001;

        return 4;

    } else if(sub_register == PMSC_CTRL0) {

        fr[0] = (((uint8_t)pmsc_f->face) << 6) |
                (((uint8_t)pmsc_f->txclks) << 4) |
                (((uint8_t)pmsc_f->rxclks) << 2) |
                pmsc_f->sysclks;

        fr[1] = (((uint8_t)pmsc_f->amce) << 7) |
                (((uint8_t)pmsc_f->adcce) << 2) |
                0b00000010;

        fr[2] = (((uint8_t)pmsc_f->khzclken) << 7) |
                (((uint8_t)pmsc_f->gpdrn) << 3) |
                (((uint8_t)pmsc_f->gpdce) << 2) |
                (((uint8_t)pmsc_f->gprn) << 1) |
                pmsc_f->gpce |
                0b00110000;

        fr[3] = (((uint8_t)pmsc_f->softreset) << 4) |
                pmsc_f->pll2_seq_en;

        return 4;
    }

    return 0;
}
