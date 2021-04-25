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

#include "dw1000.h"

static const SPIConfig spi_cfg = { .end_cb = NULL, .ssport = IOPORT1, .sspad = SPI_SS,
        .freq = NRF5_SPI_FREQ_2MBPS, .sckpad = SPI_SCK, .mosipad = SPI_MOSI,
        .misopad = SPI_MISO, .lsbfirst = false, .mode = 2};

double calc_clock_offset(const rx_ttcko_format rx_ttcko_f, const rx_ttcki_value rx_ttcki) {
    return rx_ttcko_f.rxtofs/rx_ttcki;
}

double calc_estimated_signal_power(const rx_fqual_format rx_fqual_f,
        const rx_finfo_format rx_finfo_f, const usr_sfd_format usr_sfd_f) {

    double substractor = 0.0;

    if(rx_finfo_f.rxprfr == TRPR_MHZ16) {
        substractor = 113.27;
    } else {
        substractor = 121.74;
    }

    return (10.0 * log10(rx_fqual_f.cir_pwr * pow(2, 17)/pow(rx_finfo_f.rxpacc - usr_sfd_f.sfd_length, 2))) - substractor;
}

double calc_freq_error(const drx_conf_format drx_conf_f, const rx_finfo_format rx_finfo_f) {

    int n_samples;

    if(rx_finfo_f.rxbr == KBPS110) {
        n_samples = 8192;
    } else {
        n_samples = 1024;
    }

    return (drx_conf_f.drx_car_int * pow(2, -17)) / 2*(n_samples/998.4*pow(10, 6));
}

double calc_noise_energy_level(const agc_ctrl_format agc_ctrl_f, const chan_ctrl_format chan_ctrl_f) {

    double s;

    // It does not matter if we use .rx_chan or .tx_chan,
    // this two fields must be equal to a correct functionality
    // of the radios.
    if(chan_ctrl_f.rx_chan == CH5 || chan_ctrl_f.rx_chan == CH7) {
        s = 1.0;
    } else {
        s = 1.3335;
    }

    return (agc_ctrl_f.edv2 - 40.0) * pow(10, agc_ctrl_f.edg1) * s;
}

double calc_signal_power(const rx_time_format rx_time_f, const rx_fqual_format rx_fqual_f,
        const rx_finfo_format rx_finfo_f, const usr_sfd_format usr_sfd_f) {

    const double numerator = pow(rx_time_f.fp_ampl1, 2) + pow(rx_fqual_f.fp_ampl2, 2) + pow(rx_fqual_f.fp_ampl3, 2);

    double substractor = 0.0;

    if(rx_finfo_f.rxprfr == TRPR_MHZ16) {
        substractor = 113.27;
    } else {
        substractor = 121.74;
    }

    return (10.0 * log10(numerator/pow(rx_finfo_f.rxpacc - usr_sfd_f.sfd_length, 2))) - substractor;
}

void dw_disable(void) {
    spiStop(&SPID1);
    dw_power_off;
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
    dw_power_off;
    chThdSleepMicroseconds(10);
    dw_power_on;
}
