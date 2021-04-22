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

double calculate_clock_offset(const rx_ttcko_format rx_ttcko_f, const rx_ttcki_value rx_ttcki) {
    return rx_ttcko_f.rxtofs/rx_ttcki;
}

double calculate_estimated_signal_power(const rx_fqual_format rxfq, const rx_finfo_format rxfi) {

    double substractor = 0.0;

    if(rxfi.rxprfr == TRPR_MHZ16) {
        substractor = 113.27;
    } else {
        substractor = 121.74;
    }

    /* TODO */
    // Without subtraction the SFD symbol count from from rxpacc.
    return (10.0 * log10(rxfq.cir_pwr * pow(2, 17)/pow(rxfi.rxpacc, 2))) - substractor;
}

double calculate_signal_power(const rx_time_format rxt, const rx_fqual_format rxfq, const rx_finfo_format rxfi) {

    const double numerator = pow(rxt.fp_ampl1, 2) + pow(rxfq.fp_ampl2, 2) + pow(rxfq.fp_ampl3, 2);

    double substractor = 0.0;

    if(rxfi.rxprfr == TRPR_MHZ16) {
        substractor = 113.27;
    } else {
        substractor = 121.74;
    }

    /* TODO */
    // Without subtraction the SFD symbol count from from rxpacc.
    return (10.0 * log10(numerator/pow(rxfi.rxpacc, 2))) - substractor;
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
