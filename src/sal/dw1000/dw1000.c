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

#include "dw1000.h"

static SPIConfig spi_cfg = { .end_cb = NULL, .ssport = IOPORT1, .sspad = SPI_SS,
        .freq = NRF5_SPI_FREQ_2MBPS, .sckpad = SPI_SCK, .mosipad = SPI_MOSI,
        .misopad = SPI_MISO, .lsbfirst = false, .mode = 2};

static bool fast_SPI = true;
static mutex_t SPI_clock_freq_mtx;
static mutex_t IRQ_event_mtx;

static dw_local_data_t dw_local_data;

static dw_config_t dw_conf = {
        .chan = CH2,
        .prf = TRPR_MHZ64,
        .tx_preamb_length = L_1024,
        .rx_PAC = DW_PAC32,
        .tx_code = 9,
        .rx_code = 9,
        .ns_SFD = true,
        .data_rate = KBPS110,
        .phr_mode = STANDARD,
        .sfd_TO = (1025 + 64 - 32) // SFD timeout (preamble length + 1 + SFD length - PAC size).
};

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

bool dw_configure(void) {

    uint16_t reg_16 = lde_replicaCoeff[dw_conf.rx_code];
    uint8_t prf_index = dw_conf.prf - TRPR_MHZ16;
    uint8_t chan = dw_conf.chan;
    bool bw = ((chan == 4) || (chan == 7)) ? true : false; // Select wide or narrow band.
    bool use_DW_ns_SFD = 0;
    bool tnssfd = 0;
    bool rnssfd = 0;

    // For 110 KBPS needs a special setup.
    if(dw_conf.data_rate == KBPS110) {

        dw_local_data.sys_CFG_reg.rxm110k = true;
        // lde_replicaCoeff must be divided by 8.
        reg_16 >>= 3; // lde_replicaCoeff must be divided by 8.

    } else {

        dw_local_data.sys_CFG_reg.rxm110k = false;

    }

    dw_local_data.sys_CFG_reg.phr_mode = dw_conf.phr_mode;

    if(!set_sys_cfg(&dw_local_data.sys_CFG_reg)) {
        return false;
    }

    lde_if_format lde_if_f;
    if(!get_lde_if(&lde_if_f, LDE_REPC)) {
        return false;
    }

    lde_if_f.lde_repc = reg_16;
    // Set the lde_replicaCoeff.
    if(!set_lde_if(&lde_if_f, LDE_REPC)) {
        return false;
    }

    // Config LDE.
    lde_if_f.ntm = N_STD_FACTOR;
    lde_if_f.pmult = PEAK_MULTPLIER;
    if(!set_lde_if(&lde_if_f, LDE_CFG1)) {
        return false;
    }

    if(prf_index) {

        lde_if_f.lde_cfg2 = LDE_PARAM3_64;
        if(!set_lde_if(&lde_if_f, LDE_CFG2)) {
            return false;
        }

    } else {

        lde_if_f.lde_cfg2 = LDE_PARAM3_16;
        if(!set_lde_if(&lde_if_f, LDE_CFG2)) {
            return false;
        }

    }

    // Configure PLL2/RF PLL block CFG/TUNE (for a given channel).
    fs_ctrl_format fs_ctrl_f;
    if(!get_fs_ctrl(&fs_ctrl_f, -1)) {
        return false;
    }

    fs_ctrl_f.fs_pllcfg = fs_pll_cfg[chan_idx[chan]];
    if(!set_fs_ctrl(&fs_ctrl_f, FS_PLLCFG)) {
        return false;
    }

    fs_ctrl_f.fs_plltune = fs_pll_tune[chan_idx[chan]];
    if(!set_fs_ctrl(&fs_ctrl_f, FS_PLLTUNE)) {
        return false;
    }

    // Configure RF RX blocks (for specified channel/bandwidth).
    rf_conf_format rf_conf_f;
    rf_conf_f.rfrxctrlh = rx_config[bw];
    if(!set_rf_conf(&rf_conf_f, RF_RXCTRLH)) {
        return false;
    }

    // Configure RF TX blocks (for specified channel and PRF).

    // Configure RF TX control.
    rf_conf_f.txmq = (tx_config[chan_idx[chan]] & 0b00001110) >> 1;
    rf_conf_f.txmtune = ((tx_config[chan_idx[chan]] & 0b00000001) << 3) | ((tx_config[chan_idx[chan]] & 0b11100000) >> 5);
    if(!set_rf_conf(&rf_conf_f, RF_TXCTRL)) {
        return false;
    }

    // Configure the baseband parameters (for specified PRF, bit rate, PAC, and SFD settings).

    // DTUNE0.
    drx_conf_format drx_conf_f;
    drx_conf_f.drx_tune0b = sftsh[dw_conf.data_rate][dw_conf.ns_SFD];
    if(!set_drx_conf(&drx_conf_f, DRX_TUNE0B)) {
        return false;
    }

    // DTUNE1.
    drx_conf_f.drx_tune1a = dtune1[prf_index];
    if(!set_drx_conf(&drx_conf_f, DRX_TUNE1A)) {
        return false;
    }

    if(dw_conf.data_rate == KBPS110) {
        drx_conf_f.drx_tune1b = DRX_TUNE1b_110K;
        if(!set_drx_conf(&drx_conf_f, DRX_TUNE1B)) {
            return false;
        }
    } else {
        if(dw_conf.tx_preamb_length == L_64) {

            drx_conf_f.drx_tune1b = DRX_TUNE1b_6M8_PRE64;
            if(!set_drx_conf(&drx_conf_f, DRX_TUNE1B)) {
                return false;
            }

            drx_conf_f.drx_tune4h = DRX_TUNE4H_PRE64;
            if(!set_drx_conf(&drx_conf_f, DRX_TUNE4H)) {
                 return false;
            }

        } else {

            drx_conf_f.drx_tune1b = DRX_TUNE1b_850K_6M8;
            if(!set_drx_conf(&drx_conf_f, DRX_TUNE1B)) {
                return false;
            }

            drx_conf_f.drx_tune4h = DRX_TUNE4H_PRE128PLUS;
            if(!set_drx_conf(&drx_conf_f, DRX_TUNE4H)) {
                 return false;
            }

        }
    }

    // DTUNE2.
    drx_conf_f.drx_tune2 = digital_bb_config[prf_index][dw_conf.rx_PAC];
    if(!set_drx_conf(&drx_conf_f, DRX_TUNE2)) {
         return false;
    }

    // DTUNE3 (SFD timeout).

    // Don't allow 0 - SFD timeout will always be enabled.
    if(dw_conf.sfd_TO == 0) {
        dw_conf.sfd_TO = DW_SFDTOC_DEF;
    }

    drx_conf_f.drx_sfdtoc = dw_conf.sfd_TO;
    if(!set_drx_conf(&drx_conf_f, DRX_TUNE2)) {
         return false;
    }

    // Configure AGC parameters.

    agc_ctrl_format agc_ctrl_f;

    agc_ctrl_f.agc_tune2 = agc_config.lo32;
    if(!set_agc_ctrl(&agc_ctrl_f, AGC_TUNE2)) {
         return false;
    }

    agc_ctrl_f.agc_tune1 = agc_config.target[prf_index];
    if(!set_agc_ctrl(&agc_ctrl_f, AGC_TUNE1)) {
         return false;
    }

    // Set (non-standard) user SFD for improved performance,
    if(dw_conf.ns_SFD) {

        // Write non standard (DW) SFD length.
        usr_sfd_format usr_sfd_f;
        if(!get_usr_sfd(&usr_sfd_f, -1)) {
            return false;
        }

        usr_sfd_f.sfd_length = dwnsSFDlen[dw_conf.data_rate];
        if(!set_usr_sfd(&usr_sfd_f, USR_SFD_OCT_0_TO_3)) {
            return false;
        }

        tnssfd = true;
        rnssfd = true;
        use_DW_ns_SFD = true;
    }

    chan_ctrl_format chan_ctrl_f;
    chan_ctrl_f.tx_chan = chan;
    chan_ctrl_f.rx_chan = chan;

    chan_ctrl_f.rxprf = dw_conf.prf;
    chan_ctrl_f.tnssfd = tnssfd;
    chan_ctrl_f.rnssfd = rnssfd;

    chan_ctrl_f.dwsfd = use_DW_ns_SFD;
    chan_ctrl_f.tx_pcode = dw_conf.tx_code;
    chan_ctrl_f.rx_pcode = dw_conf.rx_code;

    if(!get_chan_ctrl(&chan_ctrl_f)) {
        return false;
    }

    // Set up TX Preamble Size, PRF and Data Rate.
    dw_local_data.tx_FCTRL.pe_txpsr = dw_conf.tx_preamb_length;
    dw_local_data.tx_FCTRL.txprf = dw_conf.prf;
    dw_local_data.tx_FCTRL.txbr = dw_conf.data_rate;
    if(!set_tx_fctrl(&dw_local_data.tx_FCTRL, -1)) {
        return false;
    }

    // The SFD transmit pattern is initialized by the DW1000 upon a user TX request, but (due to an IC issue) it is not done for an auto-ACK TX. The
    // SYS_CTRL write below works around this issue, by simultaneously initiating and aborting a transmission, which correctly initializes the SFD
    // after its configuration or reconfiguration.
    sys_ctrl_format sys_ctrl_f;
    if(!get_sys_ctrl(&sys_ctrl_f)) {
        return false;
    }

    sys_ctrl_f.trxoff = true;
    sys_ctrl_f.txstrt = true;

    // Request TX start and TRX off at the same time.
    if(!set_sys_ctrl(&sys_ctrl_f)) {
        return false;
    }

    return true;
}

void dw_disable(void) {
    spiStop(&SPID1);
    dw_power_off;
}

bool dw_eneable(const int config_flags) {

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

    chMtxObjectInit(&SPI_clock_freq_mtx);
    chMtxObjectInit(&IRQ_event_mtx);

    dw_set_slow_spi_rate();

    ret &= dw_initialise(config_flags);

    dw_set_fast_spi_rate();

    ret &= dw_configure();

    return ret;
}

bool dw_get_otp_value(const otp_memory_direction mem_dir, uint32_t* read_value) {

    otp_if_format otp_if_f;
    if(!get_otp_if(&otp_if_f, -1)) {
        return false;
    }

    // Get OTP temp.
    otp_if_f.otp_address = mem_dir;
    if(!set_otp_if(&otp_if_f, OTP_ADDR)) {
        return false;
    }

    // Perform OTP Read - Manual read mode has to be set.
    otp_if_f.otp_read = 1;
    otp_if_f.otp_rden = 1;
    if(!set_otp_if(&otp_if_f, OTP_CTRL)) {
        return false;
    }

    otp_if_f.otp_rden = 0; // OTPREAD is self clearing but OTPRDEN is not.
    if(!set_otp_if(&otp_if_f, OTP_CTRL)) {
        return false;
    }

    // Read data, available 40ns after rising edge of OTP_READ.
    chThdSleepMilliseconds(1); // ChibiOS does not allow to wait 40ns.
    if(!get_otp_if(&otp_if_f, OTP_RDAT)) {
        return false;
    }
    *read_value= otp_if_f.otp_rdat;


    return true;
}

bool dw_get_voltage_bat_and_temp(double* temperature, double* voltage) {

    // Get RF_CONF register.
    rf_conf_format rf_conf_f;
    if(!get_rf_conf(&rf_conf_f, -1)) {
        return false;
    }

    // Enable TLD Bias.
    rf_conf_f.rf_tld_bias = 0x80;
    if(!set_rf_conf(&rf_conf_f, RF_TLD_BIAS)) {
        return false;
    }

    // Enable TLD Bias and ADC Bias
    rf_conf_f.rf_tld_adc_bias = 0x0A;
    if(!set_rf_conf(&rf_conf_f, RF_TLD_ADC_BIAS)) {
        return false;
    }

    // Enable Outputs (only after Biases are up and running)
    rf_conf_f.rf_tld_adc_bias = 0x0F;
    if(!set_rf_conf(&rf_conf_f, RF_TLD_ADC_BIAS)) {
        return false;
    }

    // Get TX_CAL register.
    tx_cal_format tx_cal_f;
    if(!get_tx_cal(&tx_cal_f, -1)) {
        return false;
    }

    otp_if_format otp_if_f;

    if(fast_SPI) {

        // Reading all SAR inputs.
        tx_cal_f.sar_ctrl = 0;
        if(!set_tx_cal(&tx_cal_f, TC_SARC)) {
            return false;
        }

        // Set SAR eneable.
        tx_cal_f.sar_ctrl = 1;
        if(!set_tx_cal(&tx_cal_f, TC_SARC)) {
            return false;
        }

        // If using PLL clocks(and fast SPI rate) then this sleep is needed.
        chThdSleepMilliseconds(1);

        // Get OTP register.
        if(!get_otp_if(&otp_if_f, -1)) {
            return false;
        }

        // Read voltage and temperature.
        if(!get_tx_cal(&tx_cal_f, TC_SARL)) {
            return false;
        }

    } else {

        chMtxLock(&SPI_clock_freq_mtx);

        // Get PMSC register.
        pmsc_format pmsc_f;
        if(!get_pmsc(&pmsc_f, PMSC_CTRL0)) {
            return false;
        }

        // Set system clock to XTI - this is necessary to make sure the values read are reliable.
        pmsc_f.sysclks = SYSCLKS_19_2MHZ;
        if(!set_pmsc(&pmsc_f, PMSC_CTRL0)) {
            return false;
        }

        // Reading all SAR inputs.
        tx_cal_f.sar_ctrl = 0;
        if(!set_tx_cal(&tx_cal_f, TC_SARC)) {
            return false;
        }

        // Set SAR eneable.
        tx_cal_f.sar_ctrl = 1;
        if(!set_tx_cal(&tx_cal_f, TC_SARC)) {
            return false;
        }

        // Get OTP register.
        if(!get_otp_if(&otp_if_f, -1)) {
            return false;
        }

        // Read voltage and temperature.
        if(!get_tx_cal(&tx_cal_f, TC_SARL)) {
            return false;
        }

        // Default clocks (ENABLE_ALL_SEQ)
        pmsc_f.sysclks = SYSCLKS_AUTO;
        if(!set_pmsc(&pmsc_f, PMSC_CTRL0)) {
            return false;
        }

        chMtxUnlock(&SPI_clock_freq_mtx);

    }

    *temperature = calculate_temperature(tx_cal_f.sar_ltemp, dw_local_data.temp);
    *voltage = calculate_voltage(tx_cal_f.sar_lvbat, dw_local_data.v_bat);

    // Clear SAR enable.
    tx_cal_f.sar_ctrl = 0;
    return set_tx_cal(&tx_cal_f, TC_SARC);

}

bool dw_initialise(const int config_flags) {

    pmsc_format pmsc_f;
    uint16_t otp_xtaltrim_and_rev = 0;
    uint32_t ldo_tune = 0;

    // Double buffer mode is off by default.
    dw_local_data.dbl_buff_on = false;

    // Wait for response not active.
    dw_local_data.wait_4_resp = false;

    // Mean sleep mode has not been configured.
    dw_local_data.sleep_mode = 0;

    // Don't reset the device if DW_DW_WAKE_UP bit is set, e.g. when calling this API after wake up.
    if(!(DW_DW_WAKE_UP & config_flags)) {

        // Make sure the device is completely reset before starting initialization.

        chMtxLock(&SPI_clock_freq_mtx);

        // Get PMSC register.
        if(!get_pmsc(&pmsc_f, -1)) {
            return false;
        }

        // Set system clock to XTI.
        pmsc_f.sysclks = SYSCLKS_19_2MHZ;
        if(!set_pmsc(&pmsc_f, PMSC_CTRL0)) {
            return false;
        }

        // Disable PMSC ctrl of RF and RX clk blocks.
        pmsc_f.pktseq = PKTSEQ_DISABLE_PMSC_CONTROL_ANALOG_SUBSYS;
        if(!set_pmsc(&pmsc_f, PMSC_CTRL1)) {
            return false;
        }

        chMtxUnlock(&SPI_clock_freq_mtx);

        aon_format aon_f;
        if(!get_aon(&aon_f, -1)) {
            return false;
        }

        // Clear any AON auto download bits (as reset will trigger AON download).
        aon_f.onw_l64p = false;
        aon_f.onw_ldc = false;
        aon_f.onw_leui = false;
        aon_f.onw_rx = false;
        aon_f.onw_rad = false;
        aon_f.onw_lld0 = false;
        aon_f.onw_llde = false;
        aon_f.pres_sleep = false;

        if(!set_aon(&aon_f, AON_WCFG)) {
            return false;
        }

        // Clear the wake-up configuration.
        aon_f.sleep_en = false;
        aon_f.wake_pin = false;
        aon_f.wake_spi = false;
        aon_f.wake_cnt = false;
        aon_f.lpdiv_en = false;
        aon_f.lpclkdiva = false;
        aon_f.sleep_tim = false;

        if(!set_aon(&aon_f, AON_CFG0)) {
            return false;
        }

        // Uploads always on (AON) data array and configuration.
        aon_f.dca_enab = false;
        aon_f.dca_read = false;
        aon_f.upl_cfg = false;
        aon_f.save = false;
        aon_f.restore = false;

        if(!set_aon(&aon_f, AON_CTRL)) {
            return false;
        }

        aon_f.save = true;

        if(!set_aon(&aon_f, AON_CTRL)) {
            return false;
        }

        // Reset HIF, TX, RX and PMSC (set the reset bits)
        pmsc_f.softreset = 0;
        pmsc_f.pll2_seq_en = 0;

        if(!set_pmsc(&pmsc_f, PMSC_CTRL0)) {
            return false;
        }

        // DW1000 needs a 10us sleep to let clk PLL lock after reset - the PLL will automatically lock after the reset
        // Could also have polled the PLL lock flag, but then the SPI needs to be < 3MHz !! So a simple delay is easier.
        chThdSleepMilliseconds(1); // ChibiOS does not allow to wait 10ns.

        // Clear the reset bits.
        pmsc_f.softreset = 0xF;
        pmsc_f.pll2_seq_en = 0;

        if(!set_pmsc(&pmsc_f, PMSC_CTRL0)) {
            return false;
        }

        dw_local_data.wait_4_resp = false;

    }

    if(!((DW_DW_WAKE_UP & config_flags) && ((DW_READ_OTP_TMP | DW_READ_OTP_BAT
            | DW_READ_OTP_LID | DW_READ_OTP_PID | DW_DW_WUP_RD_OTPREV) & config_flags))) {
        // Set system clock to XTI. This is necessary to make sure the values read by dw_get_otp_value are reliable.
        // When not reading from OTP, clocks don't need to change.
        pmsc_f.sysclks = SYSCLKS_19_2MHZ;
        if(!set_pmsc(&pmsc_f, PMSC_CTRL0)) {
            return false;
        }
    }

    // Configure the CPLL lock detect.
    ext_sync_format ext_sync_f;
    if(!get_ext_sync(&ext_sync_f, -1)) {
        return false;
    }

    ext_sync_f.pllldt = true;
    if(!set_ext_sync(&ext_sync_f, EC_CTRL)) {
        return false;
    }

    // When DW1000 IC is initialized from power up, then the LDO value should be kicked from OTP, otherwise if this API is called after
    // DW1000 IC has been woken up (DW_DW_WAKE_UP bit is set) this can be skipped as LDO would have already been automatically
    // kicked/loaded on wake up
    if(!(DW_DW_WAKE_UP & config_flags)) {

        // Load LDO tune from OTP and kick it if there is a value actually programmed.

        if(!dw_get_otp_value(LDOTUNE_ADDRESS, &ldo_tune)) {

            if((ldo_tune & 0XFF) != 0) {
                // Kick LDO tune.

                // Set load LDO kick bit.
                otp_if_format otp_if_f;
                if(!get_otp_if(&otp_if_f, OTP_SF)) {
                    return false;
                }

                otp_if_f.ldo_kick = true;
                if(!set_otp_if(&otp_if_f, OTP_SF)) {
                    return false;
                }

                // LDO tune must be kicked at wake-up.
                dw_local_data.sleep_mode = AON_WCFG_ONW_LLDO;
            }
        }
    } else {
        //if LDOTUNE reg contains value different from default it means it was kicked from OTP and thus set AON_WCFG_ONW_LLDO.

        rf_conf_format rf_conf_f;
        if(!get_rf_conf(&rf_conf_f, LDOTUNE)) {
            return false;
        }

        if(rf_conf_f.ldotune != 0x88888888UL) {
            dw_local_data.sleep_mode = AON_WCFG_ONW_LLDO;
        }
    }

    if((!(DW_DW_WAKE_UP & config_flags)) || ((DW_DW_WAKE_UP & config_flags) && (DW_DW_WUP_RD_OTPREV & config_flags))) {

        // Read OTP revision number

        // Read 32 bit value, XTAL trim val is in low octet-0 (5 bits)
        uint32_t otp_val;
        if(!dw_get_otp_value(XTRIM_ADDRESS, &otp_val)) {
            return false;
        }
        otp_xtaltrim_and_rev = otp_val & 0xffff;
        // OTP revision is the next byte.
        dw_local_data.otp_rev = (otp_xtaltrim_and_rev >> 8) & 0xff;

    } else {

        // If OTP valuse are not used, if this API is called after DW1000 IC has been woken up
        // (DW_DW_WAKE_UP bit is set), set otprev to 0.
        dw_local_data.otp_rev = 0;

    }

    if(!(DW_DW_WAKE_UP & config_flags)) {

        // XTAL trim value is set in OTP for DW1000 module and EVK/TREK boards but that might not be the case in a custom design.

        // A value of 0 means that the crystal has not been trimmed.
        if ((otp_xtaltrim_and_rev & 0x1F) == 0) {

            // Set to mid-range if no calibration value inside.
            otp_xtaltrim_and_rev = 0x10;
        }

        // Configure XTAL trim.

        // The 3 MSb in this 8-bit register must be kept to 0b011 to avoid any malfunction.
        const uint8_t reg_val = (3 << 5) | (otp_xtaltrim_and_rev & 0x1F);

        fs_ctrl_format fs_ctrl_f;
        if(!get_fs_ctrl(&fs_ctrl_f, FS_XTALT)) {
            return false;
        }

        fs_ctrl_f.xtalt = reg_val;
        if(!set_fs_ctrl(&fs_ctrl_f, FS_XTALT)) {
            return false;
        }

    }

    if(DW_READ_OTP_PID & config_flags) {

        // Load Part from OTP.
        uint32_t part_ID;
        if(!dw_get_otp_value(PARTID_ADDRESS, &part_ID)) {
            return false;
        }
        dw_local_data.part_ID = part_ID;

    } else {
        dw_local_data.part_ID = 0;
    }

    if(DW_READ_OTP_LID & config_flags) {

        // Load Lot ID from OTP.
        uint32_t lot_ID;
        if(!dw_get_otp_value(LOTID_ADDRESS, &lot_ID)) {
            return false;
        }
        dw_local_data.lot_ID = lot_ID;

    } else {
        dw_local_data.lot_ID = 0;
    }

    if(DW_READ_OTP_BAT & config_flags) {

        // Load VBAT from OTP.
        uint32_t v_bat;
        if(!dw_get_otp_value(VBAT_ADDRESS, &v_bat)) {
            return false;
        }
        dw_local_data.v_bat = v_bat & 0xff;

    } else {
        dw_local_data.v_bat = 0;
    }

    if(DW_READ_OTP_TMP & config_flags) {

        // Load TEMP from OTP.
        uint32_t temp;
        if(!dw_get_otp_value(VTEMP_ADDRESS, &temp)) {
            return false;
        }
        dw_local_data.temp = temp & 0xff;

    } else {
        dw_local_data.temp = 0;
    }

    // Load leading edge detect code (LDE/microcode).
    if(!(DW_DW_WAKE_UP & config_flags)) {

        if(DW_LOADUCODE & config_flags) {

            // Set up clocks.
            pmsc_f.sysclks = SYSCLKS_125MHZ;
            pmsc_f.adcce = true;
            if(!set_pmsc(&pmsc_f, PMSC_CTRL0)) {
                return false;
            }

            // Kick off the LDE load.

            // Set load LDE kick bit.
            otp_if_format otp_if_f;
            if(!get_otp_if(&otp_if_f, OTP_CTRL)) {
                return false;
            }

            otp_if_f.lde_load = true;

            if(!set_otp_if(&otp_if_f, OTP_CTRL)) {
                return false;
            }

            // Allow time for code to upload (should take up to 120 us).
            chThdSleepMilliseconds(1); // ChibiOS does not allow to wait 120us.

            // Default clocks (ENABLE_ALL_SEQ)
            pmsc_f.sysclks = SYSCLKS_AUTO;
            if(!set_pmsc(&pmsc_f,PMSC_CTRL0)) {
                return false;
            }

            // Microcode must be loaded at wake-up if loaded on initialization.
            dw_local_data.sleep_mode = AON_WCFG_ONW_LLDE;

        } else {

            // Should disable the LDERUN bit enable if LDE has not been loaded.
            pmsc_f.lderune = 0;
            if(!set_pmsc(&pmsc_f, OTP_CTRL)) {
                return false;
            }
        }
    } else {

        //if DW_DW_WUP_NO_UCODE is set then assume that the UCODE was loaded from ROM (i.e. DW_LOADUCODE was set on power up),
        //thus set AON_WCFG_ONW_LLDE, otherwise don't set the AON_WCFG_ONW_LLDE bit in the sleep_mode configuration
        if((DW_DW_WUP_NO_UCODE & config_flags) == 0) {
            dw_local_data.sleep_mode = true;
        }

    }

    // Enable clocks for sequencing
    pmsc_f.sysclks = SYSCLKS_AUTO;
    if(!set_pmsc(&pmsc_f, PMSC_CTRL0)) {
        return false;
    }

    // The 3 bits in AON CFG1 register must be cleared to ensure proper operation of the DW1000 in DEEPSLEEP mode.
    aon_format aon_f;
    if(!get_aon(&aon_f, AON_CFG1)) {
        return false;
    }

    aon_f.sleep_cen = 0;
    aon_f.smxx = 0;
    aon_f.lposc_cal = 0;

    if(!set_pmsc(&pmsc_f, AON_CFG1)) {
        return false;
    }

    // Read sysconfig register.
    if(!get_sys_cfg(&dw_local_data.sys_CFG_reg)) {
        return false;
    }

    if(!get_tx_fctrl(&dw_local_data.tx_FCTRL, -1)) {
        return false;
    }

    return true;
}

bool dw_send_message(uwb_frame frame, const uint16_t frame_size, uint16_t tx_buffer_offset, bool ranging, uint8_t mode) {

    // Write frame data to DW1000 and prepare transmission.
    if(!set_tx_buffer(frame, frame_size)){
        return false;
    }

    tx_fctrl_format tx_fctrl_f = dw_local_data.tx_FCTRL;

    tx_fctrl_f.tflen = frame_size;
    tx_fctrl_f.txboffs = tx_buffer_offset;
    tx_fctrl_f.tr = ranging;

    if(!set_tx_fctrl(&tx_fctrl_f, -1)) {
        return false;
    }

    // Start transmission.
    sys_ctrl_format sys_ctrl_f;
    if(!get_sys_ctrl(&sys_ctrl_f)) {
        return false;
    }

    if(mode & DW_RESPONSE_EXPECTED) {
        sys_ctrl_f.wait4resp = true;
        dw_local_data.wait_4_resp = true;
    }

    if (mode & DW_START_TX_DELAYED) {

        // Both SYS_CTRL_TXSTRT and SYS_CTRL_TXDLYS to correctly enable TX.
        sys_ctrl_f.txdlys = true;
        sys_ctrl_f.txstrt = true;
        if(!set_sys_ctrl(&sys_ctrl_f)) {
            return false;
        }

        sys_evt_sts_format sys_evt_sts_f;
        if(!get_sys_event_sts(&sys_evt_sts_f, -1)) {
            return false;
        }

        // Transmit delayed send set over half a period away or power up error
        // (there is enough time to send but not to power up individual blocks).
        if (sys_evt_sts_f.txpute != 0 || sys_evt_sts_f.hpdwarn != 0) {

            // If HPDWARN or TXPUTE are set this indicates that the TXDLYS was set too late for the specified DX_TIME.
            // remedial action is to cancel delayed send and report error.
            sys_ctrl_f.trxoff = true;
            if(!(set_sys_ctrl(&sys_ctrl_f))) {
                return false;
            }

            // Failed !
            return false;
        }

    } else {

        sys_ctrl_f.txstrt = true;
        if(!set_sys_ctrl(&sys_ctrl_f)) {
            return false;
        }
    }

    // Set masks of the events related to the sending of messages.
    sys_evt_msk_format sys_evt_msk_f = {0b0, 0b0, 0b0, 0b0, 0b0, 0b0, 0b0, 0b0,
            0b0, 0b0, 0b0, 0b0, 0b0, 0b0, 0b0, 0b0, 0b0, 0b0, 0b0, 0b0, 0b0, 0b0, 0b0, 0b0, 0b0, 0b0, 0b0, 0b0};
    sys_evt_msk_f.mtxfrs = true;

    sys_evt_sts_format sys_evt_sts_f = dw_wait_irq_event(sys_evt_msk_f);

    if(!sys_evt_sts_f.txfrs) {

        // Error in the transmission buffer.
        sys_evt_sts_f.txberr = false;
        set_sys_event_sts(&sys_evt_sts_f, SES_OCT_0_TO_3);

        return false;
    }

    // Clear TX frame sent event.
    sys_evt_sts_f.txfrs = false;
    if(!set_sys_event_sts(&sys_evt_sts_f, SES_OCT_0_TO_3)) {
        return false;
    }

    /* Increment the blink frame sequence number (modulo 256). */
    //tx_msg[BLINK_FRAME_SN_IDX]++;

    return true;
}

bool dw_receive_message(uwb_frame frame, uint8_t mode) {

    if ((mode & DWT_NO_SYNC_PTRS) == 0) {

        // Synchronizes rx buffer pointers need to make sure that the host/IC
        // buffer pointers are aligned before starting RX.

        // Need to make sure that the host/IC buffer pointers are aligned before starting RX.
        sys_evt_sts_format sys_evt_sts_f;
        if(!get_sys_event_sts(&sys_evt_sts_f, -1)) {
            return false;
        }

        if(sys_evt_sts_f.icrbp != sys_evt_sts_f.hsrbp) {

            // We need to swap RX buffer status reg (write one to toggle internally).
            sys_ctrl_format sys_ctrl_f;
            if(!get_sys_ctrl(&sys_ctrl_f)) {
                return false;
            }

            sys_ctrl_f.hrbpt = true;
            if(!set_sys_ctrl(&sys_ctrl_f)) {
                return false;
            }
        }


    }

    sys_ctrl_format sys_ctrl_f;
    if(!get_sys_ctrl(&sys_ctrl_f)) {
        return false;
    }

    sys_ctrl_f.rxenab = true;

    if (mode & DWT_START_RX_DELAYED) {
        sys_ctrl_f.rxdlye = true;
    }

    if(!set_sys_ctrl(&sys_ctrl_f)) {
        return false;
    }

    // Check for errors.
    if (mode & DWT_START_RX_DELAYED) {

        sys_evt_sts_format sys_evt_sts_f;
        if(!get_sys_event_sts(&sys_evt_sts_f, -1)) {
            return false;
        }

        // If delay has passed do immediate RX on unless DWT_IDLE_ON_DLY_ERR is true.
        if (sys_evt_sts_f.hpdwarn) {

            // Turn the delayed receive off.
            turn_off_transceiver();

            // If DWT_IDLE_ON_DLY_ERR not set then re-enable receiver.
            if((mode & DWT_IDLE_ON_DLY_ERR) == 0) {
                sys_ctrl_f.rxenab = true;
                if(!get_sys_ctrl(&sys_ctrl_f)) {
                    return false;
                }
            }

            return false;
        }
    }

    // Set masks of the events related to the receiving of messages.
    sys_evt_msk_format sys_evt_msk_f= {0b0, 0b0, 0b0, 0b0, 0b0, 0b0, 0b0, 0b0,
            0b0, 0b0, 0b0, 0b0, 0b0, 0b0, 0b0, 0b0, 0b0, 0b0, 0b0, 0b0, 0b0, 0b0, 0b0, 0b0, 0b0, 0b0, 0b0, 0b0};
    sys_evt_msk_f.mrxfcg = true;
    sys_evt_msk_f.mrxphe = true;
    sys_evt_msk_f.mrxfce = true;
    sys_evt_msk_f.mrxrfsl = true;
    sys_evt_msk_f.mrxsfdto = true;
    sys_evt_msk_f.maffrej = true;
    sys_evt_msk_f.mldeerr = true;

    sys_evt_sts_format sys_evt_sts_f = dw_wait_irq_event(sys_evt_msk_f);

    if (sys_evt_sts_f.rxfcg) {

        // A frame has been received, copy it to our local buffer.
        rx_finfo_format rx_finfo_f;
        if(!get_rx_finfo(&rx_finfo_f)) {
            return false;
        }

        if (rx_finfo_f.rxflen <= TX_RX_BUFFER_MAX_SIZE) {
            if(!get_rx_buffer(frame, rx_finfo_f.rxflen)) {
                return false;
            }
        }

        // Clear good RX frame event in the DW1000 status register.
        sys_evt_sts_f.rxfcg = false;
        if(!set_sys_event_sts(&sys_evt_sts_f, SES_OCT_0_TO_3)) {
            return false;
        }

    } else {

        // Clear RX error events in the DW1000 status register.
        sys_evt_msk_f.mrxfcg = false;
        sys_evt_msk_f.mrxphe = false;
        sys_evt_msk_f.mrxfce = false;
        sys_evt_msk_f.mrxrfsl = false;
        sys_evt_msk_f.mrxsfdto = false;
        sys_evt_msk_f.maffrej = false;
        sys_evt_msk_f.mldeerr = false;
        if(!set_sys_event_sts(&sys_evt_sts_f, SES_OCT_0_TO_3)) {
            return false;
        }

        return false;
    }

    return true;
}

void dw_reset(void) {
    dw_power_off;
    chThdSleepMicroseconds(10);
    dw_power_on;
}

void dw_set_fast_spi_rate(void) {

    if(!fast_SPI) {
        spi_cfg.freq = NRF5_SPI_FREQ_8MBPS;
        spiStart(&SPID1, &spi_cfg);
        fast_SPI = true;
    }
}

void dw_set_slow_spi_rate(void) {

    if(fast_SPI) {
        spi_cfg.freq = NRF5_SPI_FREQ_2MBPS;
        spiStart(&SPID1, &spi_cfg);
        fast_SPI = false;
    }
}

bool turn_off_transceiver(void) {

    // Read set interrupt mask.
    sys_evt_msk_format mask;
    if(!get_sys_event_msk(&mask)) {
        return false;
    }

    // Need to beware of interrupts occurring in the middle of following read modify write cycle
    // We can disable the radio, but before the status is cleared an interrupt can be set (e.g. the
    // event has just happened before the radio was disabled)
    // thus we need to disable interrupt during this operation.
    chMtxLock(&IRQ_event_mtx);

    // Clear interrupt mask, so we don't get any unwanted events.
    sys_evt_msk_format clear_mask= {0b0, 0b0, 0b0, 0b0, 0b0, 0b0, 0b0, 0b0,
            0b0, 0b0, 0b0, 0b0, 0b0, 0b0, 0b0, 0b0, 0b0, 0b0, 0b0, 0b0, 0b0, 0b0, 0b0, 0b0, 0b0, 0b0, 0b0, 0b0};
    if(!set_sys_event_msk(&clear_mask)) {
        return false;
    }

    // Disable the radio.
    sys_ctrl_format sys_ctrl_f;
    if(!get_sys_ctrl(&sys_ctrl_f)) {
        return false;
    }

    sys_ctrl_f.trxoff = true;
    if(!set_sys_ctrl(&sys_ctrl_f)) {
        return false;
    }

    sys_evt_sts_format sys_evt_sts_f;
    if(!get_sys_event_sts(&sys_evt_sts_f, -1)) {
        return false;
    }

    // Forcing Transceiver off - so we do not want to see any new events that may have happened.
    sys_evt_sts_f.aat = true;
    sys_evt_sts_f.txfrb = true;
    sys_evt_sts_f.txprs = true;
    sys_evt_sts_f.txphs = true;
    sys_evt_sts_f.txfrs = true;
    sys_evt_sts_f.rxphe = true;
    sys_evt_sts_f.rxfce = true;
    sys_evt_sts_f.rxrfsl = true;
    sys_evt_sts_f.rxsfdto = true;
    sys_evt_sts_f.affrej = true;
    sys_evt_sts_f.ldeerr = true;
    sys_evt_sts_f.rxrfto = true;
    sys_evt_sts_f.rxpto = true;
    sys_evt_sts_f.rxdfr = true;
    sys_evt_sts_f.rxfcg = true;
    sys_evt_sts_f.rxprd = true;
    sys_evt_sts_f.rxsfdd = true;
    sys_evt_sts_f.rxphd = true;
    sys_evt_sts_f.ldedone = true;

    if(!set_sys_event_sts(&sys_evt_sts_f, -1)) {
        return false;
    }

    // Synchronizes rx buffer pointers need to make sure that the host/IC
    // buffer pointers are aligned before starting RX.
    if(sys_evt_sts_f.icrbp != sys_evt_sts_f.hsrbp) {

        // We need to swap RX buffer status reg (write one to toggle internally).
        sys_ctrl_format sys_ctrl_f;
        if(!get_sys_ctrl(&sys_ctrl_f)) {
            return false;
        }

        sys_ctrl_f.hrbpt = true;
        if(!set_sys_ctrl(&sys_ctrl_f)) {
            return false;
        }
    }

    // Set interrupt mask to what it was
    if(!set_sys_event_msk(&mask)) {
        return false;
    }

    // Enable/restore interrupts again.
    chMtxUnlock(&IRQ_event_mtx);
    dw_local_data.wait_4_resp = false;

    return true;
}

sys_evt_sts_format dw_wait_irq_event(sys_evt_msk_format sys_evt_msk_f) {

    palEnablePadEvent(IOPORT1, DW_IRQ, PAL_EVENT_MODE_RISING_EDGE);

    sys_evt_msk_format previous_mask;
    sys_evt_msk_format current_mask;
    get_sys_event_msk(&previous_mask);

    // Set mask respecting the previous values.
    current_mask.maffrej = previous_mask.maffrej | sys_evt_msk_f.maffrej;
    current_mask.mtxberr = previous_mask.mtxberr | sys_evt_msk_f.mtxberr;
    current_mask.mhpdwarn = previous_mask.mhpdwarn | sys_evt_msk_f.mhpdwarn;
    current_mask.mrxsfdto = previous_mask.mrxsfdto | sys_evt_msk_f.mrxsfdto;
    current_mask.mcpllll = previous_mask.mcpllll | sys_evt_msk_f.mcpllll;
    current_mask.mrfpllll = previous_mask.mrfpllll | sys_evt_msk_f.mrfpllll;
    current_mask.mslp2init = previous_mask.mslp2init | sys_evt_msk_f.mslp2init;
    current_mask.mgpioirq = previous_mask.mgpioirq | sys_evt_msk_f.mgpioirq;
    current_mask.mrxpto = previous_mask.mrxpto | sys_evt_msk_f.mrxpto;
    current_mask.mrxovrr = previous_mask.mrxovrr | sys_evt_msk_f.mrxovrr;
    current_mask.mldeerr = previous_mask.mldeerr | sys_evt_msk_f.mldeerr;
    current_mask.mrxrfto = previous_mask.mrxrfto | sys_evt_msk_f.mrxrfto;
    current_mask.mrxrfsl = previous_mask.mrxrfsl | sys_evt_msk_f.mrxrfsl;
    current_mask.mrxfce = previous_mask.mrxfce | sys_evt_msk_f.mrxfce;
    current_mask.mrxfcg = previous_mask.mrxfcg | sys_evt_msk_f.mrxfcg;
    current_mask.mrxdfr = previous_mask.mrxdfr | sys_evt_msk_f.mrxdfr;
    current_mask.mrxphe = previous_mask.mrxphe | sys_evt_msk_f.mrxphe;
    current_mask.mrxphd = previous_mask.mrxphd | sys_evt_msk_f.mrxphd;
    current_mask.mldedone = previous_mask.mldedone | sys_evt_msk_f.mldedone;
    current_mask.mrxsfdd = previous_mask.mrxsfdd | sys_evt_msk_f.mrxsfdd;
    current_mask.mrxprd = previous_mask.mrxprd | sys_evt_msk_f.mrxprd;
    current_mask.mrxfce = previous_mask.mrxfce | sys_evt_msk_f.mrxfce;
    current_mask.mrxfcg = previous_mask.mrxfcg | sys_evt_msk_f.mrxfcg;
    current_mask.mrxdfr = previous_mask.mrxdfr | sys_evt_msk_f.mrxdfr;
    current_mask.mrxphe = previous_mask.mrxphe | sys_evt_msk_f.mrxphe;
    current_mask.mrxphd = previous_mask.mrxphd | sys_evt_msk_f.mrxphd;
    current_mask.mldedone = previous_mask.mldedone | sys_evt_msk_f.mldedone;
    current_mask.mrxsfdd = previous_mask.mrxsfdd | sys_evt_msk_f.mrxsfdd;
    current_mask.mrxprd = previous_mask.mrxprd | sys_evt_msk_f.mrxprd;
    current_mask.mtxfrs = previous_mask.mtxfrs | sys_evt_msk_f.mtxfrs;
    current_mask.mtxphs = previous_mask.mtxphs | sys_evt_msk_f.mtxphs;
    current_mask.mtxprs = previous_mask.mtxprs | sys_evt_msk_f.mtxprs;
    current_mask.mtxfrb = previous_mask.mtxfrb | sys_evt_msk_f.mtxfrb;
    current_mask.maat = previous_mask.maat | sys_evt_msk_f.maat;
    current_mask.mesyncr = previous_mask.mesyncr | sys_evt_msk_f.mesyncr;
    current_mask.mcplock = previous_mask.mcplock | sys_evt_msk_f.mcplock;

    set_sys_event_msk(&current_mask);

    bool event_happens = false;
    sys_evt_sts_format sys_evt_sts_f;

    while(!event_happens) {

        if(!palReadPad(IOPORT1, DW_IRQ)) {
            palWaitPadTimeout(IOPORT1, DW_IRQ, TIME_INFINITE);
        }

        get_sys_event_sts(&sys_evt_sts_f, -1);

        event_happens |= sys_evt_msk_f.maffrej & sys_evt_sts_f.affrej;
        event_happens |= sys_evt_msk_f.mtxberr & sys_evt_sts_f.txberr;
        event_happens |= sys_evt_msk_f.mhpdwarn & sys_evt_sts_f.hpdwarn;
        event_happens |= sys_evt_msk_f.mrxsfdto & sys_evt_sts_f.rxsfdto;
        event_happens |= sys_evt_msk_f.mcpllll & sys_evt_sts_f.clkpll_ll;
        event_happens |= sys_evt_msk_f.mrfpllll & sys_evt_sts_f.rfpll_ll;
        event_happens |= sys_evt_msk_f.mslp2init & sys_evt_sts_f.slp2init;
        event_happens |= sys_evt_msk_f.mgpioirq & sys_evt_sts_f.gpioirq;
        event_happens |= sys_evt_msk_f.mrxpto & sys_evt_sts_f.rxpto;
        event_happens |= sys_evt_msk_f.mrxovrr & sys_evt_sts_f.rxovrr;
        event_happens |= sys_evt_msk_f.mldeerr & sys_evt_sts_f.ldeerr;
        event_happens |= sys_evt_msk_f.mrxrfto & sys_evt_sts_f.rxrfto;
        event_happens |= sys_evt_msk_f.mrxrfsl & sys_evt_sts_f.rxrfsl;
        event_happens |= sys_evt_msk_f.mrxfce & sys_evt_sts_f.rxfce;
        event_happens |= sys_evt_msk_f.mrxfcg & sys_evt_sts_f.rxfcg;
        event_happens |= sys_evt_msk_f.mrxdfr & sys_evt_sts_f.rxdfr;
        event_happens |= sys_evt_msk_f.mrxphe & sys_evt_sts_f.rxphe;
        event_happens |= sys_evt_msk_f.mrxphd & sys_evt_sts_f.rxphd;
        event_happens |= sys_evt_msk_f.mldedone & sys_evt_sts_f.ldedone;
        event_happens |= sys_evt_msk_f.mrxsfdd & sys_evt_sts_f.rxsfdd;
        event_happens |= sys_evt_msk_f.mrxprd & sys_evt_sts_f.rxprd;
        event_happens |= sys_evt_msk_f.mrxfce & sys_evt_sts_f.rxfce;
        event_happens |= sys_evt_msk_f.mrxfcg & sys_evt_sts_f.rxfcg;
        event_happens |= sys_evt_msk_f.mrxdfr & sys_evt_sts_f.rxdfr;
        event_happens |= sys_evt_msk_f.mrxphe & sys_evt_sts_f.rxphe;
        event_happens |= sys_evt_msk_f.mrxphd & sys_evt_sts_f.rxphd;
        event_happens |= sys_evt_msk_f.mldedone & sys_evt_sts_f.ldedone;
        event_happens |= sys_evt_msk_f.mrxsfdd & sys_evt_sts_f.rxsfdd;
        event_happens |= sys_evt_msk_f.mrxprd & sys_evt_sts_f.rxprd;
        event_happens |= sys_evt_msk_f.mtxfrs & sys_evt_sts_f.txfrs;
        event_happens |= sys_evt_msk_f.mtxphs & sys_evt_sts_f.txphs;
        event_happens |= sys_evt_msk_f.mtxprs & sys_evt_sts_f.txprs;
        event_happens |= sys_evt_msk_f.mtxfrb & sys_evt_sts_f.txfrb;
        event_happens |= sys_evt_msk_f.maat & sys_evt_sts_f.aat;
        event_happens |= sys_evt_msk_f.mesyncr & sys_evt_sts_f.esyncr;
        event_happens |= sys_evt_msk_f.mcplock & sys_evt_sts_f.cplock;

    }

    // Clean mask.
    set_sys_event_msk(&previous_mask);

    return sys_evt_sts_f;
}
