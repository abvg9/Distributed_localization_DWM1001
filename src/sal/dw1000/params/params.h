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
#ifndef _PARAMS_H_
#define _PARAMS_H_

#include "format.h"
#include <stdbool.h>

#define NUM_BR 3
#define NUM_PRF 2
#define NUM_PACS 4
#define NUM_BW 2            // 2 bandwidths are supported.
#define NUM_SFD 2           // Supported number of SFDs - standard = 0, non-standard = 1.
#define NUM_CH 6            // Supported channels are 1, 2, 3, 4, 5, 7.
#define NUM_CH_SUPPORTED 8  // Supported channels are '0', 1, 2, 3, 4, 5, '6', 7.
#define PCODES 25           // Supported preamble codes.

// Constants for specifying preamble acquisition chunk (PAC). Size in symbols.
#define DW_PAC8        0   // PAC 8 (recommended for RX of preamble length 128 and below.
#define DW_PAC16       1   // PAC 16 (recommended for RX of preamble length 256.
#define DW_PAC32       2   // PAC 32 (recommended for RX of preamble length 512.
#define DW_PAC64       3   // PAC 64 (recommended for RX of preamble length 1024 and up.

#define SPEED_OF_LIGHT 299702547 // Speed of light in air, in meters per second.

typedef struct {
    uint32_t lo32;
    uint16_t target[NUM_PRF];
} agc_cfg_struct ;

#define AGC_TUNE2_VAL           0X2502A907UL
#define AGC_TUNE1_16M           0x8870
#define AGC_TUNE1_64M           0x889B
extern const agc_cfg_struct agc_config;

#define DRX_TUNE0b_110K_STD     0x000A
#define DRX_TUNE0b_110K_NSTD    0x0016
#define DRX_TUNE0b_850K_STD     0x0001
#define DRX_TUNE0b_850K_NSTD    0x0006
#define DRX_TUNE0b_6M8_STD      0x0001
#define DRX_TUNE0b_6M8_NSTD     0x0002
// SFD threshold settings for 110k, 850k, 6.8Mb standard and non-standard
extern const uint16_t sftsh[NUM_BR][NUM_SFD];

#define DRX_TUNE1a_PRF16        0x0087
#define DRX_TUNE1a_PRF64        0x008D
extern const uint16_t dtune1[NUM_PRF];

#define DRX_TUNE1b_110K         0x0064
#define DRX_TUNE1b_850K_6M8     0x0020
#define DRX_TUNE1b_6M8_PRE64    0x0010
#define DRX_TUNE4H_PRE64        0x0010
#define DRX_TUNE4H_PRE128PLUS   0x0028

#define DW_SFDTOC_DEF           0x1041  // Default SFD timeout value.

#define FS_PLLCFG_CH1           0x09000407UL    // Operating Channel 1.
#define FS_PLLCFG_CH2           0x08400508UL    // Operating Channel 2.
#define FS_PLLCFG_CH3           0x08401009UL    // Operating Channel 3.
#define FS_PLLCFG_CH4           FS_PLLCFG_CH2   // Operating Channel 4 (same as 2).
#define FS_PLLCFG_CH5           0x0800041DUL    // Operating Channel 5.
#define FS_PLLCFG_CH7           FS_PLLCFG_CH5   // Operating Channel 7 (same as 5).
extern const uint32_t fs_pll_cfg[NUM_CH];

#define FS_PLLTUNE_CH1          0x1E            // Operating Channel 1.
#define FS_PLLTUNE_CH2          0x26            // Operating Channel 2.
#define FS_PLLTUNE_CH3          0x56            // Operating Channel 3.
#define FS_PLLTUNE_CH4          FS_PLLTUNE_CH2  // Operating Channel 4 (same as 2).
#define FS_PLLTUNE_CH5          0xBE            // Operating Channel 5.
#define FS_PLLTUNE_CH7          FS_PLLTUNE_CH5  // Operating Channel 7 (same as 5).
extern const uint8_t fs_pll_tune[NUM_CH];

#define RF_RXCTRLH_NBW          0xD8            // RXCTRLH value for narrow bandwidth channels.
#define RF_RXCTRLH_WBW          0xBC            // RXCTRLH value for wide bandwidth channels.
extern const uint8_t rx_config[NUM_BW];

#define RF_TXCTRL_CH1           0x00005C40UL    // 32-bit value to program to Sub-Register 0x28:0C RF_TXCTRL.
#define RF_TXCTRL_CH2           0x00045CA0UL    // 32-bit value to program to Sub-Register 0x28:0C RF_TXCTRL.
#define RF_TXCTRL_CH3           0x00086CC0UL    // 32-bit value to program to Sub-Register 0x28:0C RF_TXCTRL.
#define RF_TXCTRL_CH4           0x00045C80UL    // 32-bit value to program to Sub-Register 0x28:0C RF_TXCTRL.
#define RF_TXCTRL_CH5           0x001E3FE0UL    // 32-bit value to program to Sub-Register 0x28:0C RF_TXCTRL.
#define RF_TXCTRL_CH7           0x001E7DE0UL    // 32-bit value to program to Sub-Register 0x28:0C RF_TXCTRL.
extern const uint32_t tx_config[NUM_CH];

#define DW_NS_SFD_LEN_110K      64              // Decawave non-standard SFD length for 110 KBPS.
#define DW_NS_SFD_LEN_850K      16              // Decawave non-standard SFD length for 850 KBPS.
#define DW_NS_SFD_LEN_6M8       8               // Decawave non-standard SFD length for 6.8 MBPS.
extern const uint8_t dwnsSFDlen[NUM_BR];        // Length of SFD for each of the bitrates

#define DRX_TUNE2_PRF16_PAC8    0x311A003CUL
#define DRX_TUNE2_PRF16_PAC16   0x331A0052UL
#define DRX_TUNE2_PRF16_PAC32   0x351A009AUL
#define DRX_TUNE2_PRF16_PAC64   0x371A011DUL
#define DRX_TUNE2_PRF64_PAC8    0x313B006BUL
#define DRX_TUNE2_PRF64_PAC16   0x333B00BEUL
#define DRX_TUNE2_PRF64_PAC32   0x353B015EUL
#define DRX_TUNE2_PRF64_PAC64   0x373B0296UL
extern const uint32_t digital_bb_config[NUM_PRF][NUM_PACS];

extern const uint8_t chan_idx[NUM_CH_SUPPORTED];

#define TEMP_COMP_FACTOR_CH2 327 // (INT) (0.0798 * 4096)
#define TEMP_COMP_FACTOR_CH5 607 // (INT) (0.1482 * 4096)

#define MIXER_GAIN_STEP 0.5
#define DA_ATTN_STEP    2.5

#define MIX_DA_FACTOR   (DA_ATTN_STEP/MIXER_GAIN_STEP)

#define PEAK_MULTPLIER  0x6 //3 -> (0x3 * 32) & 0x00E0
#define N_STD_FACTOR    13

#define LDE_PARAM3_16 (0x1607)
#define LDE_PARAM3_64 (0x0607)

#define LDE_REPC_PCODE_1        0x5998
#define LDE_REPC_PCODE_2        0x5998
#define LDE_REPC_PCODE_3        0x51EA
#define LDE_REPC_PCODE_4        0x428E
#define LDE_REPC_PCODE_5        0x451E
#define LDE_REPC_PCODE_6        0x2E14
#define LDE_REPC_PCODE_7        0x8000
#define LDE_REPC_PCODE_8        0x51EA
#define LDE_REPC_PCODE_9        0x28F4
#define LDE_REPC_PCODE_10       0x3332
#define LDE_REPC_PCODE_11       0x3AE0
#define LDE_REPC_PCODE_12       0x3D70
#define LDE_REPC_PCODE_13       0x3AE0
#define LDE_REPC_PCODE_14       0x35C2
#define LDE_REPC_PCODE_15       0x2B84
#define LDE_REPC_PCODE_16       0x35C2
#define LDE_REPC_PCODE_17       0x3332
#define LDE_REPC_PCODE_18       0x35C2
#define LDE_REPC_PCODE_19       0x35C2
#define LDE_REPC_PCODE_20       0x47AE
#define LDE_REPC_PCODE_21       0x3AE0
#define LDE_REPC_PCODE_22       0x3850
#define LDE_REPC_PCODE_23       0x30A2
#define LDE_REPC_PCODE_24       0x3850
extern const uint16_t lde_replicaCoeff[PCODES];

// Multiplication factors to convert frequency offset in Hertz to PPM crystal offset
// NB: also changes sign so a positive value means the local RX clock is running slower than the remote TX device.
#define HERTZ_TO_PPM_MULTIPLIER_CHAN_1 -1.0e6/3494.4e6
#define HERTZ_TO_PPM_MULTIPLIER_CHAN_2 -1.0e6/3993.6e6
#define HERTZ_TO_PPM_MULTIPLIER_CHAN_3 -1.0e6/4492.8e6
#define HERTZ_TO_PPM_MULTIPLIER_CHAN_5 -1.0e6/6489.6e6

// Multiplication factors to convert carrier integrator value to a frequency offset in hertz.
#define FREQ_OFFSET_MULTIPLIER          (998.4e6/2.0/1024.0/131072.0)
#define FREQ_OFFSET_MULTIPLIER_110KB    (998.4e6/2.0/8192.0/131072.0)

// Default antenna delay values for 64 MHz PRF.
/* IMPORTANT NOTE:
 * The sum of the values is the TX to RX antenna delay, this should be experimentally determined by a calibration process. Here we use a hard coded
 * value (expected to be a little low so a positive error will be seen on the resultant distance estimate. For a real production application, each
 * device should have its own antenna delay properly calibrated to get good precision when performing range measurements.
 * To calibrate this values, this API provides two functions calc_signal_power and calc_estimated_signal_power in dwm1001.c source.
 */
#define TX_ANT_DLY 2.5830325*pow(10,-7)
#define RX_ANT_DLY 2.5830325*pow(10,-7)

// In seconds.
#define DEFAULT_API_DELAY_CALC_DIST_RESP_S (0.5)

// In system time units.
#define DEFAULT_API_DELAY_CALC_DIST_RESP_STU                   \
        str_calculate_register_val(DEFAULT_API_DELAY_CALC_DIST_RESP_S)

// Structure for setting device configuration.
typedef struct {
    channel chan;                               // Channel number.
    transmit_receive_pulse_repetition_freq prf; // Pulse repetition frequency.
    preamble_lenght_selection tx_preamb_length; // Preamble length.
    uint8_t rx_PAC;                             // Acquisition Chunk Size (Relates to RX preamble length)
    uint8_t tx_code;                            // TX preamble code
    uint8_t rx_code;                            // RX preamble code
    bool ns_SFD;                                // 0 to use standard SFD, 1 to use non-standard SFD.
    transmit_receive_bit_rate data_rate;        // Data rate.
    phr_mode phr_mode;                          // PHR mode.
    uint16_t sfd_TO;                            // SFD timeout value (in symbols).
} dw_config_t;

// Possibles values of the sleep_mode field.
typedef enum {
    AON_WCFG_ONW_LLDO = 0x1000,  // On Wake-up load the LDO tune value.
    AON_WCFG_ONW_LLDE = 0x0800, // On Wake-up load the LDE microcode.
} wake_up_mode;

// Structure to hold device data.
typedef struct {
    uint32_t part_ID;           // IC Part ID - read during initialization.
    uint32_t lot_ID;            // IC Lot ID - read during initialization.
    uint8_t v_bat;              // IC V bat read during production and stored in OTP (Vmeas @ 3V3).
    uint8_t temp;               // IC V temp read during production and stored in OTP (Tmeas @ 23C).
    uint8_t otp_rev;            // OTP revision number (read during initialization).
    tx_fctrl_format tx_FCTRL;   // Keep TX_FCTRL register config.
    sys_cfg_format sys_CFG_reg; // Local copy of system config register.
    bool dbl_buff_on;           // Double RX buffer mode flag.
    bool wait_4_resp;           // wait4response was set with last TX start command.
    wake_up_mode sleep_mode;    // Used for automatic reloading of LDO tune and microcode at wake-up.
} dw_local_data_t;

#endif // _PARAMS_H_
