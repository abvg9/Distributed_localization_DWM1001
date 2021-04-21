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

#ifndef _DW1000_H_
#define _DW1000_H_

#include "register.h"

#define DW_POWER_ON  palSetPad(IOPORT1, DW_RST);
#define DW_POWER_OFF palClearPad(IOPORT1, DW_RST);

/**
 * @brief Disables SPI driver and turn off the dwm1000.
 *
 */
void dw_disable(void);

/**
 * @brief Resets the dw1000 and enables SPI driver.
 *
 * @return bool: Size of the spi_frame formed with the given eui_format structure.
 *
 * @note: This function must be called before any other function.
 *
 */
bool dw_eneable(void);

/**
 * @brief Resets the dw1000.
 *
 */
void dw_reset(void);

/**
 * @brief Calculates the clock offset between two dw1000 devices.
 *
 * @param[in] rx_ttcko_f: Structure which contains the receiver time tracking offset.
 * @param[int] rx_ttcki: Value that contains the receiver time tracking interval.
 *
 * @return double: Clock offset in ppm units.
 *
 */
double calculate_clock_offset(rx_ttcko_format rx_ttcko_f, rx_ttcki_value rx_ttcki);

#endif /* _DWM1000_H_ */
