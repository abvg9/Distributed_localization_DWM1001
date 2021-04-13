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

/* Enable SPI driver to connect with the dwm1000. */
void dw_disable(void);

/* Disable SPI driver to disconnect from the dwm1000. */
bool dw_eneable(void);

void dw_reset(void);

/* API functions. */

/* DEV_ID */
bool get_dev_id(dev_id_format* dev_id_f);

/* EUI */
bool get_eui(eui_format* eui_f);
bool set_eui(eui_format* eui_f);

/* PAN_ADR */
bool get_pan_adr(pan_adr_format* pan_adr_f);
bool set_pan_id(uint16_t* pan_id_f);
bool set_pan_short_adr(uint16_t* short_addr_f);

/* SYS_CFG*/
bool get_sys_cfg(sys_cfg_format* sys_cfg_f);
bool set_sys_cfg(sys_cfg_format* sys_cfg_f);

/* SYS_TIME */
bool get_sys_time(double* seconds);

#endif /* _DWM1000_H_ */
