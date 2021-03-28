#ifndef _DWM1000_H_
#define _DWM1000_H_

#include "hal.h"
#include <stdbool.h>
#include "register.h"

bool dwm_eneable(void);

void dwm_disable(void);

void send_command(const command command, uint8_t rx_buf[]);

void swap_frame(uint8_t frame[], size_t n);

#endif /* _DWM1000_H_ */
