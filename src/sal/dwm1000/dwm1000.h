#ifndef _DWM1000_H_
#define _DWM1000_H_

#include "hal.h"
#include <stdbool.h>
#include "register.h"

/* Frame of the SPI communication with the dwm1000. */
typedef uint8_t* frame;

/* Enable SPI driver to connect with the dwm1000. */
void dwm_disable(void);

/* Disable SPI driver to disconnect from the dwm1000. */
bool dwm_eneable(void);

frame read_command(const register_id ri);

bool write_command(const register_id ri);

/* Reverse an uint8_t array. */
frame swap_frame(frame frame, size_t n);

#endif /* _DWM1000_H_ */
