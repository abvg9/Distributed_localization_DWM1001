#include "ch.h"
#include "dwm1000.h"

int main(void) {

    halInit();
    chSysInit();

    if(dwm_eneable()) {
        read_command(EUI);
        read_command(PANADR);
    }

    while (true) {
        chThdSleepMilliseconds(500);
    }
}
