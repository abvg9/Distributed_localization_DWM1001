#include "ch.h"
#include "dw1000.h"

int main(void) {

    halInit();
    chSysInit();

    if(dwm_eneable()) {
    }

    while (true) {
        chThdSleepMilliseconds(500);
    }
}
