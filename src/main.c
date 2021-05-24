#include "ch.h"

int main(void) {

    halInit();
    chSysInit();

    while (true) {
        chThdSleepMilliseconds(500);
    }
}
