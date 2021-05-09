#include "ch.h"
#include "dw1000.h"

static uint8_t tx_msg[] = {0xC5, 0, 'H', 'O', 'L', 'A', 'A', 'B', 'V', 'G', 0, 0};

int main(void) {

    halInit();
    chSysInit();

    double voltage = 0.0;
    double temperature = 0.0;
    bool enable = false;
    bool ret_temp_vbat = false;
    bool ret_send_message = false;

    enable = dw_eneable(DW_LOADNONE | DW_READ_OTP_BAT | DW_READ_OTP_TMP);

    while (true) {

        chThdSleepMilliseconds(500);

        if(enable) {
            ret_temp_vbat = dw_get_voltage_bat_and_temp(&temperature, &voltage);
            ret_send_message = dw_send_message(tx_msg, sizeof(tx_msg), 0, false, DW_START_TX_IMMEDIATE);
        }

    }
}
