#include "dwm1000.h"

SPIConfig hs_spi_cfg = {
        .end_cb = NULL,
        .ssport = IOPORT1,
        .sspad =  SPI_SS,
        .freq = NRF5_SPI_FREQ_8MBPS,
        .sckpad = SPI_SCK,
        .mosipad = SPI_MOSI,
        .misopad = SPI_MISO,
        .lsbfirst = false,
        .mode = 2
};

void swap_frame(uint8_t frame[], size_t n) {

    unsigned int low;
    unsigned int high;
    uint8_t tmp = 0;

    for(low = 0, high = n - 1; low < high; ++low, --high) {
        tmp = frame[low];
        frame[low] = frame[high];
        frame[high] = tmp;
    }

}

bool dwm_eneable(void) {

    spiStart(&SPID1, &hs_spi_cfg);

    uint8_t rx_buf[DEV_ID_COMMAND.rx_buf_size];
    send_command(DEV_ID_COMMAND, rx_buf);

    unsigned int i = 0;
    while(i < DEV_ID_COMMAND.rx_buf_size && rx_buf[i] == DEVICE_ID[i]) {
        ++i;
    }

    return i != DEV_ID_COMMAND.rx_buf_size;
}
void dwm_disable(void) {
    spiStop(&SPID1);
}

void send_command(const command command, uint8_t rx_buf[]) {

    uint8_t tx_buf[command.tx_buf_size];
    tx_buf[0] = command.id;

    spiSelect(&SPID1);
    spiSend(&SPID1, command.tx_buf_size, tx_buf);
    spiReceive(&SPID1, command.rx_buf_size, rx_buf);
    spiUnselect(&SPID1);

    swap_frame(rx_buf, 4);
}
