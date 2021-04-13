#include "dwm1000.h"

extern const command COMMAND_PANEL[];

void dwm_disable(void) {
    spiStop(&SPID1);
}

bool dwm_eneable(void) {

    SPIConfig spi_cfg = {
            .end_cb = NULL,
            .ssport = IOPORT1,
            .sspad =  SPI_SS,
            .freq = NRF5_SPI_FREQ_2MBPS,
            .sckpad = SPI_SCK,
            .mosipad = SPI_MOSI,
            .misopad = SPI_MISO,
            .lsbfirst = false,
            .mode = 2
    };

    spiStart(&SPID1, &spi_cfg);
    frame rx_buf = read_command(DEV_ID);

    swap_frame(rx_buf, COMMAND_PANEL[DEV_ID].rx_buf_size);

    unsigned int i = 0;
    while(i < COMMAND_PANEL[DEV_ID].rx_buf_size && DEVICE_ID[i] == rx_buf[i]) {
        ++i;
    }

    free(rx_buf);
    rx_buf = NULL;

    return i == COMMAND_PANEL[DEV_ID].rx_buf_size;
}

frame read_command(const register_id ri) {

    if(is_command_valid(ri, true)) {

        frame tx_buf = malloc(sizeof(uint8_t) * COMMAND_PANEL[ri].tx_buf_size);
        tx_buf[0] = ri;

        frame rx_buf = malloc(sizeof(uint8_t) * COMMAND_PANEL[ri].rx_buf_size);

        spiSelect(&SPID1);
        spiSend(&SPID1, COMMAND_PANEL[ri].tx_buf_size, tx_buf);
        spiReceive(&SPID1, COMMAND_PANEL[ri].rx_buf_size, rx_buf);
        spiUnselect(&SPID1);

        /* DEBUG */
        uint8_t debug_offset[COMMAND_PANEL[ri].rx_buf_size];
        for(unsigned int i = 0; i < COMMAND_PANEL[ri].rx_buf_size; ++i) {
            debug_offset[i] = rx_buf[i + COMMAND_PANEL[ri].offset];
        }

        uint8_t debug_no_offset[COMMAND_PANEL[ri].rx_buf_size];
        for(unsigned int i = 0; i < COMMAND_PANEL[ri].rx_buf_size; ++i) {
            debug_no_offset[i] = rx_buf[i];
        }

        free(tx_buf);
        tx_buf = NULL;

        return rx_buf;
    }

    return NULL;
}

bool write_command(const register_id ri) {

    if(is_command_valid(ri, false)) {
        // ri | REG_WRITE_FLAG;
        return true;
    }

    return false;
}

frame swap_frame(frame f, size_t n) {

    unsigned int low;
    unsigned int high;
    uint8_t tmp = 0;
    frame ret = f;

    for(low = 0, high = n - 1; low < high; ++low, --high) {
        tmp = ret[low];
        ret[low] = ret[high];
        ret[high] = tmp;
    }

    return ret;
}
