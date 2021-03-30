#include "register.h"

/* Constant ID value of the dwm1000. */
const uint8_t DEVICE_ID[4] = {0xDE, 0xCA, 0x01, 0x30};

#define RESERVED {0U, 0U, RE, 0}

/* Structure through which user can send commands to the dwm1000. */
const command COMMAND_PANEL[] = {
        {1U, 4U, RO, 0}, // DEV_ID
        {2U, 8U, RW, 8}, // EUI
        RESERVED,        // RESERVED_1
        {1U, 4U, RW, 4}  // PANADR

};

bool is_access_permission_valid(const register_id id, const bool read) {

    switch(COMMAND_PANEL[id].ra) {
        case RO:
            return read;
        case WO:
            return !read;
        case RW:
            return true;
        default:
            return false;
    }
}

bool is_command_valid(const register_id id, const bool read) {
    return is_id_valid(id) && is_access_permission_valid(id, read);
}

bool is_id_valid(const register_id id) {
    return id <= MAX_REG_ID_VALUE;
}
