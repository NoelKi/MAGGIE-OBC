#include "storage/persistent_state.hpp"
#include <EEPROM.h>

static constexpr int     ADDR_MAGIC      = 0;
static constexpr int     ADDR_BOOT_COUNT = 1;   // 4 Byte
static constexpr int     ADDR_STATE      = 5;
static constexpr int     ADDR_STATE_INV  = 6;
static constexpr uint8_t MAGIC           = 0x4D;   // 'M'

void PersistentState::begin() {
    if (EEPROM.read(ADDR_MAGIC) != MAGIC) {
        // Erster Start (Flash geloescht = 0xFF). Zaehler auf 0, Zustand
        // ungueltig lassen - loadState() liefert dann false.
        for (int i = 0; i < 4; i++) EEPROM.update(ADDR_BOOT_COUNT + i, 0);
        EEPROM.update(ADDR_MAGIC, MAGIC);
    }

    uint32_t count = 0;
    for (int i = 0; i < 4; i++) {
        count |= static_cast<uint32_t>(EEPROM.read(ADDR_BOOT_COUNT + i)) << (8 * i);
    }
    boot_count_ = count + 1;

    for (int i = 0; i < 4; i++) {
        EEPROM.update(ADDR_BOOT_COUNT + i, static_cast<uint8_t>(boot_count_ >> (8 * i)));
    }
}

bool PersistentState::loadState(uint8_t& state) const {
    const uint8_t s   = EEPROM.read(ADDR_STATE);
    const uint8_t inv = EEPROM.read(ADDR_STATE_INV);
    if (static_cast<uint8_t>(~s) != inv) return false;
    state = s;
    return true;
}

void PersistentState::saveState(uint8_t state) {
    // Faellt der Strom zwischen den beiden Bytes, passt das Paar nicht mehr
    // zusammen: loadState() meldet ungueltig, der Start erfolgt in PRE_LAUNCH.
    EEPROM.update(ADDR_STATE, state);
    EEPROM.update(ADDR_STATE_INV, static_cast<uint8_t>(~state));
}
