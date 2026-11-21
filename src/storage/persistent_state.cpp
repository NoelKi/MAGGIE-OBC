#include "storage/persistent_state.hpp"
#include <EEPROM.h>

static constexpr int     ADDR_MAGIC      = 0;
static constexpr int     ADDR_BOOT_COUNT = 1;   // 4 Byte
static constexpr int     ADDR_STATE      = 5;
static constexpr int     ADDR_STATE_INV  = 6;
static constexpr int     ADDR_CKPT       = 7;    // 2 Byte
static constexpr int     ADDR_CKPT_INV   = 9;    // 2 Byte
static constexpr int     ADDR_FLAGS      = 11;
static constexpr int     ADDR_FLAGS_INV  = 12;
static constexpr int     ADDR_ARM_ENABLE     = 13;
static constexpr int     ADDR_ARM_ENABLE_INV = 14;
// Schritt der Flug-Trajektorie in ZWEI Faechern (je Wert + Pruefwert, 4 Byte),
// abwechselnd beschrieben: Faellt der Strom mitten im Schreiben, ist nur ein
// Fach kaputt, das andere hoechstens 1 s alt.
static constexpr int     ADDR_ARM_STEP_A     = 15;   // 15..18
static constexpr int     ADDR_ARM_STEP_B     = 19;   // 19..22
static constexpr int     ADDR_HDRM_BLIND     = 23;
static constexpr int     ADDR_HDRM_BLIND_INV = 24;
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

uint16_t PersistentState::loadCheckpoint() const {
    const uint16_t v   = EEPROM.read(ADDR_CKPT)     | (EEPROM.read(ADDR_CKPT + 1) << 8);
    const uint16_t inv = EEPROM.read(ADDR_CKPT_INV) | (EEPROM.read(ADDR_CKPT_INV + 1) << 8);
    return static_cast<uint16_t>(~v) == inv ? v : 0;
}

void PersistentState::saveCheckpoint(uint16_t seconds) {
    const uint16_t inv = static_cast<uint16_t>(~seconds);
    EEPROM.update(ADDR_CKPT,         static_cast<uint8_t>(seconds));
    EEPROM.update(ADDR_CKPT + 1,     static_cast<uint8_t>(seconds >> 8));
    EEPROM.update(ADDR_CKPT_INV,     static_cast<uint8_t>(inv));
    EEPROM.update(ADDR_CKPT_INV + 1, static_cast<uint8_t>(inv >> 8));
}

uint8_t PersistentState::loadFlightFlags() const {
    const uint8_t v   = EEPROM.read(ADDR_FLAGS);
    const uint8_t inv = EEPROM.read(ADDR_FLAGS_INV);
    return static_cast<uint8_t>(~v) == inv ? v : 0;
}

void PersistentState::saveFlightFlags(uint8_t flags) {
    EEPROM.update(ADDR_FLAGS, flags);
    EEPROM.update(ADDR_FLAGS_INV, static_cast<uint8_t>(~flags));
}

bool PersistentState::loadArmFlightEnable() const {
    const uint8_t v   = EEPROM.read(ADDR_ARM_ENABLE);
    const uint8_t inv = EEPROM.read(ADDR_ARM_ENABLE_INV);
    // Nur genau 0x01 mit passendem Pruefbyte zaehlt als "an" - alles andere
    // (nie beschrieben, halb geschrieben) ist "aus".
    return static_cast<uint8_t>(~v) == inv && v == 0x01;
}

uint8_t PersistentState::loadHdrmBlind() const {
    const uint8_t v   = EEPROM.read(ADDR_HDRM_BLIND);
    const uint8_t inv = EEPROM.read(ADDR_HDRM_BLIND_INV);
    return static_cast<uint8_t>(~v) == inv ? v : 0;
}

void PersistentState::saveHdrmBlind(uint8_t mask) {
    EEPROM.update(ADDR_HDRM_BLIND, mask);
    EEPROM.update(ADDR_HDRM_BLIND_INV, static_cast<uint8_t>(~mask));
}

void PersistentState::saveArmFlightEnable(bool enabled) {
    const uint8_t v = enabled ? 0x01 : 0x00;
    EEPROM.update(ADDR_ARM_ENABLE, v);
    EEPROM.update(ADDR_ARM_ENABLE_INV, static_cast<uint8_t>(~v));
}

static bool readStepSlot(int addr, uint16_t& step) {
    const uint16_t v   = EEPROM.read(addr)     | (EEPROM.read(addr + 1) << 8);
    const uint16_t inv = EEPROM.read(addr + 2) | (EEPROM.read(addr + 3) << 8);
    if (static_cast<uint16_t>(~v) != inv) return false;
    step = v;
    return true;
}

static void writeStepSlot(int addr, uint16_t step) {
    const uint16_t inv = static_cast<uint16_t>(~step);
    EEPROM.update(addr,     static_cast<uint8_t>(step));
    EEPROM.update(addr + 1, static_cast<uint8_t>(step >> 8));
    EEPROM.update(addr + 2, static_cast<uint8_t>(inv));
    EEPROM.update(addr + 3, static_cast<uint8_t>(inv >> 8));
}

bool PersistentState::loadArmStep(uint16_t& step) const {
    // Der Schritt waechst nur - von zwei gueltigen Faechern ist das groessere
    // das neuere.
    uint16_t a = 0, b = 0;
    const bool va = readStepSlot(ADDR_ARM_STEP_A, a);
    const bool vb = readStepSlot(ADDR_ARM_STEP_B, b);
    if (!va && !vb) return false;
    step = (va && vb) ? (a > b ? a : b) : (va ? a : b);
    return true;
}

void PersistentState::saveArmStep(uint16_t step) {
    writeStepSlot(arm_step_slot_b_ ? ADDR_ARM_STEP_B : ADDR_ARM_STEP_A, step);
    arm_step_slot_b_ = !arm_step_slot_b_;
}

void PersistentState::resetArmStep() {
    writeStepSlot(ADDR_ARM_STEP_A, 0);
    writeStepSlot(ADDR_ARM_STEP_B, 0);
}
