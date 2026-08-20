/**
 * @file softpwm_check.cpp
 * @brief Misst, ob MotorHAL auf Pin 40/41 wirklich ein Signal erzeugt.
 *
 * Pin 40/41 haben keinen FlexPWM-Kanal, deshalb taktet MotorHAL sie per
 * IntervalTimer selbst (siehe motor_hal.cpp, softPwmIsr). Dieser Pfad ist neu
 * und laesst sich nicht am Schreibtisch nachrechnen - hier wird er gemessen.
 *
 * Der Trick: auf dem i.MX RT liefert digitalRead() auch fuer einen als OUTPUT
 * konfigurierten Pin den tatsaechlichen Pad-Zustand zurueck. Der Teensy kann
 * seine eigene PWM also selbst abtasten. Kein Oszilloskop, kein Motorstrom,
 * keine externe Beschaltung noetig - laeuft am blanken Board.
 *
 * Getestet wird die ECHTE Flight-Klasse MotorHAL, nicht ein Nachbau.
 *
 * Erwartung bei setSpeed(120): Pin 41 ~47% (120/256), Pin 40 0%.
 * Kommt 0% oder 100% heraus, taktet die Software-PWM nicht.
 *
 * Bauen und flashen:    pio run -e softpwm -t upload
 * Monitor:              pio device monitor -e softpwm
 * Zurueck zur Firmware: pio run -e teensy41 -t upload
 */

#include <Arduino.h>
#include "hal/motor_hal.hpp"
#include "pin_config.hpp"

static MotorHAL motor(PIN_M1_A, PIN_M1_B, 1);

/// Abtastfenster je Messung - deutlich laenger als eine PWM-Periode (1 ms).
static constexpr uint32_t SAMPLE_MS = 100;

/**
 * Tastverhaeltnis eines Pins durch Rueckabtasten bestimmen.
 * @return Anteil der HIGH-Abtastungen in Prozent.
 */
static float measureDuty(uint8_t pin) {
    uint32_t high = 0, total = 0;
    const uint32_t t0 = millis();
    while (millis() - t0 < SAMPLE_MS) {
        if (digitalRead(pin)) high++;
        total++;
    }
    return total ? (100.0f * static_cast<float>(high) / static_cast<float>(total)) : 0.0f;
}

/// Eine Sollgeschwindigkeit setzen und beide Kanaele nachmessen.
static void check(int16_t speed, float expect_a, float expect_b) {
    motor.setSpeed(speed);
    delay(20);                       // Einschwingen der ersten Periode

    const float duty_a = measureDuty(PIN_M1_A);
    const float duty_b = measureDuty(PIN_M1_B);

    const float err_a = fabsf(duty_a - expect_a);
    const float err_b = fabsf(duty_b - expect_b);
    const bool  ok    = err_a <= 5.0f && err_b <= 5.0f;

    Serial.printf("setSpeed(%+4d):  Pin %u (A) = %5.1f%% (erwartet %5.1f%%)   "
                  "Pin %u (B) = %5.1f%% (erwartet %5.1f%%)   %s\n",
                  speed, PIN_M1_A, duty_a, expect_a,
                  PIN_M1_B, duty_b, expect_b, ok ? "OK" : "<-- ABWEICHUNG");
}

/**
 * Gegenprobe ohne PWM: kann der Pin ueberhaupt treiben?
 *
 * MUSS vor MotorHAL::init() laufen. Danach tickt der IntervalTimer der
 * Software-PWM mit 256 kHz und schreibt bei Duty 0 unablaessig LOW auf beide
 * Kanaele - ein digitalWrite(HIGH) waere nach 3,9 us wieder ueberschrieben und
 * der Test wuerde faelschlich "Pin treibt nicht" melden.
 */
static void checkStaticDrive(uint8_t pin) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, HIGH);
    delay(5);
    const bool reads_high = digitalRead(pin);
    digitalWrite(pin, LOW);
    delay(5);
    const bool reads_low = !digitalRead(pin);

    Serial.printf("Statiktest Pin %u:  HIGH lesbar %s, LOW lesbar %s  %s\n",
                  pin, reads_high ? "ja" : "NEIN", reads_low ? "ja" : "NEIN",
                  (reads_high && reads_low) ? "OK" : "<-- Pin treibt nicht");
}

/// Kompletter Messdurchlauf. Wiederholbar, damit der Monitor ihn nicht
/// verpassen kann, wenn er erst nach dem Boot verbunden wird.
static void runAll() {
    Serial.println();
    Serial.println("=== MotorHAL Software-PWM: Selbstmessung auf Pin 40/41 ===");
    Serial.printf("Kanal A = Pin %u, Kanal B = Pin %u\n", PIN_M1_A, PIN_M1_B);

    // Erst der Statiktest (ohne laufende Software-PWM), dann die HAL. Nur beim
    // ersten Durchlauf - danach tickt der Timer und macht ihn wertlos (s.o.).
    static bool static_done = false;
    if (!static_done) {
        checkStaticDrive(PIN_M1_A);
        checkStaticDrive(PIN_M1_B);
        Serial.println();
        static_done = true;
    }

    motor.init();
    Serial.printf("MotorHAL meldet: %s\n",
                  motor.usesSoftPwm() ? "Software-PWM aktiv"
                                      : "Hardware-PWM (analogWrite) <-- auf 40/41 wirkungslos!");
    Serial.printf("pinHasHardwarePwm(%u) = %d, pinHasHardwarePwm(%u) = %d\n\n",
                  PIN_M1_A, MotorHAL::pinHasHardwarePwm(PIN_M1_A),
                  PIN_M1_B, MotorHAL::pinHasHardwarePwm(PIN_M1_B));

    check(0,       0.0f,  0.0f);
    check(64,     25.0f,  0.0f);   //  64/256 = 25%
    check(120,    46.9f,  0.0f);   // 120/256 = 47%  (DEFAULT_ON_SPEED)
    check(255,    99.6f,  0.0f);   // 255/256 = 99.6%
    check(-120,    0.0f, 46.9f);
    check(0,       0.0f,  0.0f);

    motor.setSpeed(0);
    Serial.println("\nFertig. Der Motor wurde nur getaktet, nicht geregelt.");
    Serial.println("Beliebige Taste = Messung wiederholen.");
}

void setup() {
    Serial.begin(115200);
    const uint32_t t0 = millis();
    while (!Serial && millis() - t0 < 3000) { /* auf den Monitor warten */ }
    runAll();
}

void loop() {
    // Alle 5 s erneut messen, bis jemand mitliest; eine Taste stoesst sofort an.
    static uint32_t last = 0;
    if (Serial.available()) {
        while (Serial.available()) Serial.read();
        runAll();
        last = millis();
        return;
    }
    if (millis() - last >= 5000) {
        runAll();
        last = millis();
    }
}
