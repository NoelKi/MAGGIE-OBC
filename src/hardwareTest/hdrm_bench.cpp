/**
 * @file hdrm_bench.cpp
 * @brief Handbetrieb fuer HDRM-Motor 1 am Tisch, ohne Zustandsmaschine und Downlink.
 *
 * Fuer die Fehlersuche an der Strecke Teensy -> DRV8871 -> Motor -> Encoder.
 * Gefahren wird ueber die ECHTE Flight-Klasse MotorHAL auf den Pins aus
 * pin_config.hpp (Treiber 41/40, Encoder 0/1) - es wird also genau der Pfad
 * getestet, den auch die Firmware benutzt, nur ohne TEST-Sperre, ohne REXUS
 * und ohne Telemetrie dazwischen.
 *
 * Die Encoderposition laeuft dauerhaft mit. Ist der Motor stromlos, laesst sich
 * die Welle von Hand drehen - zaehlt der Encoder dabei, ist seine Verdrahtung
 * und Versorgung in Ordnung.
 *
 * Bauen und flashen:    pio run -e bench -t upload
 * Monitor:              pio device monitor -e bench
 * Zurueck zur Firmware: pio run -e teensy41 -t upload
 */

#include <Arduino.h>
#include <Encoder.h>
#include "hal/motor_hal.hpp"
#include "pin_config.hpp"

static MotorHAL motor(PIN_M1_A, PIN_M1_B, 1);

/// Dauer eines Bursts - kurz genug, dass ein blockierter Motor nichts abbekommt.
static constexpr uint32_t BURST_MS = 600;

static long last_pos = 0;

static void printMenu() {
    Serial.println();
    Serial.println("=== MAGGIE HDRM-Motor: Handbetrieb ===");
    Serial.printf("Treiber Pin %u/%u, Encoder Pin %u/%u, %s\n\n",
                  PIN_M1_A, PIN_M1_B, PIN_M1_ENC_A, PIN_M1_ENC_B,
                  motor.usesSoftPwm() ? "Software-PWM" : "Hardware-PWM");
    Serial.println("  f / r   Burst mit vollem Moment (PWM 255), 600 ms vor / zurueck");
    Serial.println("  o       Dauerlauf an (PWM 120, wie Telecommand MOTOR_ON)");
    Serial.println("  x       aus");
    Serial.println("  h / j   geregelte halbe Umdrehung vor / zurueck");
    Serial.println("  z       Encoder auf 0 setzen");
    Serial.println("  ?       dieses Menue");
    Serial.println();
    Serial.println("Ohne Motorstrom: Welle von Hand drehen - zaehlt der Encoder,");
    Serial.println("ist seine Verdrahtung in Ordnung.");
    Serial.println();
}

/// Position ausgeben, sobald sie sich geaendert hat.
static void reportEncoder() {
    const long pos = motor.getPosition();
    if (pos == last_pos) return;
    Serial.printf("Encoder = %ld  (%.1f Grad)\n",
                  pos, 360.0f * static_cast<float>(pos)
                       / static_cast<float>(MotorHAL::COUNTS_PER_REV));
    last_pos = pos;
}

/// Kurz mit voller Kraft fahren und melden, was der Encoder dazu sagt.
static void burst(int16_t speed) {
    const long start = motor.getPosition();
    Serial.printf("\n--> Burst setSpeed(%+d) fuer %lu ms\n",
                  speed, static_cast<unsigned long>(BURST_MS));

    motor.setSpeed(speed);
    const uint32_t t0 = millis();
    while (millis() - t0 < BURST_MS) {
        reportEncoder();
    }
    motor.setSpeed(0);

    const long delta = motor.getPosition() - start;
    Serial.printf("<-- aus. Delta = %+ld Counts  %s\n", delta,
                  delta == 0 ? "(keine Bewegung - Motorstrom, Treiber oder Encoder pruefen)"
                             : "");
}

void setup() {
    Serial.begin(115200);
    const uint32_t t0 = millis();
    while (!Serial && millis() - t0 < 3000) { /* auf den Monitor warten */ }

    motor.init();
    motor.initEncoder(PIN_M1_ENC_A, PIN_M1_ENC_B);
    motor.off();

    printMenu();
}

void loop() {
    motor.update();      // laufende geregelte Fahrt weiterfuehren
    reportEncoder();

    if (!Serial.available()) return;
    const char c = static_cast<char>(Serial.read());
    if (c == '\n' || c == '\r') return;

    switch (c) {
        case '?': printMenu(); break;
        case 'f': burst(255);  break;
        case 'r': burst(-255); break;

        case 'o':
            motor.on();
            Serial.println("Dauerlauf an (PWM 120). 'x' schaltet aus.");
            break;

        case 'x':
            motor.off();
            Serial.println("Motor aus.");
            break;

        case 'h':
            Serial.printf("Geregelt: +180 Grad ab %ld\n", motor.getPosition());
            motor.halfTurnForward();
            break;

        case 'j':
            Serial.printf("Geregelt: -180 Grad ab %ld\n", motor.getPosition());
            motor.halfTurnReverse();
            break;

        case 'z':
            motor.zeroPosition();
            last_pos = 0;
            Serial.println("Encoder auf 0 gesetzt.");
            break;

        default:
            Serial.printf("Unbekannte Taste '%c' - '?' zeigt das Menue.\n", c);
            break;
    }
}
