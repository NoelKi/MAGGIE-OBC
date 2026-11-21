// Host-Test der Downlink-Stufen (TelemetryDownlink::sendFrame, data_rates.hpp).
//
// Die UART wird mit genau DOWNLINK_BAUDRATE geleert (Teensy: 40 Byte Core-Puffer
// + 512 Byte serial4_tx_extra). Die Frames kommen im Takt von System::run():
// Status 2 Hz, Diagnose reihum, Gelenke 7 x 5 Hz, IMU, Kraft (Wandlung 80 Hz),
// Motoren, RCU-IMU, Arm-Spannung/-Temperatur. Geprueft wird:
//  - Stufe 1 (STATUS) und 2 (Gelenkwinkel) verlieren NICHTS
//  - jede Art der Stufe 3 kommt gleichmaessig an (keine verhungert)
//  - die Leitung ist nicht dauerhaft ueberbucht
//
//   tools/downlink_sim/run.sh   (baut fuer 38400 UND 115200 Baud)
#include <Arduino.h>
#include "obc_log.hpp"
#include "data_rates.hpp"
#include "hal/telemetry_hal.hpp"
#include "hal/arm_hal.hpp"
#include <map>
#include <string>
#include <vector>

static uint32_t g_ms = 0;
uint32_t millis() { return g_ms; }
FakeLog Log;

static constexpr int TX_CAPACITY = 40 + 512;

class FakeUart : public HardwareSerial {
public:
    void begin(uint32_t, uint16_t) override {}
    int available() override { return 0; }
    int read() override { return -1; }
    int availableForWrite() override { return TX_CAPACITY - static_cast<int>(queue_.size()); }
    size_t write(const uint8_t* b, size_t n) override {
        for (size_t i = 0; i < n; i++) queue_.push_back(b[i]);
        return n;
    }
    /// 1 ms Leitung: DL_BYTES_PER_S / 1000 Byte gehen raus (Bruchteile gesammelt).
    void drain() {
        credit_ += rates::DL_BYTES_PER_S;
        while (credit_ >= 1000 && !queue_.empty()) {
            credit_ -= 1000;
            rx_.push_back(queue_.front());
            queue_.erase(queue_.begin());
            busy_ms_frac_++;
            if (rx_.size() == DOWNLINK_FRAME_SIZE) {
                arrived[{rx_[1], rx_[2]}]++;
                rx_.clear();
            }
        }
        if (queue_.empty()) credit_ = 0;   // Leitung ruht - kein Guthaben ansparen
    }
    std::map<std::pair<uint8_t, uint8_t>, long> arrived;
    long bytesOnLine() const { return busy_ms_frac_; }
private:
    std::vector<uint8_t> queue_;
    std::vector<uint8_t> rx_;
    uint64_t credit_ = 0;
    long busy_ms_frac_ = 0;
};

class NullUart : public HardwareSerial {
public:
    void begin(uint32_t, uint16_t) override {}
    int available() override { return 0; }
    int read() override { return -1; }
    int availableForWrite() override { return 1000; }
    size_t write(const uint8_t*, size_t n) override { return n; }
};

static int g_failed = 0;
static void check(bool ok, const std::string& what) {
    printf("    [%s] %s\n", ok ? " OK " : "FEHL", what.c_str());
    if (!ok) g_failed++;
}

int main() {
    const uint32_t RUN_MS = 60000;
    printf("== Downlink %lu Baud (%lu B/s), %lu s ==\n",
           (unsigned long)rates::DOWNLINK_BAUDRATE, (unsigned long)rates::DL_BYTES_PER_S,
           (unsigned long)(RUN_MS / 1000));
    printf("   Stufe 3: IMU %lu ms, Kraft %lu ms, Motor %lu ms, RCU-IMU %lu ms; Grenzen Rest %d B, Gelenke %d B\n",
           (unsigned long)rates::DL_IMU_INTERVAL_MS, (unsigned long)rates::DL_FORCE_INTERVAL_MS,
           (unsigned long)rates::DL_MOTOR_INTERVAL_MS, (unsigned long)rates::ARM_IMU_INTERVAL_MS,
           rates::DL_BULK_QUEUE_MAX, rates::DL_MOTION_QUEUE_MAX);

    FakeUart uart;
    TelemetryDownlink dl(uart);
    dl.init(rates::DOWNLINK_BAUDRATE);
    NullUart null_uart;
    ArmLink arm(null_uart);

    std::map<std::pair<uint8_t, uint8_t>, long> generated;
    auto gen = [&](DownlinkSubsystem s, uint8_t m, int n = 1) { generated[{static_cast<uint8_t>(s), m}] += n; };

    IMUReading imu{};
    ForceReading f1{}; f1.count = 3;
    ForceReading f2{}; f2.count = 4;
    ArmJoint joint{};
    ArmRcuImu rcu{};
    uint32_t last_status = 0, last_diag = 0, last_joint = 0, last_imu = 0, last_motor = 0,
             last_rcu = 0, last_power = 0, last_f1 = 0, last_f2 = 0;
    uint8_t diag_slot = 0, power_step = 0, joint_node = 1;
    bool motor2_first = false;
    const uint8_t cam_flags[4] = {}, cam_age[4] = {};
    constexpr uint8_t SYS = static_cast<uint8_t>(DownlinkSubsystem::SYS);

    for (g_ms = 1; g_ms <= RUN_MS; g_ms++) {
        const uint32_t now = g_ms;
        if (now - last_status >= rates::STATUS_INTERVAL_MS) {
            last_status = now;
            dl.sendSystem(0, 0, now); gen(DownlinkSubsystem::SYS, 0x01);
        }
        if (now - last_diag >= rates::SYS_INTERVAL_MS / 8) {
            last_diag = now;
            switch (diag_slot) {
                case 0: dl.sendUplinkStats(0, 0, 0, 0); dl.sendUplinkRaw(nullptr, 0);
                        generated[{SYS, 0x02}]++; generated[{SYS, 0x03}]++; break;
                case 1: dl.sendArmLink(arm, now); generated[{SYS, 0x04}]++; break;
                case 2: dl.sendArmTraj(0, 0, 0, 0, 0, 0); generated[{SYS, 0x05}]++; break;
                case 3: dl.sendArmFlight(0, 0, 0, 0, 0, 0, 0, 0); generated[{SYS, 0x06}]++; break;
                case 4: dl.sendCamera(cam_flags, cam_age); dl.sendCameraInfo(1, 1, 0, 0);
                        generated[{SYS, 0x07}]++; generated[{SYS, 0x08}]++; break;
                case 5: dl.sendSd(2, 0, 0, 0, 0); generated[{SYS, 0x09}]++; break;
                case 6: dl.sendBoot(1, 0, 0, 0, false, 0, false); generated[{SYS, 0x0A}]++; break;
                default: break;
            }
            diag_slot = (diag_slot + 1) % 8;
        }
        if (now - last_joint >= rates::ARM_JOINT_INTERVAL_MS / ARM_MAX_JOINTS) {
            last_joint = now;
            if (joint_node <= 7) { dl.sendArmJoint(joint_node, joint); gen(DownlinkSubsystem::ARM, 0x01); }
            joint_node = joint_node % ARM_MAX_JOINTS + 1;
        }
        if (now - last_imu >= rates::DL_IMU_INTERVAL_MS) {
            last_imu = now;
            dl.sendImu(imu); gen(DownlinkSubsystem::IMU, 0x01); gen(DownlinkSubsystem::IMU, 0x02);
        }
        // Kraft: Wandlung alle 12,5 ms (80 Hz), Sensor 2 versetzt.
        if ((now * 2) % 25 == 0 && now - last_f1 >= rates::DL_FORCE_INTERVAL_MS) {
            last_f1 = now; dl.sendForce(DownlinkForceMsg::TARGET1, f1, false, true); gen(DownlinkSubsystem::FORCE, 0x01);
        }
        if (((now + 6) * 2) % 25 == 0 && now - last_f2 >= rates::DL_FORCE_INTERVAL_MS) {
            last_f2 = now; dl.sendForce(DownlinkForceMsg::TARGET2, f2, false, true); gen(DownlinkSubsystem::FORCE, 0x02);
        }
        if (now - last_motor >= rates::DL_MOTOR_INTERVAL_MS) {
            last_motor = now;
            motor2_first = !motor2_first;
            dl.sendMotor(motor2_first ? DownlinkMotorMsg::STATE2 : DownlinkMotorMsg::STATE, 0, 0, 0);
            dl.sendMotor(motor2_first ? DownlinkMotorMsg::STATE : DownlinkMotorMsg::STATE2, 0, 0, 0);
            gen(DownlinkSubsystem::MOTOR, 0x01); gen(DownlinkSubsystem::MOTOR, 0x02);
        }
        if (now - last_rcu >= rates::ARM_IMU_INTERVAL_MS) {
            last_rcu = now;
            dl.sendArmRcuImu(rcu); gen(DownlinkSubsystem::ARM, 0x04); gen(DownlinkSubsystem::ARM, 0x05);
        }
        if (now - last_power >= rates::ARM_POWER_INTERVAL_MS / (2 * ARM_MAX_JOINTS)) {
            last_power = now;
            const uint8_t node = power_step / 2 + 1;
            if (node <= 7) {
                if ((power_step & 1) == 0) { dl.sendArmVoltage(node, joint); gen(DownlinkSubsystem::ARM, 0x02); }
                else                       { dl.sendArmTemp(node, joint);    gen(DownlinkSubsystem::ARM, 0x03); }
            }
            power_step = (power_step + 1) % (2 * ARM_MAX_JOINTS);
        }
        uart.drain();
    }

    static const std::map<std::pair<uint8_t, uint8_t>, const char*> NAMES = {
        {{0x03, 0x01}, "SYS/STATE"}, {{0x03, 0x02}, "SYS/UPLINK"}, {{0x03, 0x03}, "SYS/UPLINK_RAW"},
        {{0x03, 0x04}, "SYS/ARM_LINK"}, {{0x03, 0x05}, "SYS/ARM_TRAJ"}, {{0x03, 0x06}, "SYS/ARM_FLIGHT"},
        {{0x03, 0x07}, "SYS/CAMERA"}, {{0x03, 0x08}, "SYS/CAMERA_INFO"}, {{0x03, 0x09}, "SYS/SD"},
        {{0x03, 0x0A}, "SYS/BOOT"}, {{0x05, 0x01}, "ARM/JOINT"}, {{0x01, 0x01}, "IMU/ACCEL"},
        {{0x01, 0x02}, "IMU/GYRO"}, {{0x04, 0x01}, "FORCE/T1"}, {{0x04, 0x02}, "FORCE/T2"},
        {{0x02, 0x01}, "MOTOR/1"}, {{0x02, 0x02}, "MOTOR/2"}, {{0x05, 0x04}, "ARM/RCU_ACCEL"},
        {{0x05, 0x05}, "ARM/RCU_GYRO"}, {{0x05, 0x02}, "ARM/VOLTAGE"}, {{0x05, 0x03}, "ARM/TEMP"},
    };
    printf("\n   %-16s %9s %9s %7s\n", "Frame", "erzeugt", "angekom.", "Hz");
    for (const auto& [key, n] : generated) {
        const long got = uart.arrived[key];
        const auto it = NAMES.find(key);
        printf("   %-16s %9ld %9ld %7.1f\n", it != NAMES.end() ? it->second : "?", n, got,
               got * 1000.0 / RUN_MS);
    }
    const double load = uart.bytesOnLine() * 100.0 / (rates::DL_BYTES_PER_S * (RUN_MS / 1000.0));
    printf("\n   Leitung ausgelastet: %.0f %%\n", load);

    check(dl.uartDropped(DownlinkPrio::STATUS) == 0, "Stufe 1 (Status) verliert nichts");
    check(dl.uartDropped(DownlinkPrio::MOTION) == 0, "Stufe 2 (Gelenkwinkel) verliert nichts");
    long bulk_gen = 0;
    bool fair = true;
    for (const auto& [key, n] : generated) {
        if (TelemetryDownlink::prioOf(key.first, key.second) != DownlinkPrio::BULK) continue;
        bulk_gen += n;
        if (uart.arrived[key] < n * 0.9) fair = false;
    }
    check(fair, "Stufe 3: jede Art kommt zu >= 90 % an (" +
                std::to_string(dl.uartDropped(DownlinkPrio::BULK)) + " von " +
                std::to_string(bulk_gen) + " verworfen)");
    check(load < 95.0, "Leitung nicht ueberbucht");
    printf("\n%s - %d Pruefung(en) fehlgeschlagen.\n", g_failed ? "FEHLER" : "ALLES OK", g_failed);
    return g_failed ? 1 : 0;
}
