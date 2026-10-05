// VRUM: scripted drive test for the shared DriveController (--drive-test).
// Included by bench_main.cpp. Drives the car headlessly through launch control,
// ISR upshifts, paddle spam, a TCU over-rev refusal, the Shift Assist queue and
// an overheat run, printing what happened so the logic is verified by numbers
// rather than by ear.

#include "../include/drive_controller.h"

#include <iomanip>

namespace {

struct DriveProbe {
    Simulator *sim;
    Engine *engine;
    DriveController *drive;
    std::vector<int16_t> *buffer;
    double t = 0.0;

    double rpm() const { return units::toRpm(std::abs(engine->getSpeed())); }
    double kmh() const {
        return units::convert(std::abs(sim->getVehicle()->getSpeed()), units::km / units::hour);
    }

    void step(double seconds, bool print = false) {
        const double dt = 1.0 / 60.0;
        const int frames = (int)(seconds * 60.0 + 0.5);
        for (int i = 0; i < frames; ++i) {
            drive->update(dt);
            stepFrame(sim, dt, *buffer);
            t += dt;
            for (const auto &e : drive->drainEvents()) {
                std::cout << "    [" << std::fixed << std::setprecision(2) << t << "s] "
                          << e.text << "\n";
            }
        }
        if (print) report();
    }

    void report() const {
        const DriveController::Status &s = drive->getStatus();
        std::cout << "    t=" << std::fixed << std::setprecision(2) << t
                  << "  gear=" << (s.gear + 1) << "  rpm=" << (int)rpm()
                  << "  speed=" << std::setprecision(1) << kmh() << " km/h"
                  << "  clutch=" << std::setprecision(2) << s.clutch
                  << "  H2O=" << std::setprecision(1) << s.coolantC << "C\n";
    }
};

int runDriveTest(Simulator *simulator, Engine *engine, std::vector<int16_t> &buffer) {
    DriveController drive;
    drive.attach(simulator);
    drive.brakeRampTime = 0.0;

    DriveProbe p{ simulator, engine, &drive, &buffer };
    int failures = 0;
    auto check = [&](bool ok, const std::string &what) {
        std::cout << (ok ? "  PASS  " : "  FAIL  ") << what << "\n";
        if (!ok) ++failures;
    };

    const Transmission::Parameters &tp = simulator->getTransmission()->getTcuParameters();
    const double revLimit = units::toRpm(engine->getIgnitionModule()->getRevLimit());
    std::cout << "drive test: rev limit " << (int)revLimit << " rpm, shift_time "
              << tp.ShiftTime << " s, launch_control " << tp.LaunchControl
              << " @ " << (int)units::toRpm(tp.LaunchRpm) << " rpm, upshift_cut "
              << tp.UpshiftCut << "\n";

    // Start it the way a driver does: starter held, throttle cracked, until it
    // catches and runs on its own.
    simulator->m_starterMotor.m_enabled = true;
    drive.setThrottleInput(0.15);
    for (int i = 0; i < 8 * 60 && p.rpm() < 1200.0; ++i) p.step(1.0 / 60.0);
    simulator->m_starterMotor.m_enabled = false;
    drive.setThrottleInput(0.0);
    p.step(1.5);
    std::cout << "  started, idling at " << (int)p.rpm() << " rpm\n";
    check(p.rpm() > 500.0, "engine starts and idles");
    drive.setThrottleInput(1.0);
    for (int i = 0; i < 6; ++i) { p.step(0.1); std::cout << "    neutral WOT rpm=" << (int)p.rpm() << "\n"; }
    drive.setThrottleInput(0.0);
    p.step(2.0);
    std::cout << "    back to idle rpm=" << (int)p.rpm() << "\n";

    if (std::getenv("VRUM_ISOLATE") != nullptr) {
        auto wot = [&](const char *label) {
            drive.setThrottleInput(1.0);
            for (int i = 0; i < 4; ++i) {
                p.step(0.1);
                std::cout << "    " << label << " rpm=" << (int)p.rpm()
                          << " clutch=" << simulator->getTransmission()->getClutchPressure() << "\n";
            }
            drive.setThrottleInput(0.0);
            p.step(1.5);
        };
        drive.setBrakeInput(1.0); wot("N+brake"); drive.setBrakeInput(0.0);
        drive.requestShift(+1); p.step(0.5);
        drive.setManualClutch(true, 0.0); wot("1st, clutch held 0, no brake");
        drive.setBrakeInput(1.0); wot("1st, clutch held 0, brake");
        drive.setManualClutch(false, 1.0); drive.setBrakeInput(0.0);
        return 0;
    }

    // --- 1. Launch control -------------------------------------------------
    std::cout << "\n[1] launch control\n";
    drive.setThrottleInput(0.0);
    drive.setBrakeInput(1.0);
    p.step(0.5);
    drive.requestShift(+1);   // N -> 1st
    p.step(0.5, true);

    if (tp.LaunchControl) {
        drive.setThrottleInput(1.0);
        for (int i = 0; i < 15; ++i) {
            p.step(0.1);
            const DriveController::Status &s = drive.getStatus();
            std::cout << "    trace rpm=" << (int)p.rpm() << " thr=" << s.throttle
                      << " cut=" << s.sparkCut << " clutch=" << s.clutch
                      << " ign=" << engine->getIgnitionModule()->m_enabled
                      << " launch=" << (int)s.launch << "\n";
        }
        double peak = 0.0, low = 1e9;
        for (int i = 0; i < 60; ++i) {
            p.step(1.0 / 60.0);
            peak = std::max(peak, p.rpm());
            low = std::min(low, p.rpm());
        }
        p.report();
        const double target = units::toRpm(tp.LaunchRpm);
        check(drive.getStatus().launch == DriveController::LaunchState::Armed,
              "armed with brake + full throttle in 1st");
        check(peak < target + 400 && low > target - 900,
              "holds launch rpm (" + std::to_string((int)low) + ".." +
              std::to_string((int)peak) + " vs " + std::to_string((int)target) + ")");
        check(p.kmh() < 1.0, "car stays put while armed");

        drive.setBrakeInput(0.0);
        p.step(1.0, true);
        check(p.kmh() > 10.0, "car launches when the brake is released");
    }
    else {
        drive.setBrakeInput(0.0);
        drive.setThrottleInput(1.0);
        p.step(1.0, true);
    }

    // --- 2. Upshifts at the limiter (ISR cut) -----------------------------
    std::cout << "\n[2] upshifts\n";
    drive.assist.master = true;
    drive.assist.upshiftCut = true;
    drive.assist.queueDownshifts = false;
    bool sawCut = false;
    for (int shifts = 0; shifts < 3; ++shifts) {
        double guard = 0.0;
        while (p.rpm() < revLimit - 400.0 && guard < 15.0) { p.step(1.0 / 60.0); guard += 1.0 / 60.0; }
        drive.requestShift(+1);
        for (int i = 0; i < 30; ++i) {
            p.step(1.0 / 60.0);
            sawCut = sawCut || drive.getStatus().sparkCut;
        }
        p.report();
    }
    check(drive.getStatus().gear == 3, "reached 4th gear");
    if (tp.UpshiftCut) check(sawCut, "ISR spark cut fired during upshifts");

    // --- 3. Paddle spam lockout -------------------------------------------
    std::cout << "\n[3] paddle spam\n";
    {
        const DriveController::ShiftResult a = drive.requestShift(+1);
        const DriveController::ShiftResult b = drive.requestShift(-1);
        check(a == DriveController::ShiftResult::Accepted, "first paddle accepted");
        check(b == DriveController::ShiftResult::Locked, "second paddle locked out mid-shift");
        p.step(0.5, true);
    }

    // Spam with no throttle must not gain speed (the old clutch-dump bug).
    drive.setThrottleInput(0.0);
    p.step(0.5);
    const double v0 = p.kmh();
    for (int i = 0; i < 20; ++i) {
        drive.requestShift((i % 2 == 0) ? -1 : +1);
        p.step(0.15);
    }
    const double v1 = p.kmh();
    std::cout << "    speed before spam " << v0 << " km/h, after " << v1 << " km/h\n";
    check(v1 <= v0 + 1.0, "paddle spam with throttle closed does not accelerate the car");

    // --- 4. TCU refusal and the Shift Assist queue -------------------------
    std::cout << "\n[4] TCU over-rev guard + queue\n";
    // Climb to near the limiter in the current gear, then try to drop two.
    drive.setThrottleInput(1.0);
    {
        double guard = 0.0;
        while (p.rpm() < revLimit - 500.0 && guard < 15.0) { p.step(1.0 / 60.0); guard += 1.0 / 60.0; }
    }
    p.report();
    drive.assist.queueDownshifts = false;
    const DriveController::ShiftResult refused = drive.requestShift(-1);
    check(refused == DriveController::ShiftResult::RefusedOverRev, "TCU refuses a downshift past the limiter");

    drive.assist.queueDownshifts = true;
    const int gearBefore = drive.getStatus().gear;
    const DriveController::ShiftResult queued = drive.requestShift(-1);
    check(queued == DriveController::ShiftResult::Queued, "Shift Assist queues it instead");
    drive.setThrottleInput(0.0);
    drive.setBrakeInput(1.0);
    double peakAfter = 0.0;
    for (int i = 0; i < 6 * 60 && drive.getStatus().gear == gearBefore; ++i) {
        p.step(1.0 / 60.0);
    }
    for (int i = 0; i < 60; ++i) { p.step(1.0 / 60.0); peakAfter = std::max(peakAfter, p.rpm()); }
    p.report();
    check(drive.getStatus().gear == gearBefore - 1, "queued downshift fired once it became safe");
    check(peakAfter < revLimit, "and it did not over-rev (" + std::to_string((int)peakAfter) + " rpm)");
    drive.setBrakeInput(0.0);

    // --- 5. Cooling + limp mode --------------------------------------------
    std::cout << "\n[5] overheat on the dyno (no airflow, wide open)\n";
    drive.requestShift(-1);
    p.step(0.3);
    while (drive.getStatus().gear > -1) { drive.requestShift(-1); p.step(0.3); }
    simulator->m_dyno.m_enabled = true;
    simulator->m_dyno.m_hold = true;
    drive.dynoSpeed = units::rpm(std::min(7000.0, revLimit * 0.8));
    drive.setThrottleInput(1.0);
    const double c0 = drive.getStatus().coolantC;
    bool limp = false;
    double secs = 0.0;
    for (; secs < 90.0 && !limp; secs += 1.0) {
        p.step(1.0);
        limp = drive.getStatus().limp;
    }
    p.report();
    std::cout << "    coolant " << c0 << " -> " << drive.getStatus().coolantC
              << " C in " << secs << " s\n";
    check(limp, "sustained WOT with no airflow overheats into limp mode");

    std::cout << "\n" << (failures == 0 ? "ALL DRIVE CHECKS PASSED" : "DRIVE CHECKS FAILED: ")
              << (failures == 0 ? "" : std::to_string(failures)) << "\n";
    return failures == 0 ? 0 : 3;
}

} // namespace
