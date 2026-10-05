#include "../include/drive_controller.h"

#include "../include/simulator.h"
#include "../include/engine.h"
#include "../include/transmission.h"
#include "../include/vehicle.h"
#include "../include/ignition_module.h"
#include "../include/units.h"

#include <algorithm>
#include <cmath>

namespace {
    double clamp01(double v) { return (v < 0.0) ? 0.0 : (v > 1.0 ? 1.0 : v); }

    std::string rpmText(double radPerSec) {
        return std::to_string((int)std::round(units::toRpm(radPerSec)));
    }

    // Lower heating value of gasoline, J/kg.
    constexpr double FuelLhv = 44.0e6;
}

DriveController::DriveController() { /* void */ }

void DriveController::attach(Simulator *simulator) {
    m_simulator = simulator;
    m_engine = (simulator != nullptr) ? simulator->getEngine() : nullptr;
    m_transmission = (simulator != nullptr) ? simulator->getTransmission() : nullptr;
    m_vehicle = (simulator != nullptr) ? simulator->getVehicle() : nullptr;

    m_phase = ShiftPhase::Idle;
    m_queuedSteps = 0;
    m_launch = LaunchState::Off;
    m_limp = false;
    m_engineAccel = 0.0;
    m_lastEngineSpeed = 0.0;
    m_clutch = 1.0;
    m_throttle = 0.0;
    m_brake = 0.0;
    dynoSpeed = 0.0;

    if (m_vehicle != nullptr) {
        const Vehicle::Parameters &p = m_vehicle->getParameters();
        // Warm start: the engine is at operating temperature, not a cold soak.
        m_coolant = p.thermostatTemperature;
        m_oil = p.thermostatTemperature + 8.0;
    }

    m_lastFuel = (m_engine != nullptr) ? m_engine->getTotalFuelMassConsumed() : 0.0;

    if (m_engine != nullptr) m_engine->getIgnitionModule()->m_tcuCutFraction = 0.0;
}

void DriveController::setTcuEnabled(bool enabled) {
    if (m_transmission != nullptr) m_transmission->setTcuEnabled(enabled);
}

bool DriveController::isTcuEnabled() const {
    return m_transmission != nullptr && m_transmission->isTcuEnabled();
}

std::vector<DriveController::Event> DriveController::drainEvents() {
    std::vector<Event> out;
    out.swap(m_events);
    return out;
}

void DriveController::emit(EventType type, const std::string &text, bool automatic) {
    if (m_events.size() > 64) m_events.erase(m_events.begin());
    m_events.push_back({ type, text, automatic });
}

double DriveController::revLimit() const {
    return (m_engine != nullptr) ? m_engine->getIgnitionModule()->getRevLimit() : 0.0;
}

double DriveController::ceilingSpeed() const {
    if (m_transmission == nullptr) return 0.0;
    return revLimit() - m_transmission->getTcuOverRevMargin();
}

double DriveController::engineSpeed() const {
    return (m_engine != nullptr) ? std::abs(m_engine->getSpeed()) : 0.0;
}

double DriveController::advancePedal(
    double current, double target, double pressSeconds, double releaseSeconds, double dt)
{
    const bool pressing = (target > current);
    const double rampSeconds = pressing ? pressSeconds : releaseSeconds;
    if (rampSeconds <= 0.0) return target;

    const double step = dt / rampSeconds;
    const double diff = target - current;
    if (diff > step) return current + step;
    if (diff < -step) return current - step;

    return target;
}

DriveController::ShiftResult DriveController::requestShift(int delta) {
    if (m_transmission == nullptr || delta == 0) return ShiftResult::Invalid;

    const int gear = m_transmission->getGear();
    const int target = gear + delta;
    const bool up = delta > 0;

    m_status.lastPaddle = up ? 1 : -1;
    m_status.paddleAge = 0.0;

    if (target < -1 || target >= m_transmission->getGearCount()) {
        m_status.lastPaddleAccepted = false;
        return ShiftResult::Invalid;
    }

    // An upshift is an explicit "stop waiting" from the driver.
    if (up && m_queuedSteps > 0) {
        m_queuedSteps = 0;
        emit(EventType::QueueCancelled, "SHIFT ASSIST: queued downshift cancelled");
    }

    // Paddle lockout. A real sequential box cannot start a second shift while
    // the first is still moving the clutch, and letting it do so is exactly
    // how paddle-spam turned every blip into a clutch dump that pushed the car.
    if (m_phase != ShiftPhase::Idle) {
        m_status.lastPaddleAccepted = false;
        emit(EventType::ShiftLocked, "TCU: shift in progress");
        return ShiftResult::Locked;
    }

    if (!up && target >= 0 && m_transmission->isTcuEnabled()) {
        const double predicted = m_transmission->predictedEngineSpeed(target);
        const double ceiling = ceilingSpeed();
        if (predicted > ceiling && ceiling > 0.0) {
            if (assist.master && assist.queueDownshifts) {
                m_queuedSteps = (std::min)(m_queuedSteps + 1, gear);
                m_status.lastPaddleAccepted = true;
                emit(EventType::ShiftQueued,
                    "TCU: downshift QUEUED until " + rpmText(ceiling) + " rpm is safe");
                return ShiftResult::Queued;
            }

            m_status.lastPaddleAccepted = false;
            emit(EventType::ShiftRefused,
                "TCU: DOWNSHIFT REFUSED - would hit " + rpmText(predicted) + " rpm");
            return ShiftResult::RefusedOverRev;
        }
    }

    m_status.lastPaddleAccepted = true;
    return startShift(target, false);
}

DriveController::ShiftResult DriveController::startShift(int target, bool automatic) {
    const Transmission::Parameters &p = m_transmission->getTcuParameters();
    const int gear = m_transmission->getGear();

    m_shiftTarget = target;
    m_shiftIsUp = target > gear;
    m_shiftCut = m_shiftIsUp && gear >= 0 && p.UpshiftCut
        && assist.master && assist.upshiftCut;

    // Neutral needs no sequence -- just open the dog.
    if (target == -1 || gear == -1) {
        m_transmission->changeGear(target);
        m_phase = ShiftPhase::Engage;
        m_phaseTime = 0.0;
        m_phaseDuration = (std::max)(0.05, p.ShiftTime * 0.65);
    }
    else {
        m_phase = ShiftPhase::Disengage;
        m_phaseTime = 0.0;
        m_phaseDuration = (std::max)(0.01, p.ShiftTime * 0.35);
    }

    const std::string name = (target == -1) ? "N" : std::to_string(target + 1);
    emit(m_shiftIsUp ? EventType::ShiftUp : EventType::ShiftDown,
        std::string(automatic ? "SHIFT ASSIST: " : "") + "SHIFTED TO " + name, automatic);

    return ShiftResult::Accepted;
}

void DriveController::updateShift(double dt, double &throttle, double &clutchTarget, double &cut) {
    if (m_phase == ShiftPhase::Idle) return;

    const Transmission::Parameters &p = m_transmission->getTcuParameters();
    m_phaseTime += dt;

    switch (m_phase) {
    case ShiftPhase::Disengage:
        clutchTarget = 0.0;
        if (m_shiftCut) cut = 1.0;

        if (m_phaseTime >= m_phaseDuration) {
            m_transmission->changeGear(m_shiftTarget);

            if (!m_shiftIsUp && revMatch && m_shiftTarget >= 0) {
                // Aim below the limiter. rev_limit is what actually cuts spark,
                // and it is NOT the redline gauge mark.
                m_blipTarget = m_transmission->getTargetEngineSpeed();
                const double ceiling = revLimit() - units::rpm(200);
                if (m_blipTarget > ceiling) m_blipTarget = ceiling;

                m_phase = ShiftPhase::Blip;
                m_phaseTime = 0.0;
                m_phaseDuration = RevMatchMaxDuration;
            }
            else {
                m_phase = ShiftPhase::Engage;
                m_phaseTime = 0.0;
                m_phaseDuration = (std::max)(0.02, p.ShiftTime * 0.65);
            }
        }
        break;

    case ShiftPhase::Blip:
    {
        // PREDICTIVE BANG-BANG, not a proportional loop. A P-loop scaled
        // throttle by remaining error; for a tall-gear step it asked for about
        // half throttle, and at 7-8k half throttle is where this V12's net
        // torque crosses zero, so the engine parked in a limit cycle. Go wide
        // open and close on a predicted crossing instead. Do not "improve" this
        // back into a P-controller.
        clutchTarget = 0.0;
        const double projected = engineSpeed() + m_engineAccel * RevMatchLeadTime;
        if (m_blipTarget > 1.0 && projected < m_blipTarget && m_phaseTime < m_phaseDuration) {
            throttle = 1.0;
        }
        else {
            m_phase = ShiftPhase::Engage;
            m_phaseTime = 0.0;
            m_phaseDuration = (std::max)(0.02, p.ShiftTime * 0.65);
        }
        break;
    }

    case ShiftPhase::Engage:
    {
        const double s = clamp01(m_phaseTime / m_phaseDuration);
        clutchTarget = s;
        // Hold the ISR cut into the first part of the clutch closing so the
        // crank is pulled down by the gear rather than fighting it.
        if (m_shiftCut && s < 0.3) cut = 1.0;

        if (m_phaseTime >= m_phaseDuration) {
            m_phase = ShiftPhase::Idle;
            m_shiftCut = false;
        }
        break;
    }

    default:
        break;
    }
}

void DriveController::updateQueue() {
    if (m_queuedSteps <= 0 || m_phase != ShiftPhase::Idle) return;
    if (!(assist.master && assist.queueDownshifts)) {
        m_queuedSteps = 0;
        return;
    }

    const int gear = m_transmission->getGear();
    if (gear <= 0) {
        m_queuedSteps = 0;
        return;
    }

    const double predicted = m_transmission->predictedEngineSpeed(gear - 1);
    if (!m_transmission->isTcuEnabled() || predicted <= ceilingSpeed()) {
        --m_queuedSteps;
        startShift(gear - 1, true);
    }
}

void DriveController::updateAssist() {
    if (!assist.master || m_phase != ShiftPhase::Idle || m_queuedSteps > 0) return;
    if (m_launch == LaunchState::Armed || m_launch == LaunchState::Launching) return;
    if (m_simulator->m_dyno.m_enabled) return;

    const int gear = m_transmission->getGear();
    if (gear < 0) return;

    const double rpm = engineSpeed();
    const double limit = revLimit();

    if (assist.autoUpshift && gear + 1 < m_transmission->getGearCount()
        && rpm >= limit - units::rpm(250) && m_throttle > 0.3)
    {
        startShift(gear + 1, true);
        return;
    }

    // Under braking, walk down whenever the lower gear lands in the meat of the
    // rev range -- low enough that the TCU would always allow it.
    if (assist.autoDownshift && gear > 0 && m_brake > 0.15) {
        const double predicted = m_transmission->predictedEngineSpeed(gear - 1);
        if (rpm < 0.5 * limit && predicted < 0.75 * limit) {
            startShift(gear - 1, true);
        }
    }
}

void DriveController::updateLaunch(double &clutchTarget, double &cut) {
    const Transmission::Parameters &p = m_transmission->getTcuParameters();
    m_status.launchAvailable = p.LaunchControl;

    if (!p.LaunchControl || !launchEnabled) {
        m_launch = LaunchState::Off;
        m_launchLimiterCut = false;
        return;
    }

    const bool stopped = m_vehicle != nullptr && std::abs(m_vehicle->getSpeed()) < 1.0;
    const bool firstGear = m_transmission->getGear() == 0;
    const double rpm = engineSpeed();

    switch (m_launch) {
    case LaunchState::Off:
    case LaunchState::Ready:
        m_launch = (stopped && firstGear) ? LaunchState::Ready : LaunchState::Off;
        if (m_launch == LaunchState::Ready && m_brake > 0.5 && m_throttleTarget > 0.9
            && m_phase == ShiftPhase::Idle)
        {
            m_launch = LaunchState::Armed;
            emit(EventType::LaunchArmed,
                "LAUNCH CONTROL ARMED - " + rpmText(p.LaunchRpm) + " rpm, release brake");
        }
        break;

    case LaunchState::Armed:
        if (m_throttleTarget < 0.8 || !firstGear) {
            m_launch = LaunchState::Ready;
            m_launchLimiterCut = false;
            break;
        }

        clutchTarget = 0.0;
        // Soft limiter with hysteresis: full cut above the launch rpm, spark
        // back 150 rpm below. The on-off cycling is the launch-control stutter.
        if (rpm > p.LaunchRpm) m_launchLimiterCut = true;
        else if (rpm < p.LaunchRpm - units::rpm(150)) m_launchLimiterCut = false;
        if (m_launchLimiterCut) cut = 1.0;

        if (m_brakeTarget < 0.1) {
            m_launch = LaunchState::Launching;
            m_launchTime = 0.0;
            m_launchLimiterCut = false;
            emit(EventType::Launch, "LAUNCH!");
        }
        break;

    case LaunchState::Launching:
        // Clutch ramp runs in update(), which owns dt.
        break;
    }
}

void DriveController::updateThermal(double dt) {
    if (m_vehicle == nullptr || m_engine == nullptr) return;
    const Vehicle::Parameters &p = m_vehicle->getParameters();
    m_status.coolingModelled = p.cooling;
    m_status.overheatC = p.overheatTemperature;

    const double fuel = m_engine->getTotalFuelMassConsumed();
    double burned = fuel - m_lastFuel;
    m_lastFuel = fuel;
    if (!p.cooling) {
        m_limp = false;
        return;
    }
    if (burned < 0.0) burned = 0.0;  // a consumption counter reset

    const double heatIn = p.coolantHeatFraction * FuelLhv * burned;

    // Thermostat: below its temperature almost no coolant reaches the radiator.
    double open = (m_coolant - (p.thermostatTemperature - 5.0)) / 10.0;
    open = 0.05 + 0.95 * clamp01(open);
    const double airflow = std::abs(m_vehicle->getSpeed());
    const double conductance = open * (p.radiatorIdle + p.radiatorAirflow * airflow);
    const double heatOut = conductance * (m_coolant - p.ambientTemperature) * dt;

    if (p.engineThermalMass > 0.0) {
        m_coolant += (heatIn - heatOut) / p.engineThermalMass;
    }

    // Oil lags the coolant and runs hotter under load.
    const double oilTarget = m_coolant + 8.0 + 20.0 * m_throttle;
    m_oil += (oilTarget - m_oil) * (dt / (dt + 20.0));

    if (!m_limp && m_coolant > p.overheatTemperature) {
        m_limp = true;
        emit(EventType::LimpOn, "ECU: OVERHEAT - limp mode (power and revs limited)");
    }
    else if (m_limp && m_coolant < p.overheatTemperature - 10.0) {
        m_limp = false;
        m_limpLimiterCut = false;
        emit(EventType::LimpOff, "ECU: temperature recovered - limp mode off");
    }
}

void DriveController::updateDyno(double dt) {
    Dynamometer &dyno = m_simulator->m_dyno;
    if (dyno.m_enabled) {
        if (!dyno.m_hold) {
            if (m_simulator->getFilteredDynoTorque() > units::torque(1.0, units::ft_lb)) {
                dynoSpeed += units::rpm(500) * dt;
            }
            else {
                dynoSpeed *= (1 / (1 + dt));
            }

            if (dynoSpeed > m_engine->getRedline()) {
                dyno.m_enabled = false;
                dynoSpeed = units::rpm(0);
            }
        }
    }
    else if (!dyno.m_hold) {
        dynoSpeed = units::rpm(0);
    }

    if (dynoSpeed < m_engine->getDynoMinSpeed()) dynoSpeed = m_engine->getDynoMinSpeed();
    if (dynoSpeed > m_engine->getDynoMaxSpeed()) dynoSpeed = m_engine->getDynoMaxSpeed();
    dyno.m_rotationSpeed = dynoSpeed;
}

void DriveController::update(double dt) {
    if (m_simulator == nullptr || m_engine == nullptr) return;

    m_status.paddleAge += dt;

    // Smoothed crank acceleration for the rev-match predictor.
    {
        const double current = engineSpeed();
        if (dt > 0.0) {
            const double raw = (current - m_lastEngineSpeed) / dt;
            const double a = dt / (dt + EngineAccelFilterRC);
            m_engineAccel = m_engineAccel * (1.0 - a) + raw * a;
        }
        m_lastEngineSpeed = current;
    }

    // Pedals. Press and release ramp separately: a return spring snaps shut
    // far faster than a foot rolls on.
    m_throttle = advancePedal(m_throttle, clamp01(m_throttleTarget),
                              throttleRampTime, pedalReleaseTime, dt);
    m_brake = advancePedal(m_brake, clamp01(m_brakeTarget),
                           brakeRampTime, pedalReleaseTime, dt);

    double throttle = m_throttle;
    double clutchTarget = 1.0;
    double cut = 0.0;

    if (m_transmission != nullptr) {
        updateQueue();
        updateAssist();
        updateShift(dt, throttle, clutchTarget, cut);
        updateLaunch(clutchTarget, cut);

        if (m_launch == LaunchState::Launching) {
            const double t = m_transmission->getTcuParameters().LaunchClutchTime;
            m_launchTime += dt;
            const double s = (t > 0.0) ? clamp01(m_launchTime / t) : 1.0;
            if (clutchTarget > s) clutchTarget = s;
            if (s >= 1.0) m_launch = LaunchState::Ready;
        }
    }

    // Anti-stall. A TCU never holds a gear against a stopped car with the
    // clutch shut -- that stalls the engine, which is exactly what selecting
    // 1st at a standstill used to do. Below AntiStallFullRpm the clutch is
    // engaged in proportion to engine speed, so the car creeps off on throttle
    // and coasting to a stop in gear just opens the clutch. Cars with `tcu:
    // false` keep the manual-gearbox stall.
    if (m_transmission != nullptr && m_transmission->isTcuEnabled()
        && m_transmission->getGear() >= 0 && m_launch != LaunchState::Armed)
    {
        const double rpm = units::toRpm(engineSpeed());
        double s = (rpm - AntiStallOpenRpm) / (AntiStallFullRpm - AntiStallOpenRpm);
        s = clamp01(s);
        // Stopped and off the throttle: no creep, clutch fully open. Otherwise
        // the bite at idle drags a high-idling engine (this V12 sits ~1150)
        // down into a stall before the driver has done anything.
        const bool stopped = m_vehicle != nullptr && std::abs(m_vehicle->getSpeed()) < 1.0;
        if (stopped && m_throttleTarget < 0.05) s = 0.0;
        if (clutchTarget > s) clutchTarget = s;
    }

    updateThermal(dt);
    if (m_limp) {
        if (throttle > 0.5) throttle = 0.5;
        const double limpLimit = 0.6 * revLimit();
        const double rpm = engineSpeed();
        if (rpm > limpLimit) m_limpLimiterCut = true;
        else if (rpm < limpLimit - units::rpm(150)) m_limpLimiterCut = false;
        if (m_limpLimiterCut) cut = 1.0;
    }

    if (m_manualClutch) clutchTarget = clamp01(m_manualClutchPressure);

    const double clutchRC = 0.001;
    const double clutch_s = dt / (dt + clutchRC);
    m_clutch = m_clutch * (1 - clutch_s) + clamp01(clutchTarget) * clutch_s;

    m_engine->setSpeedControl(throttle);
    m_engine->getIgnitionModule()->m_tcuCutFraction = cut;
    m_simulator->setBrake(m_brake);
    if (m_transmission != nullptr) m_transmission->setClutchPressure(m_clutch);

    updateDyno(dt);

    // Status snapshot for the UI.
    Status &s = m_status;
    s.gear = (m_transmission != nullptr) ? m_transmission->getGear() : -1;
    s.gearCount = (m_transmission != nullptr) ? m_transmission->getGearCount() : 0;
    s.queuedSteps = m_queuedSteps;
    s.queuedGear = s.gear - m_queuedSteps;
    s.phase = m_phase;
    s.phaseProgress = (m_phaseDuration > 0.0) ? clamp01(m_phaseTime / m_phaseDuration) : 0.0;
    s.launch = m_launch;
    s.upshiftCutAvailable = m_transmission != nullptr
        && m_transmission->getTcuParameters().UpshiftCut;
    s.coolantC = m_coolant;
    s.oilC = m_oil;
    s.limp = m_limp;
    s.throttle = throttle;
    s.brake = m_brake;
    s.clutch = m_clutch;
    s.sparkCut = cut > 0.0;
    s.tcuEnabled = isTcuEnabled();

    s.advisorAvailable = false;
    if (m_transmission != nullptr && s.gear > 0) {
        const double predicted = m_transmission->predictedEngineSpeed(s.gear - 1);
        if (predicted > 0.0) {
            s.advisorAvailable = true;
            s.advisorRpm = units::toRpm(predicted);
            s.advisorSafe = predicted <= ceilingSpeed();
        }
    }
}
