#ifndef VRUM_DRIVE_CONTROLLER_H
#define VRUM_DRIVE_CONTROLLER_H

#include <string>
#include <vector>

class Simulator;
class Engine;
class Transmission;
class Vehicle;

// VRUM: everything between the driver's inputs and the simulator.
//
// Pedal travel, the shift sequence (clutch open -> ratio swap -> blip or spark
// cut -> clutch close), the TCU's over-rev guard, launch control, the cooling
// model with limp mode, Shift Assist, and the dyno sweep. It used to live inside
// the Windows app; it lives in the core library now so the desktop build and the
// Android build run literally the same code. Platform code only feeds it inputs
// once per frame and reads Status back.
//
// Nothing in here may touch rendering, windowing, or audio-device code.
class DriveController {
    public:
        enum class ShiftResult {
            Accepted,
            Locked,         // a shift is already in progress (paddle lockout)
            RefusedOverRev, // TCU: predicted rpm past the limiter
            Queued,         // Shift Assist parked a refused downshift
            Invalid         // past top gear / below neutral / no transmission
        };

        enum class ShiftPhase { Idle, Disengage, Blip, Engage };
        enum class LaunchState { Off, Ready, Armed, Launching };

        enum class EventType {
            Message,
            ShiftUp,
            ShiftDown,
            ShiftLocked,
            ShiftRefused,
            ShiftQueued,
            QueueCancelled,
            LaunchArmed,
            Launch,
            LimpOn,
            LimpOff
        };

        struct Event {
            EventType type;
            std::string text;
            bool automatic = false;
        };

        // Each Shift Assist feature is a sub-toggle; none act unless `master`
        // is on. They are additive: a manual paddle press always wins over an
        // automatic decision, because both use the same single shift sequence.
        struct Assist {
            bool master = false;
            bool queueDownshifts = true;
            bool autoUpshift = false;
            bool autoDownshift = false;
            bool upshiftCut = true;
        };

        struct Status {
            int gear = -1;              // -1 = neutral
            int gearCount = 0;
            int queuedSteps = 0;        // downshifts waiting for the TCU
            int queuedGear = -1;        // where the queue will end up
            ShiftPhase phase = ShiftPhase::Idle;
            double phaseProgress = 0.0;

            // Paddle feedback: direction of the last request, whether it was
            // accepted, and how long ago (seconds) -- the UI fades a flash on it.
            int lastPaddle = 0;
            bool lastPaddleAccepted = true;
            double paddleAge = 1e9;

            // Shift Advisor: what the next downshift would spin the engine to.
            double advisorRpm = 0.0;
            bool advisorSafe = true;
            bool advisorAvailable = false;

            LaunchState launch = LaunchState::Off;
            bool launchAvailable = false;   // the script enables launch control
            bool upshiftCutAvailable = false;

            double coolantC = 0.0;
            double oilC = 0.0;
            bool coolingModelled = false;
            bool limp = false;
            double overheatC = 0.0;

            double throttle = 0.0;      // what actually reaches the engine
            double brake = 0.0;
            double clutch = 1.0;
            bool sparkCut = false;
            bool tcuEnabled = false;
        };

    public:
        DriveController();

        // Call after every script (re)load. nullptr detaches.
        void attach(Simulator *simulator);

        // Driver inputs, 0..1 pedal TARGETS (travel times are applied here).
        void setThrottleInput(double target) { m_throttleTarget = target; }
        void setBrakeInput(double target) { m_brakeTarget = target; }
        // Manual clutch pedal: when `held`, the clutch follows `pressure`
        // (0 = pedal floored) instead of the automatic sequence.
        void setManualClutch(bool held, double pressure) {
            m_manualClutch = held;
            m_manualClutchPressure = pressure;
        }

        ShiftResult requestShift(int delta);

        void update(double dt);

        const Status &getStatus() const { return m_status; }
        std::vector<Event> drainEvents();

        // Settings. Plain fields so desktop sliders can bind to their address.
        double throttleRampTime = 0.0;
        double brakeRampTime = 0.25;
        double pedalReleaseTime = 0.0;
        bool revMatch = true;
        bool launchEnabled = true;
        Assist assist;

        // Dyno sweep state, shared by both platforms.
        double dynoSpeed = 0.0;

        void setTcuEnabled(bool enabled);
        bool isTcuEnabled() const;

        // Tunables (seconds).
        static constexpr double RevMatchLeadTime = 0.06;
        static constexpr double EngineAccelFilterRC = 0.02;
        static constexpr double RevMatchMaxDuration = 1.2;
        // Anti-stall clutch window (rpm): fully open below, fully closed above.
        static constexpr double AntiStallOpenRpm = 1300.0;
        static constexpr double AntiStallFullRpm = 2500.0;

    protected:
        void emit(EventType type, const std::string &text, bool automatic = false);
        ShiftResult startShift(int target, bool automatic);
        double ceilingSpeed() const;
        double revLimit() const;
        double engineSpeed() const;
        void updateShift(double dt, double &throttle, double &clutchTarget, double &cut);
        void updateQueue();
        void updateAssist();
        void updateLaunch(double &clutchTarget, double &cut);
        void updateThermal(double dt);
        void updateDyno(double dt);

        static double advancePedal(double current, double target,
                                   double pressSeconds, double releaseSeconds, double dt);

        Simulator *m_simulator = nullptr;
        Engine *m_engine = nullptr;
        Transmission *m_transmission = nullptr;
        Vehicle *m_vehicle = nullptr;

        double m_throttleTarget = 0.0;
        double m_brakeTarget = 0.0;
        double m_throttle = 0.0;
        double m_brake = 0.0;
        bool m_manualClutch = false;
        double m_manualClutchPressure = 1.0;
        double m_clutch = 1.0;

        double m_engineAccel = 0.0;
        double m_lastEngineSpeed = 0.0;

        // Shift sequence
        ShiftPhase m_phase = ShiftPhase::Idle;
        double m_phaseTime = 0.0;
        double m_phaseDuration = 0.0;
        int m_shiftTarget = -1;
        bool m_shiftIsUp = false;
        bool m_shiftCut = false;
        double m_blipTarget = 0.0;

        int m_queuedSteps = 0;

        // Launch
        LaunchState m_launch = LaunchState::Off;
        double m_launchTime = 0.0;
        bool m_launchLimiterCut = false;

        // Cooling
        double m_coolant = 90.0;
        double m_oil = 98.0;
        double m_lastFuel = 0.0;
        bool m_limp = false;
        bool m_limpLimiterCut = false;

        Status m_status;
        std::vector<Event> m_events;
};

#endif /* VRUM_DRIVE_CONTROLLER_H */
