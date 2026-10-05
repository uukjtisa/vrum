#ifndef ATG_ENGINE_SIM_TRANSMISSION_H
#define ATG_ENGINE_SIM_TRANSMISSION_H

#include "vehicle.h"
#include "engine.h"
#include "scs.h"

#include <cmath>

class Transmission {
    public:
        struct Parameters {
            int GearCount = 0;
            const double *GearRatios = nullptr;
            double MaxClutchTorque = 0.0;

            // VRUM: transmission control unit. A downshift multiplies crank
            // speed by the ratio step, and nothing here used to check the
            // result -- a 7->6 step is roughly 1.21:1, so 8000 rpm lands near
            // 9700, past the limiter, and the spark cut turns what should be a
            // clean downshift into a stumble. With this on, the shift is simply
            // refused until it is survivable.
            //
            // Per-engine because it is a property of the car, not the sim: a
            // 1970s manual has no TCU and should be allowed to money-shift.
            bool TcuEnabled = true;
            // How far below the rev limiter a predicted downshift must land, in
            // rad/s. Headroom for the blip's own overshoot.
            double TcuOverRevMargin = 0.0;

            // Seconds for a complete shift: clutch open -> ratio swap -> clutch
            // closed. Shift requests arriving inside this window are refused
            // (the paddle lockout), which is what stops paddle-spam from
            // turning every blip into a clutch dump.
            double ShiftTime = 0.12;
            // Upshift ignition cut (Lamborghini ISR style): spark is cut while
            // the clutch is open so the crank drops to the new gear's speed,
            // and the unburnt charge cracks in a hot pipe.
            bool UpshiftCut = false;
            // Launch control: brake + full throttle at a standstill in 1st
            // holds the engine at LaunchRpm (rad/s) on a soft spark-cut
            // limiter; releasing the brake drops the clutch over LaunchClutchTime.
            bool LaunchControl = false;
            double LaunchRpm = 0.0;
            double LaunchClutchTime = 0.35;
        };

    public:
        Transmission();
        ~Transmission();

        void initialize(const Parameters &params);
        void update(double dt);
        void addToSystem(
            atg_scs::RigidBodySystem *system,
            atg_scs::RigidBody *rotatingMass,
            Vehicle *vehicle,
            Engine *engine);
        void changeGear(int newGear);
        inline int getGear() const { return m_gear; }
        inline void setClutchPressure(double pressure) { m_clutchPressure = pressure; }
        inline double getClutchPressure() const { return m_clutchPressure; }

        // Engine speed (rad/s) the current gear demands. The clutch constraint
        // ties the crankshaft to the rotating mass 1:1, so once engaged the
        // engine must turn at exactly this speed -- which makes it the target a
        // rev-match blip is aiming for.
        double getTargetEngineSpeed() const {
            return (m_gear == -1 || m_rotatingMass == nullptr)
                ? 0.0
                : std::abs(m_rotatingMass->v_theta);
        }

        // What the crankshaft would spin at, in rad/s, if `newGear` were
        // selected right now -- computed WITHOUT committing to the shift.
        //
        // changeGear() conserves the rotating mass's kinetic energy across the
        // ratio change, so the new speed follows from the inertia ratio alone,
        // and the clutch ties the crank to that mass 1:1. Works from neutral
        // too: changeGear() only rewrites `I` when engaging a gear, so in
        // neutral the mass keeps coasting with the last gear's inertia and this
        // still reflects real road speed.
        double predictedEngineSpeed(int newGear) const {
            if (m_rotatingMass == nullptr || m_vehicle == nullptr) return 0.0;
            if (newGear < 0 || newGear >= m_gearCount) return 0.0;

            const double f = m_vehicle->getTireRadius()
                / (m_vehicle->getDiffRatio() * m_gearRatios[newGear]);
            const double new_I = m_vehicle->getMass() * f * f;
            if (new_I <= 0.0 || m_rotatingMass->I <= 0.0) return 0.0;

            return std::abs(m_rotatingMass->v_theta)
                * std::sqrt(m_rotatingMass->I / new_I);
        }

        int getGearCount() const { return m_gearCount; }
        bool isTcuEnabled() const { return m_tcuEnabled; }
        void setTcuEnabled(bool enabled) { m_tcuEnabled = enabled; }
        double getTcuOverRevMargin() const { return m_tcuOverRevMargin; }
        const Parameters &getTcuParameters() const { return m_params; }

    protected:
        atg_scs::ClutchConstraint m_clutchConstraint;
        atg_scs::RigidBody *m_rotatingMass;
        Vehicle *m_vehicle;

        int m_gear;
        int m_newGear;
        int m_gearCount;
        double *m_gearRatios;
        double m_maxClutchTorque;
        double m_clutchPressure;

        bool m_tcuEnabled;
        double m_tcuOverRevMargin;

        // Copy of the script parameters (gear ratio pointer excluded) so the
        // drive controller can read shift timing, ISR and launch settings.
        Parameters m_params;
};

#endif /* ATG_ENGINE_SIM_TRANSMISSION_H */
