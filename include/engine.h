#ifndef ATG_ENGINE_SIM_ENGINE_H
#define ATG_ENGINE_SIM_ENGINE_H

#include "part.h"

#include "piston.h"
#include "connecting_rod.h"
#include "crankshaft.h"
#include "cylinder_bank.h"
#include "cylinder_head.h"
#include "exhaust_system.h"
#include "ignition_module.h"
#include "intake.h"
#include "combustion_chamber.h"
#include "units.h"
#include "throttle.h"

#include <string>

class Simulator;
class Vehicle;
class Transmission;
class Engine : public Part {
    public:
        struct Parameters {
            int cylinderBanks;
            int cylinderCount;
            int crankshaftCount;
            int exhaustSystemCount;
            int intakeCount;

            std::string name;

            double starterTorque = units::torque(90.0, units::ft_lb);
            double starterSpeed = units::rpm(200);
            double redline = units::rpm(6500);
            double dynoMinSpeed = units::rpm(1000);
            double dynoMaxSpeed = units::rpm(6500);
            double dynoHoldStep = units::rpm(100);

            Throttle *throttle;

            double initialSimulationFrequency;
            double initialHighFrequencyGain;
            double initialNoise;
            double initialJitter;

            // Input-stage low-pass cutoff in Hz; 0 = auto (0.45 * simulation
            // frequency). This is the brightness ceiling of the whole engine note.
            double initialHighFrequencyCutoff = 0.0;
            double initialHighFrequencyBiteCutoff = 0.0;
            // Engine speed window (rad/s) over which the hf_gain bite fades in.
            // max <= min disables the ramp and the bite is always at full value.
            double hfBiteMinSpeed = 0.0;
            double hfBiteMaxSpeed = 0.0;
            // Gas-system substeps per simulation step. Higher = finer fluid
            // timestep but the cost multiplies against simulation_frequency.
            int fluidSimulationSteps = 8;
            // Auto-leveler peak target, out of 32767. The leveler is feedback-only
            // with no lookahead, so a transient bigger than its running peak is
            // emitted at the old gain -- headroom is what stops that clipping.
            double levelerTarget = 30000.0;

            // Overrun ("pop tune") fuel enrichment. On a closed throttle a real
            // tune injects fuel that does not burn in the cylinder, so raw fuel
            // reaches the hot pipe and cracks. Without it, lifting off admits
            // almost no fuel (measured p_fuel 0.005) and nothing can ignite.
            // 0 = off.
            double overrunFuelRate = 0.0;
            double overrunThrottleThreshold = 0.15;   // below this = "lifted"
            double overrunMinSpeed = 0.0;             // rad/s; below this, no pops
            // Seconds of crackle allowed per lift-off. Real decel pops are a
            // burst that fades, not a sound that continues to a standstill.
            // Re-arms only when the throttle is opened again.
            double overrunDuration = 1.2;
            // Fraction of ignition events suppressed while crackling. Must stay
            // well below 1.0 -- a total cut leaves the engine making no power at
            // all, so it drags to a stall and cannot recover without throttle.
            double overrunIgnitionCutFraction = 0.45;
        };

    public:
        Engine();
        virtual ~Engine();

        void initialize(const Parameters &params);
        virtual void destroy();

        std::string getName() const { return m_name; }

        virtual Crankshaft *getOutputCrankshaft() const;
        virtual void setSpeedControl(double s);
        virtual double getSpeedControl();
        virtual void setThrottle(double throttle);
        virtual double getThrottle() const;
        virtual double getThrottlePlateAngle() const;
        virtual void calculateDisplacement();
        double getDisplacement() const { return m_displacement; }
        virtual double getIntakeFlowRate() const;
        virtual void update(double dt);

        virtual double getManifoldPressure() const;
        virtual double getIntakeAfr() const;
        virtual double getExhaustO2() const;
        virtual double getRpm() const;
        virtual double getSpeed() const;
        virtual bool isSpinningCw() const;

        virtual void resetFuelConsumption();
        virtual double getTotalFuelMassConsumed() const;
        double getTotalVolumeFuelConsumed() const;

        inline double getStarterTorque() const { return m_starterTorque; }
        inline double getStarterSpeed() const { return m_starterSpeed; }
        inline double getRedline() const { return m_redline; }
        inline double getDynoMinSpeed() const { return m_dynoMinSpeed; }
        inline double getDynoMaxSpeed() const { return m_dynoMaxSpeed; }
        inline double getDynoHoldStep() const { return m_dynoHoldStep; }

        int getCylinderBankCount() const { return m_cylinderBankCount; }
        int getCylinderCount() const { return m_cylinderCount; }
        int getCrankshaftCount() const { return m_crankshaftCount; }
        int getExhaustSystemCount() const { return m_exhaustSystemCount; }
        int getIntakeCount() const { return m_intakeCount; }
        int getMaxDepth() const;

        Crankshaft *getCrankshaft(int i) const { return &m_crankshafts[i]; }
        CylinderBank *getCylinderBank(int i) const { return &m_cylinderBanks[i]; }
        CylinderHead *getHead(int i) const { return &m_heads[i]; }
        Piston *getPiston(int i) const { return &m_pistons[i]; }
        ConnectingRod *getConnectingRod(int i) const { return &m_connectingRods[i]; }
        IgnitionModule *getIgnitionModule() { return &m_ignitionModule; }
        ExhaustSystem *getExhaustSystem(int i) const { return &m_exhaustSystems[i]; }
        Intake *getIntake(int i) const { return &m_intakes[i]; }
        CombustionChamber *getChamber(int i) const { return &m_combustionChambers[i]; }
        Fuel *getFuel() { return &m_fuel; }

        double getSimulationFrequency() const { return m_initialSimulationFrequency; }
        double getInitialHighFrequencyGain() const { return m_initialHighFrequencyGain; }
        double getInitialNoise() const { return m_initialNoise; }
        double getInitialJitter() const { return m_initialJitter; }
        double getInitialHighFrequencyCutoff() const { return m_initialHighFrequencyCutoff; }
        double getInitialHighFrequencyBiteCutoff() const { return m_initialHighFrequencyBiteCutoff; }
        // Set while a rev-match blip is running. The overrun crackle works by
        // cutting a fraction of the sparks, which is exactly how the rev
        // limiter works too -- so letting it fire during an automatic blip
        // makes the throttle stumble in the driver's hand. The crackle is still
        // wanted on the settle afterwards, just not mid-blip.
        void setOverrunSuppressed(bool suppressed) { m_overrunSuppressed = suppressed; }
        bool getOverrunSuppressed() const { return m_overrunSuppressed; }

        double getHfBiteMinSpeed() const { return m_hfBiteMinSpeed; }
        double getHfBiteMaxSpeed() const { return m_hfBiteMaxSpeed; }
        void setHfBiteSpeedWindow(double minSpeed, double maxSpeed) {
            m_hfBiteMinSpeed = minSpeed;
            m_hfBiteMaxSpeed = maxSpeed;
        }
        int getFluidSimulationSteps() const { return m_fluidSimulationSteps; }
        double getLevelerTarget() const { return m_levelerTarget; }
        double getOverrunFuelRate() const { return m_overrunFuelRate; }
        double getOverrunThrottleThreshold() const { return m_overrunThrottleThreshold; }
        double getOverrunMinSpeed() const { return m_overrunMinSpeed; }
        double getOverrunDuration() const { return m_overrunDuration; }
        double getOverrunIgnitionCutFraction() const {
            return m_overrunIgnitionCutFraction;
        }
        // Runtime master switch for backfires (in-app toggle). Independent of
        // the .mr tuning so it can be silenced without editing anything.
        bool getBackfireEnabled() const { return m_backfireEnabled; }
        void setBackfireEnabled(bool e) { m_backfireEnabled = e; }
        void setOverrunFuelRate(double r) { m_overrunFuelRate = r; }

        virtual Simulator *createSimulator(Vehicle *vehicle, Transmission *transmission);

    protected:
        std::string m_name;

        Crankshaft *m_crankshafts;
        int m_crankshaftCount;

        CylinderBank *m_cylinderBanks;
        CylinderHead *m_heads;
        int m_cylinderBankCount;

        Piston *m_pistons;
        ConnectingRod *m_connectingRods;
        CombustionChamber *m_combustionChambers;
        int m_cylinderCount;

        double m_starterTorque;
        double m_starterSpeed;
        double m_redline;
        double m_dynoMinSpeed;
        double m_dynoMaxSpeed;
        double m_dynoHoldStep;

        double m_initialSimulationFrequency;
        double m_initialHighFrequencyGain;
        double m_initialNoise;
        double m_initialJitter;
        double m_initialHighFrequencyCutoff;
        double m_initialHighFrequencyBiteCutoff;
        bool m_overrunSuppressed = false;
        double m_hfBiteMinSpeed;
        double m_hfBiteMaxSpeed;
        int m_fluidSimulationSteps;
        double m_levelerTarget;
        double m_overrunFuelRate;
        double m_overrunThrottleThreshold;
        double m_overrunMinSpeed;
        double m_overrunDuration;
        double m_overrunIgnitionCutFraction;
        bool m_backfireEnabled = false;

        ExhaustSystem *m_exhaustSystems;
        int m_exhaustSystemCount;

        Intake *m_intakes;
        int m_intakeCount;

        IgnitionModule m_ignitionModule;
        Fuel m_fuel;

        Throttle *m_throttle;

        double m_throttleValue;
        double m_displacement;
};

#endif /* ATG_ENGINE_SIM_ENGINE_H */
