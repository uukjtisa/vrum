#ifndef ATG_ENGINE_SIM_IGNITION_MODULE_H
#define ATG_ENGINE_SIM_IGNITION_MODULE_H

#include "part.h"

#include "crankshaft.h"
#include "function.h"
#include "units.h"

class IgnitionModule : public Part {
    public:
        struct Parameters {
            int cylinderCount;
            Crankshaft *crankshaft;
            Function *timingCurve;
            double revLimit = units::rpm(6000.0);
            double limiterDuration = 0.5 * units::sec;
        };

        struct SparkPlug {
            double angle = 0;
            bool ignitionEvent = false;
            bool enabled = false;
        };

    public:
        IgnitionModule();
        virtual ~IgnitionModule();

        virtual void destroy();

        void initialize(const Parameters &params);
        void setFiringOrder(int cylinderIndex, double angle);
        void reset();
        void update(double dt);
        bool cutEvent() const;

        bool getIgnitionEvent(int index) const;
        void resetIgnitionEvents();

        double getTimingAdvance();

        bool m_enabled;
        // Overrun ignition cut ("pop tune" / anti-lag). Kept separate from
        // m_enabled, which is the user's ignition switch. Cutting spark pumps
        // the charge out unburnt, carrying BOTH fuel and oxygen into the hot
        // pipe -- on a closed throttle the cylinder otherwise consumes the
        // oxygen, and react() needs ~12.5x more oxygen than fuel, so decel
        // could never crackle.
        //
        // This is a FRACTION (0..1), not a flag. Cutting 100% of events means
        // the engine makes no power at all while lifted, so it drags itself to
        // a stall and will not restart without throttle -- which is exactly
        // what a total cut did. A real pop tune kills only some events, so the
        // engine keeps running while still dumping raw charge.
        double m_ignitionCutFraction = 0.0;

        // Spark cut requested by the drive controller (upshift ISR cut, launch
        // and limp-mode soft limiters). Separate from the overrun cut above
        // because the simulator rewrites that one every fluid step; the
        // effective cut is the larger of the two.
        double m_tcuCutFraction = 0.0;

        // The speed the limiter actually acts on. This is `rev_limit` in the
        // script and is NOT the same number as the engine's `redline`, which is
        // only a gauge marking -- anything that needs to stay clear of the
        // limiter has to ask for this one.
        double getRevLimit() const { return m_revLimit; }

    protected:
        SparkPlug *getPlug(int i);

        Function *m_timingCurve;
        SparkPlug *m_plugs;
        Crankshaft *m_crankshaft;
        int m_cylinderCount;

        double m_lastCrankshaftAngle;
        double m_revLimit;
        double m_revLimitTimer;
        double m_limiterDuration;
};

#endif /* ATG_ENGINE_SIM_IGNITION_MODULE_H */
