#ifndef ATG_ENGINE_SIM_EXHAUST_SYSTEM_H
#define ATG_ENGINE_SIM_EXHAUST_SYSTEM_H

#include "part.h"

#include "gas_system.h"
#include "impulse_response.h"

class Fuel;

class ExhaustSystem : public Part {
    friend class Engine;

    public:
        struct Parameters {
            double length;
            double collectorCrossSectionArea;
            double outletFlowRate;
            double primaryTubeLength;
            double primaryFlowRate;
            double velocityDecay;
            double audioVolume;

            // Backfire / overrun crackle. Unburnt fuel reaching a hot pipe with
            // leftover oxygen ignites, and the heat release is a pressure spike
            // -- an actual pop, not a sample. Defaults to OFF (rate 0) so no
            // existing engine changes behaviour unless it opts in.
            // Fraction of the pipe's contents consumed per ignition event
            // (0..1). This is a burn fraction, not a rate: the charge builds up
            // and then goes off.
            double backfireRate = 0.0;
            double backfireAutoignitionTemperature = 0.0;
            // Fuel mole fraction that must collect before it will light.
            // Higher = rarer but bigger bangs.
            double backfireFuelThreshold = 0.005;
            // Seconds of enforced quiet after an ignition. combust() is called
            // every fluid substep -- 48000 times a second at 12 kHz x 4 -- so
            // without this it re-ignites on every one and becomes a continuous
            // blowtorch instead of discrete pops.
            double backfireRefractory = 0.05;
            // Seconds for the pipe wall to follow the gas temperature. The gas
            // itself goes cold the instant spark is cut, but real steel stays
            // glowing for seconds -- that stored heat is what actually ignites a
            // raw charge. Without it a backfire can never trigger.
            double wallHeatTimeConstant = 2.0;

            ImpulseResponse *impulseResponse;
        };

    public:
        ExhaustSystem();
        virtual ~ExhaustSystem();

        void initialize(const Parameters &params);
        virtual void destroy();

        void process(double dt);

        // Burns whatever unburnt fuel/oxygen is in the pipe if it is hot enough.
        // Returns the mass of fuel consumed (0 when nothing ignites). Separate
        // from process() because it needs the engine's Fuel, which the exhaust
        // system does not own.
        double combust(double dt, const Fuel *fuel);

        // Same ignition logic applied to an arbitrary gas volume, using this
        // system's parameters, wall temperature and refractory state. Used for
        // the per-cylinder runner+primary, which is the volume the audio is
        // actually read from (piston_engine_simulator.cpp) -- a pop that happens
        // only in the downstream collector is barely audible.
        double combustIn(GasSystem &gas, double dt, const Fuel *fuel);

        inline int getIndex() const { return m_index; }
        inline double getBackfireRate() const { return m_backfireRate; }
        inline double getWallTemperature() const { return m_wallTemperature; }

        // Instrumentation: how many discrete backfires have actually fired, and
        // how much fuel the last one burned. Without counting them there is no
        // way to tell a pop from a continuous flame by looking at the audio.
        inline int getBackfireEventCount() const { return m_backfireEventCount; }
        inline double getLastBackfireMass() const { return m_lastBackfireMass; }
        inline void resetBackfireCounters() {
            m_backfireEventCount = 0;
            m_lastBackfireMass = 0.0;
        }
        inline void setBackfireRate(double rate) { m_backfireRate = rate; }
        inline void setBackfireAutoignitionTemperature(double t) {
            m_backfireAutoignitionTemperature = t;
        }
        inline double getLength() const { return m_length; }
        inline double getFlow() const { return m_flow; }
        inline double getAudioVolume() const { return m_audioVolume; }
        inline double getPrimaryFlowRate() const { return m_primaryFlowRate; }
        inline double getCollectorCrossSectionArea() const { return m_collectorCrossSectionArea; }
        inline double getPrimaryTubeLength() const { return m_primaryTubeLength; }
        inline double getVelocityDecay() const { return m_velocityDecay; }
        inline ImpulseResponse *getImpulseResponse() const { return m_impulseResponse; }

        inline GasSystem *getSystem() { return &m_system; }

    protected:
        GasSystem m_atmosphere;
        GasSystem m_system;

        ImpulseResponse *m_impulseResponse;

        double m_length;
        double m_primaryTubeLength;
        double m_collectorCrossSectionArea;
        double m_primaryFlowRate;
        double m_outletFlowRate;
        double m_audioVolume;
        double m_velocityDecay;
        double m_backfireRate;
        double m_backfireAutoignitionTemperature;
        double m_backfireFuelThreshold;
        double m_backfireRefractory;
        double m_backfireCooldown;
        double m_lastBackfireMass;
        int m_backfireEventCount;
        double m_wallHeatTimeConstant;
        double m_wallTemperature;
        int m_index;

        double m_flow;
};

#endif /* ATG_ENGINE_SIM_EXHAUST_SYSTEM_H */
