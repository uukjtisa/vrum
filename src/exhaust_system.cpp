#include "../include/exhaust_system.h"

#include "../include/fuel.h"
#include "../include/units.h"

#include <algorithm>

ExhaustSystem::ExhaustSystem() {
    m_primaryFlowRate = 0;
    m_outletFlowRate = 0;
    m_collectorCrossSectionArea = 0;
    m_length = 0;
    m_primaryTubeLength = 0;
    m_audioVolume = 0;
    m_velocityDecay = 0;
    m_backfireRate = 0;
    m_backfireAutoignitionTemperature = 0;
    m_backfireFuelThreshold = 0.005;
    m_backfireRefractory = 0.05;
    m_backfireCooldown = 0.0;
    m_lastBackfireMass = 0.0;
    m_backfireEventCount = 0;
    m_wallHeatTimeConstant = 2.0;
    m_wallTemperature = units::celcius(25.0);
    m_flow = 0;
    m_index = -1;
    m_impulseResponse = nullptr;
}

ExhaustSystem::~ExhaustSystem() {
    /* void */
}

void ExhaustSystem::initialize(const Parameters &params) {
    const double systemWidth = std::sqrt(params.collectorCrossSectionArea);
    const double volume = params.collectorCrossSectionArea * params.length;
    const double systemLength = params.length;
    m_system.initialize(
            units::pressure(1.0, units::atm),
            volume,
            units::celcius(25.0));
    m_system.setGeometry(
        systemLength,
        systemWidth,
        1.0,
        0.0);

    m_atmosphere.initialize(
        units::pressure(1.0, units::atm),
        units::volume(1000.0, units::m3),
        units::celcius(25.0));
    m_atmosphere.setGeometry(
        units::distance(10.0, units::m),
        units::distance(10.0, units::m),
        1.0,
        0.0);

    m_primaryFlowRate = params.primaryFlowRate;
    m_audioVolume = params.audioVolume;
    m_outletFlowRate = params.outletFlowRate;
    m_collectorCrossSectionArea = params.collectorCrossSectionArea;
    m_velocityDecay = params.velocityDecay;
    m_backfireRate = params.backfireRate;
    m_backfireAutoignitionTemperature = params.backfireAutoignitionTemperature;
    m_backfireFuelThreshold = params.backfireFuelThreshold;
    m_backfireRefractory = params.backfireRefractory;
    m_backfireCooldown = 0.0;
    m_lastBackfireMass = 0.0;
    m_backfireEventCount = 0;
    m_wallHeatTimeConstant = params.wallHeatTimeConstant;
    m_wallTemperature = units::celcius(25.0);
    m_impulseResponse = params.impulseResponse;
    m_length = params.length;
    m_primaryTubeLength = params.primaryTubeLength;
}

void ExhaustSystem::destroy() {
    /* void */
}

double ExhaustSystem::combust(double dt, const Fuel *fuel) {
    return combustIn(m_system, dt, fuel);
}

double ExhaustSystem::combustIn(GasSystem &gas, double dt, const Fuel *fuel) {
    if (m_backfireRate <= 0.0 || fuel == nullptr) return 0.0;

    // An ignition locks the pipe out for a refractory window, otherwise this
    // re-ignites on every call and becomes a continuous flame rather than
    // discrete pops. The countdown itself lives in process(), which runs once
    // per fluid substep -- decrementing it here drained it once per CYLINDER
    // (6x too fast per bank), which measured as 93 pops/sec on lift-off.
    if (m_backfireCooldown > 0.0) return 0.0;

    const GasSystem::Mix mix = gas.mix();
    if (mix.p_fuel <= 0.0 || mix.p_o2 <= 0.0) return 0.0;

    // Ignition comes off the hot PIPE, not the gas. When spark is cut the gas
    // going down the exhaust is cold raw charge (measured: 319 K at the rev
    // limiter) while the steel is still glowing from a second ago. Triggering on
    // gas temperature makes a backfire impossible; triggering on wall
    // temperature is both physically right and what actually pops.
    const double T = m_wallTemperature;
    if (T <= m_backfireAutoignitionTemperature) return 0.0;

    // Raw fuel has to ACCUMULATE to an ignitable concentration before it lights.
    // Burning it continuously as it arrives gives a steady flame and no pop --
    // measured, it only raised crest factor 4.8 -> 5.7 dB with no discrete
    // events. Waiting for a charge to build and then consuming most of it in one
    // step is what makes a bang, and it self-regulates: the burn empties the
    // pipe, so it goes quiet until enough fuel collects again.
    if (mix.p_fuel < m_backfireFuelThreshold) return 0.0;

    const double fuelBurned =
        gas.react(m_backfireRate * gas.n(), mix);
    const double massFuelBurned = fuelBurned * fuel->getMolecularMass();

    // Same energy release path the cylinder uses -- react() only rearranges the
    // mixture, the heat has to be added explicitly.
    gas.changeEnergy(massFuelBurned * fuel->getEnergyDensity());

    if (massFuelBurned > 0.0) {
        m_backfireCooldown = m_backfireRefractory;
        m_lastBackfireMass = massFuelBurned;
        ++m_backfireEventCount;
    }

    return massFuelBurned;
}

void ExhaustSystem::process(double dt) {
    GasSystem::Mix airMix;
    airMix.p_fuel = 0;
    airMix.p_inert = 1.0;
    airMix.p_o2 = 0.0;

    m_atmosphere.reset(units::pressure(1.0, units::atm), units::celcius(25.0), airMix);
    GasSystem::FlowParameters flowParams;
    flowParams.crossSectionArea_0 = m_collectorCrossSectionArea;
    flowParams.crossSectionArea_1 = units::area(10, units::m2);
    flowParams.direction_x = 1.0;
    flowParams.direction_y = 0.0;
    flowParams.dt = dt;
    flowParams.system_0 = &m_atmosphere;
    flowParams.system_1 = &m_system;
    flowParams.k_flow = m_outletFlowRate;

    m_flow = m_system.flow(flowParams);

    // One refractory countdown per substep, regardless of how many cylinders
    // feed this bank.
    if (m_backfireCooldown > 0.0) m_backfireCooldown -= dt;

    // Pipe wall thermal mass: steel lags the gas by seconds, so it stays hot
    // through a spark cut or a closed throttle. This is what a backfire ignites
    // against.
    if (m_wallHeatTimeConstant > 0.0) {
        const double alpha = dt / (dt + m_wallHeatTimeConstant);
        m_wallTemperature +=
            (m_system.temperature() - m_wallTemperature) * alpha;
    }

    m_system.dissipateExcessVelocity();
    m_system.updateVelocity(dt, m_velocityDecay);
}
