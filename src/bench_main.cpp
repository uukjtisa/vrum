/*
 * engine-sim-bench -- headless render-to-WAV for VRUM
 * ---------------------------------------------------
 * Runs the simulation with no window and writes the audio to a WAV, so engine
 * tuning can be swept and measured unattended instead of hand-driven through the
 * GUI. This is the loop AngeTheGreat describes in his video -- render a clip,
 * measure it, change one thing, repeat -- except the measuring is done by
 * scripts/analyze_sound.py rather than by ear.
 *
 * Run it from the build/ directory (like the app) so that "../assets/main.mr"
 * and the impulse-response paths resolve.
 *
 *   Release\engine-sim-bench.exe --rpm 3000 --seconds 5 --out ../workspace/x.wav
 *   Release\engine-sim-bench.exe --set simulation_frequency=24000 --set fluid_simulation_steps=2
 *
 * Holding rpm with the dyno is what makes two clips comparable: it removes
 * engine speed as a variable so a spectrum difference means a tuning difference.
 */

#include "../include/engine.h"
#include "../include/piston_engine_simulator.h"
#include "../include/simulator.h"
#include "../include/transmission.h"
#include "../include/units.h"
#include "../include/vehicle.h"
#include "../include/wav_reader.h"
#include "../include/wav_writer.h"

#include "../scripting/include/compiler.h"

#include <cstring>
#include <iostream>
#include <string>
#include <vector>

namespace {

struct Options {
    std::string script = "../assets/main.mr";
    std::string out = "../workspace/audio_capture/bench.wav";
    std::string label;
    bool debug = false;          // print exhaust gas state while recording
    double rpm = 0.0;            // 0 = don't hold, just idle
    double throttle = 0.0;       // 0..1
    double seconds = 5.0;
    bool driveTest = false;      // run the scripted DriveController test
    double warmup = 3.0;
    // Conditions during warmup, if they differ from the recorded ones. Needed
    // for anything history-dependent: a backfire wants a pipe made hot by real
    // combustion first, then the spark cut or the closed throttle.
    double warmupRpm = -1.0;
    double warmupThrottle = -1.0;
    std::vector<std::pair<std::string, double>> overrides;
};

void printUsage() {
    std::cout <<
        "engine-sim-bench -- headless render-to-WAV\n\n"
        "  --script PATH      engine script (default ../assets/main.mr)\n"
        "  --out PATH         output wav (default ../workspace/audio_capture/bench.wav)\n"
        "  --rpm N            hold this engine speed with the dyno (default: free idle)\n"
        "  --throttle X       0..1 speed control (default 0)\n"
        "  --seconds N        seconds of audio to record (default 5)\n"
        "  --drive-test       scripted launch/shift/TCU/overheat test of the drive controller\n"
        "  --warmup N         seconds to settle before recording (default 3)\n"
        "  --label TEXT       printed in the summary line\n"
        "  --set NAME=VALUE   override a tuning parameter; repeatable. One of:\n"
        "                       simulation_frequency, fluid_simulation_steps,\n"
        "                       hf_gain, hf_cutoff, hf_bite_cutoff, noise, jitter,\n"
        "                       leveler_target, convolution, volume,\n"
        "                       overrun_fuel_rate, backfire_enabled, backfire_rate,\n"
        "                       backfire_temperature, reverb_mix, reverb_room_size,\n"
        "                       reverb_damping, distance_cutoff\n";
}

bool parseArgs(int argc, char **argv, Options &o) {
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        const bool hasNext = (i + 1) < argc;

        auto next = [&]() { return std::string(argv[++i]); };

        if (arg == "--help" || arg == "-h") { printUsage(); return false; }
        else if (arg == "--script" && hasNext) o.script = next();
        else if (arg == "--out" && hasNext) o.out = next();
        else if (arg == "--label" && hasNext) o.label = next();
        else if (arg == "--debug") o.debug = true;
        else if (arg == "--rpm" && hasNext) o.rpm = std::stod(next());
        else if (arg == "--throttle" && hasNext) o.throttle = std::stod(next());
        else if (arg == "--seconds" && hasNext) o.seconds = std::stod(next());
        else if (arg == "--drive-test") o.driveTest = true;
        else if (arg == "--warmup" && hasNext) o.warmup = std::stod(next());
        else if (arg == "--warmup-rpm" && hasNext) o.warmupRpm = std::stod(next());
        else if (arg == "--warmup-throttle" && hasNext) o.warmupThrottle = std::stod(next());
        else if (arg == "--set" && hasNext) {
            const std::string kv = next();
            const size_t eq = kv.find('=');
            if (eq == std::string::npos) {
                std::cerr << "bad --set (expected NAME=VALUE): " << kv << "\n";
                return false;
            }
            o.overrides.emplace_back(kv.substr(0, eq), std::stod(kv.substr(eq + 1)));
        }
        else {
            std::cerr << "unknown argument: " << arg << "\n";
            printUsage();
            return false;
        }
    }

    return true;
}

// Mirrors EngineSimApplication::loadEngine's impulse-response loading, minus
// delta-studio: the app uses ysWindowsAudioWaveFile, we use WavReader.
bool loadImpulseResponses(Engine *engine, Simulator *simulator) {
    for (int i = 0; i < engine->getExhaustSystemCount(); ++i) {
        ImpulseResponse *response = engine->getExhaustSystem(i)->getImpulseResponse();

        WavReader reader;
        if (!reader.open(response->getFilename())) {
            std::cerr << "impulse response " << response->getFilename()
                      << ": " << reader.getError() << "\n";
            return false;
        }

        simulator->synthesizer().initializeImpulseResponse(
            reader.getSamples().data(),
            static_cast<unsigned int>(reader.getSamples().size()),
            static_cast<float>(response->getVolume()),
            i);
    }

    return true;
}

void applyOverrides(const Options &o, Engine *engine, Simulator *simulator) {
    Synthesizer::AudioParameters audio = simulator->synthesizer().getAudioParameters();
    audio.inputSampleNoise = static_cast<float>(engine->getInitialJitter());
    audio.airNoise = static_cast<float>(engine->getInitialNoise());
    audio.dF_F_mix = static_cast<float>(engine->getInitialHighFrequencyGain());
    audio.inputFilterCutoff = static_cast<float>(engine->getInitialHighFrequencyCutoff());
    audio.hfBiteCutoff = static_cast<float>(engine->getInitialHighFrequencyBiteCutoff());
    audio.levelerTarget = static_cast<float>(engine->getLevelerTarget());

    PistonEngineSimulator *pistonSim =
        dynamic_cast<PistonEngineSimulator *>(simulator);

    for (const auto &kv : o.overrides) {
        const std::string &name = kv.first;
        const double v = kv.second;

        if (name == "simulation_frequency") simulator->setSimulationFrequency((int)v);
        else if (name == "fluid_simulation_steps") {
            if (pistonSim != nullptr) pistonSim->setFluidSimulationSteps((int)v);
        }
        else if (name == "hf_gain") audio.dF_F_mix = (float)v;
        else if (name == "hf_cutoff") audio.inputFilterCutoff = (float)v;
        else if (name == "hf_bite_cutoff") audio.hfBiteCutoff = (float)v;
        else if (name == "hf_bite_min_rpm") {
            engine->setHfBiteSpeedWindow(
                units::rpm(v), engine->getHfBiteMaxSpeed());
        }
        else if (name == "hf_bite_max_rpm") {
            engine->setHfBiteSpeedWindow(
                engine->getHfBiteMinSpeed(), units::rpm(v));
        }
        else if (name == "noise") audio.airNoise = (float)v;
        else if (name == "jitter") audio.inputSampleNoise = (float)v;
        else if (name == "leveler_target") audio.levelerTarget = (float)v;
        else if (name == "overrun_fuel_rate") engine->setOverrunFuelRate(v);
        else if (name == "backfire_enabled") engine->setBackfireEnabled(v != 0.0);
        else if (name == "backfire_rate") {
            for (int i = 0; i < engine->getExhaustSystemCount(); ++i) {
                engine->getExhaustSystem(i)->setBackfireRate(v);
            }
        }
        else if (name == "backfire_temperature") {
            for (int i = 0; i < engine->getExhaustSystemCount(); ++i) {
                engine->getExhaustSystem(i)->setBackfireAutoignitionTemperature(v);
            }
        }
        else if (name == "convolution") audio.convolution = (float)v;
        else if (name == "volume") audio.volume = (float)v;
        // Outdoor perspective. Exposed here so the reverb can be measured
        // rather than judged by ear -- the first version was a flutter echo
        // and nothing in the harness could have caught it.
        else if (name == "reverb_mix") audio.reverbMix = (float)v;
        else if (name == "reverb_room_size") audio.reverbRoomSize = (float)v;
        else if (name == "reverb_damping") audio.reverbDamping = (float)v;
        else if (name == "reverb_early") audio.reverbEarly = (float)v;
        else if (name == "distance_cutoff") audio.distanceCutoff = (float)v;
        else std::cerr << "warning: unknown --set name '" << name << "', ignored\n";
    }

    simulator->synthesizer().setAudioParameters(audio);
}

// Advances the simulation by one frame and drains whatever audio it produced.
// Returns the number of samples drained.
int stepFrame(Simulator *simulator, double dt, std::vector<int16_t> &buffer) {
    simulator->startFrame(dt);
    while (simulator->simulateStep()) { /* run the frame's iterations */ }
    simulator->endFrame();

    return simulator->readAudioOutput(
        static_cast<int>(buffer.size()), buffer.data());
}

} // namespace

#include "bench_drive_test.inl"

int main(int argc, char **argv) {
    Options o;
    if (!parseArgs(argc, argv, o)) return 1;

    es_script::Compiler compiler;
    compiler.initialize();
    if (!compiler.compile(o.script.c_str())) {
        std::cerr << "failed to compile " << o.script << "\n";
        return 1;
    }

    const es_script::Compiler::Output output = compiler.execute();
    Engine *engine = output.engine;
    Vehicle *vehicle = output.vehicle;
    Transmission *transmission = output.transmission;

    if (engine == nullptr || vehicle == nullptr || transmission == nullptr) {
        std::cerr << "script did not produce an engine/vehicle/transmission\n";
        return 1;
    }

    Simulator *simulator = engine->createSimulator(vehicle, transmission);
    simulator->setSimulationFrequency(static_cast<int>(engine->getSimulationFrequency()));

    applyOverrides(o, engine, simulator);
    if (!loadImpulseResponses(engine, simulator)) return 1;

    simulator->startAudioRenderingThread();
    simulator->setSimulationSpeed(1.0);

    // Crank until it catches, exactly as holding S does in the GUI.
    engine->getIgnitionModule()->m_enabled = true;
    simulator->m_starterMotor.m_enabled = true;
    transmission->changeGear(-1);           // neutral: no drivetrain load
    transmission->setClutchPressure(0.0);

    const double dt = 1.0 / 60.0;
    std::vector<int16_t> buffer(8192);

    for (int frame = 0; frame < 60 * 8; ++frame) {
        stepFrame(simulator, dt, buffer);
        if (units::toRpm(engine->getSpeed()) > 700.0 && frame > 30) break;
    }
    simulator->m_starterMotor.m_enabled = false;

    if (o.driveTest) {
        const int result = runDriveTest(simulator, engine, buffer);
        simulator->endAudioRenderingThread();
        return result;
    }

    // Warm up under the warmup conditions (which default to the recorded ones).
    const double warmRpm = (o.warmupRpm >= 0.0) ? o.warmupRpm : o.rpm;
    const double warmThrottle =
        (o.warmupThrottle >= 0.0) ? o.warmupThrottle : o.throttle;

    engine->setSpeedControl(warmThrottle);
    if (warmRpm > 0.0) {
        simulator->m_dyno.m_enabled = true;
        simulator->m_dyno.m_hold = true;
        simulator->m_dyno.m_rotationSpeed = units::rpm(warmRpm);
    }

    for (int frame = 0; frame < (int)(o.warmup * 60); ++frame) {
        stepFrame(simulator, dt, buffer);
    }

    // Switch to the conditions actually being recorded. Counters reset here so
    // the reported backfire rate describes the recorded window, not the warmup.
    for (int i = 0; i < engine->getExhaustSystemCount(); ++i) {
        engine->getExhaustSystem(i)->resetBackfireCounters();
    }

    engine->setSpeedControl(o.throttle);
    if (o.rpm > 0.0) {
        simulator->m_dyno.m_enabled = true;
        simulator->m_dyno.m_hold = true;
        simulator->m_dyno.m_rotationSpeed = units::rpm(o.rpm);
    }

    WavWriter wav;
    if (!wav.open(o.out, 44100, 1)) {
        std::cerr << "could not open output " << o.out << "\n";
        return 1;
    }

    const int targetSamples = static_cast<int>(o.seconds * 44100);
    int written = 0;
    int guardFrames = 0;
    const int maxFrames = static_cast<int>(o.seconds * 60 * 20) + 600;

    double peakExhaustTemp = 0.0, peakFuel = 0.0, peakO2 = 0.0, peakBurn = 0.0;

    while (written < targetSamples && guardFrames++ < maxFrames) {
        const int n = stepFrame(simulator, dt, buffer);
        if (n > 0) {
            wav.write(buffer.data(), n);
            written += n;
        }

        if (o.debug) {
            // Backfire needs three things at once: fuel, oxygen, and heat.
            // Tracking the peaks shows which one is missing.
            GasSystem *gas = engine->getExhaustSystem(0)->getSystem();
            const double T = gas->temperature();
            if (T > peakExhaustTemp) peakExhaustTemp = T;
            if (gas->mix().p_fuel > peakFuel) peakFuel = gas->mix().p_fuel;
            if (gas->mix().p_o2 > peakO2) peakO2 = gas->mix().p_o2;

            const double wall = engine->getExhaustSystem(0)->getWallTemperature();
            if (wall > peakBurn) peakBurn = wall;
        }
    }

    if (o.debug) {
        int backfires = 0;
        for (int i = 0; i < engine->getExhaustSystemCount(); ++i) {
            backfires += engine->getExhaustSystem(i)->getBackfireEventCount();
        }

        const double seconds = written / 44100.0;
        std::cout << "  exhaust: gasT=" << peakExhaustTemp << " K"
                  << "  wallT=" << peakBurn << " K"
                  << "  p_fuel=" << peakFuel
                  << "  p_o2=" << peakO2 << "\n"
                  << "  BACKFIRES: " << backfires << " events in "
                  << seconds << "s = " << (backfires / seconds)
                  << "/sec  (a real car pops a few times a second at most;"
                     " thousands means it is a continuous flame)\n";
    }

    const double finalRpm = units::toRpm(engine->getSpeed());
    wav.close();
    simulator->endAudioRenderingThread();

    std::cout << (o.label.empty() ? o.out : o.label)
              << " -- wrote " << written << " samples ("
              << (written / 44100.0) << "s) at "
              << finalRpm << " rpm -> " << o.out << "\n";

    if (written < targetSamples) {
        std::cerr << "warning: stopped early (frame guard hit); the sim may not "
                     "be producing audio\n";
        return 2;
    }

    return 0;
}
