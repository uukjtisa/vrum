#include "../include/synthesizer.h"

#include "../include/utilities.h"
#include "../include/delta.h"

#include <cassert>
#include <cmath>

#undef min
#undef max

Synthesizer::Synthesizer() {
    m_inputChannels = nullptr;
    m_inputChannelCount = 0;
    m_inputBufferSize = 0;
    m_inputWriteOffset = 0.0;
    m_inputSamplesRead = 0;

    m_audioBufferSize = 0;

    m_inputSampleRate = 0.0;
    m_audioSampleRate = 0.0;

    m_targetInputFilterCutoff = 0.0f;
    m_appliedInputFilterCutoff = -1.0f;

    m_lastInputSampleOffset = 0.0;

    m_run = true;
    m_thread = nullptr;
    m_filters = nullptr;
}

Synthesizer::~Synthesizer() {
    assert(m_inputChannels == nullptr);
    assert(m_thread == nullptr);
    assert(m_filters == nullptr);
}

void Synthesizer::initialize(const Parameters &p) {
    m_inputChannelCount = p.inputChannelCount;
    m_inputBufferSize = p.inputBufferSize;
    m_inputWriteOffset = p.inputBufferSize;
    m_audioBufferSize = p.audioBufferSize;
    m_inputSampleRate = p.inputSampleRate;
    m_audioSampleRate = p.audioSampleRate;
    m_audioParameters = p.initialAudioParameters;

    m_inputSamplesRead = 0;

    m_inputWriteOffset = 0;
    m_processed = true;

    m_audioBuffer.initialize(p.audioBufferSize);
    m_inputChannels = new InputChannel[p.inputChannelCount];
    for (int i = 0; i < p.inputChannelCount; ++i) {
        m_inputChannels[i].transferBuffer = new float[p.inputBufferSize];
        m_inputChannels[i].data.initialize(p.inputBufferSize);
    }

    m_filters = new ProcessingFilters[p.inputChannelCount];
    for (int i = 0; i < p.inputChannelCount; ++i) {
        m_filters[i].airNoiseLowPass.setCutoffFrequency(
            m_audioParameters.airNoiseFrequencyCutoff, m_audioSampleRate);

        m_filters[i].derivative.m_dt = 1 / m_audioSampleRate;

        m_filters[i].inputDcFilter.setCutoffFrequency(10.0);
        m_filters[i].inputDcFilter.m_dt = 1 / m_audioSampleRate;

        m_filters[i].jitterFilter.initialize(
            10,
            m_audioParameters.inputSampleNoiseFrequencyCutoff,
            m_audioSampleRate);

    }

    // Was a hard-coded 1900 Hz here. See AudioParameters::inputFilterCutoff.
    m_targetInputFilterCutoff = m_audioParameters.inputFilterCutoff;
    m_appliedInputFilterCutoff = -1.0f;
    updateInputFilterCutoff();

    m_levelingFilter.p_target = m_audioParameters.levelerTarget;
    m_levelingFilter.p_maxLevel = m_audioParameters.levelerMaxGain;
    m_levelingFilter.p_minLevel = m_audioParameters.levelerMinGain;
    m_antialiasing.setCutoffFrequency(m_audioSampleRate * 0.45f, m_audioSampleRate);

    m_reverb.initialize(m_audioSampleRate);
    m_reverb.setRoomSize(m_audioParameters.reverbRoomSize);
    m_reverb.setDamping(m_audioParameters.reverbDamping);
    m_reverb.setEarlyLevel(m_audioParameters.reverbEarly);
    m_appliedRoomSize = m_audioParameters.reverbRoomSize;
    m_appliedDamping = m_audioParameters.reverbDamping;
    m_appliedEarly = m_audioParameters.reverbEarly;

    for (int i = 0; i < m_audioBufferSize; ++i) {
        m_audioBuffer.write(0);
    }
}

void Synthesizer::initializeImpulseResponse(
    const int16_t *impulseResponse,
    unsigned int samples,
    float volume,
    int index)
{
    unsigned int clippedLength = 0;
    for (unsigned int i = 0; i < samples; ++i) {
        if (std::abs(impulseResponse[i]) > 100) {
            clippedLength = i + 1;
        }
    }

    const unsigned int sampleCount = std::min(10000U, clippedLength);
    m_filters[index].convolution.initialize(sampleCount);
    for (unsigned int i = 0; i < sampleCount; ++i) {
        m_filters[index].convolution.getImpulseResponse()[i] =
            volume * impulseResponse[i] / INT16_MAX;
    }
}

void Synthesizer::startAudioRenderingThread() {
    m_run = true;
    m_thread = new std::thread(&Synthesizer::audioRenderingThread, this);
}

void Synthesizer::endAudioRenderingThread() {
    if (m_thread != nullptr) {
        m_run = false;
        endInputBlock();

        m_thread->join();
        delete m_thread;

        m_thread = nullptr;
    }
}

void Synthesizer::destroy() {
    m_audioBuffer.destroy();

    for (int i = 0; i < m_inputChannelCount; ++i) {
        m_inputChannels[i].data.destroy();
        m_filters[i].convolution.destroy();
    }

    m_reverb.destroy();

    delete[] m_inputChannels;
    delete[] m_filters;

    m_inputChannels = nullptr;
    m_filters = nullptr;

    m_inputChannelCount = 0;
}

int Synthesizer::readAudioOutput(int samples, int16_t *buffer) {
    std::lock_guard<std::mutex> lock(m_lock0);

    const int newDataLength = m_audioBuffer.size();
    if (newDataLength >= samples) {
        m_audioBuffer.readAndRemove(samples, buffer);
    }
    else {
        m_audioBuffer.readAndRemove(newDataLength, buffer);
        memset(
            buffer + newDataLength,
            0,
            sizeof(int16_t) * ((size_t)samples - newDataLength));
    }
    
    const int samplesConsumed = std::min(samples, newDataLength);

    return samplesConsumed;
}

void Synthesizer::waitProcessed() {
    {
        std::unique_lock<std::mutex> lk(m_lock0);
        m_cv0.wait(lk, [this] { return m_processed; });
    }
}

float Synthesizer::effectiveInputFilterCutoff() const {
    // The signal arriving here is linearly interpolated from m_inputSampleRate up to
    // m_audioSampleRate, so nothing above the simulation Nyquist is real -- anything
    // past it is interpolation imaging. Cap at 0.45 * inputSampleRate, matching what
    // the output stage does with the audio rate.
    const float ceiling = std::max(100.0f, 0.45f * m_inputSampleRate);
    const float requested = m_targetInputFilterCutoff.load(std::memory_order_relaxed);
    const float cutoff = (requested > 0.0f) ? requested : ceiling;

    return std::min(std::max(100.0f, cutoff), ceiling);
}

void Synthesizer::updateInputFilterCutoff() {
    const float cutoff = effectiveInputFilterCutoff();
    if (cutoff == m_appliedInputFilterCutoff) return;

    for (int i = 0; i < m_inputChannelCount; ++i) {
        m_filters[i].antialiasing.setCutoffFrequency(cutoff, m_audioSampleRate);
    }

    m_appliedInputFilterCutoff = cutoff;
}

void Synthesizer::writeInput(const double *data) {
    updateInputFilterCutoff();

    m_inputWriteOffset += (double)m_audioSampleRate / m_inputSampleRate;
    if (m_inputWriteOffset >= (double)m_inputBufferSize) {
        m_inputWriteOffset -= (double)m_inputBufferSize;
    }

    for (int i = 0; i < m_inputChannelCount; ++i) {
        RingBuffer<float> &buffer = m_inputChannels[i].data;
        double *history = m_inputChannels[i].history;

        // Slide the newest simulator sample in. Interpolation then runs one
        // input sample behind the simulation so the trailing tangent is
        // available; at 12 kHz that is 83 us of added latency.
        history[0] = history[1];
        history[1] = history[2];
        history[2] = history[3];
        history[3] = data[i];

        const double p0 = history[0], p1 = history[1];
        const double p2 = history[2], p3 = history[3];

        const size_t baseIndex = buffer.writeIndex();
        const double distance =
            inputDistance(m_inputWriteOffset, m_lastInputSampleOffset);
        double s =
            inputDistance(baseIndex, m_lastInputSampleOffset);
        for (; s <= distance; s += 1.0) {
            if (s >= m_inputBufferSize) s -= m_inputBufferSize;

            // Catmull-Rom through p1 and p2.
            const double f = s / distance;
            const double f2 = f * f;
            const double f3 = f2 * f;
            const double sample = 0.5 * (
                (2.0 * p1)
                + (-p0 + p2) * f
                + (2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3) * f2
                + (-p0 + 3.0 * p1 - 3.0 * p2 + p3) * f3);

            buffer.write(m_filters[i].antialiasing.fast_f(static_cast<float>(sample)));
        }

        m_inputChannels[i].lastInputSample = p2;
    }

    m_lastInputSampleOffset = m_inputWriteOffset;
}

void Synthesizer::endInputBlock() {
    std::unique_lock<std::mutex> lk(m_inputLock); 

    for (int i = 0; i < m_inputChannelCount; ++i) {
        m_inputChannels[i].data.removeBeginning(m_inputSamplesRead);
    }

    if (m_inputChannelCount != 0) {
        m_latency = m_inputChannels[0].data.size();
    }
    
    m_inputSamplesRead = 0;
    m_processed = false;

    lk.unlock();
    m_cv0.notify_one();
}

void Synthesizer::audioRenderingThread() {
    while (m_run) {
        renderAudio();
    }
}

#undef max
void Synthesizer::renderAudio() {
    std::unique_lock<std::mutex> lk0(m_lock0);

    m_cv0.wait(lk0, [this] {
        const bool inputAvailable =
            m_inputChannels[0].data.size() > 0
            && m_audioBuffer.size() < 2000;
        return !m_run || (inputAvailable && !m_processed);
    });

    const int n = std::min(
        std::max(0, 2000 - (int)m_audioBuffer.size()),
        (int)m_inputChannels[0].data.size());

    for (int i = 0; i < m_inputChannelCount; ++i) {
        m_inputChannels[i].data.read(n, m_inputChannels[i].transferBuffer);
    }
    
    m_inputSamplesRead = n;
    m_processed = true;

    lk0.unlock();

    // Flush the tail on the audio thread rather than from whoever toggled the
    // perspective; the comb buffers are read here every sample, so clearing
    // them from another thread would be a data race. Without this, switching
    // the perspective back on replays whatever was left in the delay lines.
    if (m_reverbResetRequested.exchange(false, std::memory_order_relaxed)) {
        m_reverb.reset();
    }

    // Perspective coefficients only change when the user moves them, so only
    // recompute on a real change -- setCutoffFrequency runs a tan() and the
    // reverb setters touch every comb.
    if (m_audioParameters.reverbRoomSize != m_appliedRoomSize) {
        m_reverb.setRoomSize(m_audioParameters.reverbRoomSize);
        m_appliedRoomSize = m_audioParameters.reverbRoomSize;
    }
    if (m_audioParameters.reverbDamping != m_appliedDamping) {
        m_reverb.setDamping(m_audioParameters.reverbDamping);
        m_appliedDamping = m_audioParameters.reverbDamping;
    }
    if (m_audioParameters.reverbEarly != m_appliedEarly) {
        m_reverb.setEarlyLevel(m_audioParameters.reverbEarly);
        m_appliedEarly = m_audioParameters.reverbEarly;
    }

    const float distance = m_audioParameters.distanceCutoff;
    m_distanceFilterActive = (distance > 0.0f);
    if (m_distanceFilterActive && distance != m_appliedDistanceCutoff) {
        const float ceiling = 0.45f * m_audioSampleRate;
        const float cutoff = (distance > ceiling) ? ceiling : distance;
        m_distanceFilter.setCutoffFrequency(cutoff, m_audioSampleRate);
        m_appliedDistanceCutoff = distance;
    }

    // Never let the bite filter reach the output Nyquist, or the Butterworth
    // coefficients blow up.
    const float biteRequested = m_audioParameters.hfBiteCutoff;
    m_biteFilterActive = (biteRequested > 0.0f);

    for (int i = 0; i < m_inputChannelCount; ++i) {
        m_filters[i].airNoiseLowPass.setCutoffFrequency(
            static_cast<float>(m_audioParameters.airNoiseFrequencyCutoff), m_audioSampleRate);
        m_filters[i].jitterFilter.setJitterScale(m_audioParameters.inputSampleNoise);

        if (m_biteFilterActive) {
            const float ceiling = 0.45f * m_audioSampleRate;
            const float bite = (biteRequested > ceiling) ? ceiling : biteRequested;
            m_filters[i].derivativeLowPass.setCutoffFrequency(bite, m_audioSampleRate);
        }
    }

    for (int i = 0; i < n; ++i) {
        m_audioBuffer.write(renderAudio(i));
    }

    m_cv0.notify_one();
}

double Synthesizer::getLatency() const {
    return (double)m_latency / m_audioSampleRate;
}

int Synthesizer::inputDelta(int s1, int s0) const {
    return (s1 < s0)
        ? m_inputBufferSize - s0 + s1
        : s1 - s0;
}

double Synthesizer::inputDistance(double s1, double s0) const {
    return (s1 < s0)
        ? (double)m_inputBufferSize - s0 + s1
        : s1 - s0;
}

void Synthesizer::setInputSampleRate(double sampleRate) {
    if (sampleRate != m_inputSampleRate) {
        std::lock_guard<std::mutex> lock(m_lock0);
        m_inputSampleRate = sampleRate;
    }
}

int16_t Synthesizer::renderAudio(int inputSample) {
    const float airNoise = m_audioParameters.airNoise;
    const float biteScale = m_biteScale.load(std::memory_order_relaxed);
    const float dF_F_mix = m_audioParameters.dF_F_mix * biteScale;
    const float convAmount = m_audioParameters.convolution;

    float signal = 0;
    for (int i = 0; i < m_inputChannelCount; ++i) {
        const float r_0 = 2.0 * ((double)rand() / RAND_MAX) - 1.0;

        const float jitteredSample =
            m_filters[i].jitterFilter.fast_f(m_inputChannels[i].transferBuffer[inputSample]);

        const float f_in = jitteredSample;
        const float f_dc = m_filters[i].inputDcFilter.fast_f(f_in);
        const float f = f_in - f_dc;
        const float f_p_raw = m_filters[i].derivative.f(f_in);
        const float f_p = m_biteFilterActive
            ? m_filters[i].derivativeLowPass.fast_f(f_p_raw)
            : f_p_raw;

        const float noise = 2.0 * ((double)rand() / RAND_MAX) - 1.0;
        const float r =
            m_filters->airNoiseLowPass.fast_f(noise);
        const float r_mixed =
            airNoise * r + (1 - airNoise);

        float v_in =
            f_p * dF_F_mix
            + f * r_mixed * (1 - dF_F_mix);
        if (fpclassify(v_in) == FP_SUBNORMAL) {
            v_in = 0;
        }

        const float v =
            convAmount * m_filters[i].convolution.f(v_in)
            + (1 - convAmount) * v_in;

        signal += v;
    }

    signal = m_antialiasing.fast_f(signal);

    // Listening perspective. Runs before the leveler so the leveler still sees
    // (and controls) the final signal including its tail -- placed after it,
    // a long reverb would ride on top of the level the leveler had already set
    // and clip.
    const float reverbMix = m_audioParameters.reverbMix;
    if (reverbMix > 0.0f) {
        const float wet = m_reverb.f(signal);
        signal = signal * (1.0f - reverbMix) + wet * reverbMix;
    }

    if (m_distanceFilterActive) {
        signal = m_distanceFilter.fast_f(signal);
    }

    m_levelingFilter.p_target = m_audioParameters.levelerTarget;
    const float v_leveled = m_levelingFilter.f(signal) * m_audioParameters.volume;
    int r_int = std::lround(v_leveled);
    if (r_int > INT16_MAX) {
        r_int = INT16_MAX;
    }
    else if (r_int < INT16_MIN) {
        r_int = INT16_MIN;
    }

    return static_cast<int16_t>(r_int);
}

double Synthesizer::getLevelerGain() {
    std::lock_guard<std::mutex> lock(m_lock0);
    return m_levelingFilter.getAttenuation();
}

Synthesizer::AudioParameters Synthesizer::getAudioParameters() {
    std::lock_guard<std::mutex> lock(m_lock0);
    return m_audioParameters;
}

void Synthesizer::setAudioParameters(const AudioParameters &params) {
    std::lock_guard<std::mutex> lock(m_lock0);
    m_audioParameters = params;

    // Picked up by the simulation thread on its next writeInput().
    m_targetInputFilterCutoff.store(
        params.inputFilterCutoff, std::memory_order_relaxed);
}
