#ifndef ATG_ENGINE_SIM_ENGINE_SYNTHESIZER_H
#define ATG_ENGINE_SIM_ENGINE_SYNTHESIZER_H

#include "convolution_filter.h"
#include "leveling_filter.h"
#include "derivative_filter.h"
#include "low_pass_filter.h"
#include "jitter_filter.h"
#include "ring_buffer.h"
#include "butterworth_low_pass_filter.h"
#include "reverb_filter.h"

#include <cinttypes>
#include <thread>
#include <mutex>
#include <atomic>
#include <condition_variable>

class Synthesizer {
    public:
        struct AudioParameters {
            float volume = 1.0f;
            float convolution = 1.0f;
            float dF_F_mix = 0.01f;
            float inputSampleNoise = 0.5f;
            float inputSampleNoiseFrequencyCutoff = 10000.0f;
            float airNoise = 1.0f;
            float airNoiseFrequencyCutoff = 2000.0f;
            float levelerTarget = 30000.0f;
            float levelerMaxGain = 1.9f;
            float levelerMinGain = 0.00001f;

            // Cutoff (Hz) of the low-pass applied to the simulator signal as it is
            // resampled up to the audio rate in writeInput(). This runs BEFORE the
            // derivative/dF_F_mix stage and before convolution, so it is a hard
            // ceiling on the brightness of the entire engine note -- upstream
            // hard-coded it at 1900 Hz, which low-passes the scream out of any
            // engine running above a ~4 kHz simulation Nyquist.
            // 0 = auto: 0.45 * inputSampleRate (mirrors the output stage).
            float inputFilterCutoff = 0.0f;

            // Cutoff (Hz) of the low-pass on the DERIVATIVE path only, i.e. on the
            // dF_F_mix "bite" term, applied before it is mixed back in.
            //
            // A derivative is a +6 dB/octave amplifier, so it boosts whatever sits
            // highest in the signal -- and above the firing harmonics that is the
            // fluid solver's own sample-to-sample hash, not engine sound. The
            // signal grows with rpm but that hash does not, so at idle the
            // amplified hash buries the exhaust note as a metallic clank while at
            // full throttle it is inaudible.
            //
            // Filtering here rather than via inputFilterCutoff is deliberate: it
            // decouples "how much real bandwidth the engine note keeps" from "how
            // far up the differentiator is allowed to amplify". Keep this above
            // the highest firing harmonic worth hearing (H5 at redline) and below
            // the solver's noise region. 0 = off (raw derivative, upstream
            // behaviour).
            float hfBiteCutoff = 0.0f;

            // "Outdoor" listening perspective. 0 = close-mic (the raw exhaust,
            // upstream behaviour); higher values place the engine in a space.
            // See reverb_filter.h for why this matters -- the engine model
            // cannot produce any of it, and it is most of what makes a real
            // recording sound like a car rather than a bench.
            float reverbMix = 0.0f;
            float reverbRoomSize = 0.7f;
            float reverbDamping = 0.45f;
            // Level of the discrete early reflections. High for a tunnel (hard
            // walls a few metres away), low outdoors (nothing close by).
            float reverbEarly = 0.0f;

            // Low-pass on the whole signal when listening from a distance. Air
            // absorption is frequency-dependent, so a distant engine is not
            // just quieter, it is duller -- measured against the reference
            // clips, a far-off pass has under 1% of its energy above 2 kHz
            // versus 27% close up. 0 = off.
            float distanceCutoff = 0.0f;
        };

        struct Parameters {
            int inputChannelCount = 1;
            int inputBufferSize = 1024;
            int audioBufferSize = 44100;
            float inputSampleRate = 10000;
            float audioSampleRate = 44100;
            AudioParameters initialAudioParameters;
        };

        struct InputChannel {
            RingBuffer<float> data;
            float *transferBuffer = nullptr;
            double lastInputSample = 0.0f;

            // Four-point window for Catmull-Rom upsampling: the segment being
            // drawn is history[1] -> history[2], and the outer two points supply
            // the tangents. Straight linear interpolation is only C0, so its
            // derivative is a staircase -- and hf_gain differentiates this
            // signal, which turned that staircase into an audible metallic buzz
            // that buried the exhaust note at idle. A cubic is C1, so the
            // derivative stays continuous.
            double history[4] = { 0.0, 0.0, 0.0, 0.0 };
        };

        struct ProcessingFilters {
            ConvolutionFilter convolution;
            DerivativeFilter derivative;
            ButterworthLowPassFilter<float> derivativeLowPass;
            JitterFilter jitterFilter;
            ButterworthLowPassFilter<float> airNoiseLowPass;
            LowPassFilter inputDcFilter;
            ButterworthLowPassFilter<double> antialiasing;
        };

    public:
        Synthesizer();
        ~Synthesizer();

        void initialize(const Parameters &p);
        void initializeImpulseResponse(
            const int16_t *impulseResponse,
            unsigned int samples,
            float volume,
            int index);
        void startAudioRenderingThread();
        void endAudioRenderingThread();
        void destroy();

        int readAudioOutput(int samples, int16_t *buffer);

        void writeInput(const double *data);
        void endInputBlock();

        void waitProcessed();

        void audioRenderingThread();
        void renderAudio();

        double getLatency() const;

        int inputDelta(int s1, int s0) const;
        double inputDistance(double s1, double s0) const;

        void setInputSampleRate(double sampleRate);
        double getInputSampleRate() const { return m_inputSampleRate; }

        // 0..1 multiplier on the derivative/dF_F_mix term. Set from the
        // simulation thread every step, read by the audio thread.
        void setBiteScale(float scale) {
            m_biteScale.store(scale, std::memory_order_relaxed);
        }

        // Ask the audio thread to flush the reverb tail at the top of its next
        // block. Safe to call from any thread; see renderAudio().
        void resetReverb() {
            m_reverbResetRequested.store(true, std::memory_order_relaxed);
        }

        // Recomputes the input-stage low-pass coefficients if the effective cutoff
        // has moved (either the parameter or the input sample rate changed).
        // Called from writeInput() on the simulation thread; lock-free by design.
        void updateInputFilterCutoff();
        float effectiveInputFilterCutoff() const;

        int16_t renderAudio(int inputOffset);

        double getLevelerGain();
        AudioParameters getAudioParameters();
        void setAudioParameters(const AudioParameters &params);

    //protected:
        ButterworthLowPassFilter<float> m_antialiasing;
        ReverbFilter m_reverb;
        ButterworthLowPassFilter<float> m_distanceFilter;
        float m_appliedDistanceCutoff = -1.0f;
        bool m_distanceFilterActive = false;
        float m_appliedRoomSize = -1.0f;
        float m_appliedDamping = -1.0f;
        float m_appliedEarly = -1.0f;
        LevelingFilter m_levelingFilter;
        InputChannel *m_inputChannels;
        AudioParameters m_audioParameters;
        int m_inputChannelCount;
        int m_inputBufferSize;
        int m_inputSamplesRead;
        int m_latency;
        double m_inputWriteOffset;
        double m_lastInputSampleOffset;

        RingBuffer<int16_t> m_audioBuffer;
        int m_audioBufferSize;

        float m_inputSampleRate;
        float m_audioSampleRate;

        // Written by setAudioParameters() (any thread), read by the sim thread.
        std::atomic<float> m_targetInputFilterCutoff;
        // Sim thread only -- the cutoff currently baked into the filter coefficients.
        float m_appliedInputFilterCutoff;

        // Written and read only by the audio rendering thread.
        bool m_biteFilterActive = false;

        std::atomic<float> m_biteScale{ 1.0f };
        std::atomic<bool> m_reverbResetRequested{ false };

        std::thread *m_thread;
        std::atomic<bool> m_run;
        bool m_processed;

        std::mutex m_inputLock;
        std::mutex m_lock0;
        std::condition_variable m_cv0;

        ProcessingFilters *m_filters;
};

#endif /* ATG_ENGINE_SIM_ENGINE_SYNTHESIZER_H */
