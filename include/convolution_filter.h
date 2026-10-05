#ifndef ATG_ENGINE_SIM_CONVOLUTION_FILTER_H
#define ATG_ENGINE_SIM_CONVOLUTION_FILTER_H

#include "filter.h"

#include <complex>
#include <vector>

// VRUM: zero-latency partitioned convolution.
//
// Upstream convolved directly: up to 10,000 taps per sample per exhaust bank,
// ~880M multiply-adds a second for the V12 -- the single biggest CPU cost in the
// audio path, and the thing that made a phone port look impossible.
//
// This computes the SAME sum (identical to float rounding) in two parts:
//   - head: the first `BlockSize` taps, convolved directly per sample;
//   - tail: every later tap, as uniformly partitioned overlap-save in the
//     frequency domain (UPOLS), computed once per block.
// The tail's earliest tap is `BlockSize` samples old, so the block it needs is
// always complete before its output is due -- no added latency. That matters:
// the synthesizer mixes this output with the dry signal, and any delay between
// the two would comb-filter the engine voice.
class ConvolutionFilter : public Filter {
    public:
        static constexpr int BlockSize = 128;

    public:
        ConvolutionFilter();
        virtual ~ConvolutionFilter();

        void initialize(int samples);
        virtual float f(float sample) override;
        virtual void destroy();

        int getSampleCount() const { return m_sampleCount; }

        // The caller writes the impulse response through this pointer after
        // initialize(); the frequency-domain partitions are rebuilt lazily on
        // the next f() call, so callers need no extra step.
        float *getImpulseResponse() { m_dirty = true; return m_impulseResponse; }

        // Reference implementation (the upstream direct form). Kept for the
        // bench's equivalence check; never used on the audio path.
        float directF(float sample);

    protected:
        using cfloat = std::complex<float>;

        void prepare();
        void processBlock();

        static void fft(cfloat *data, int n, const cfloat *twiddles, bool inverse);

        float *m_impulseResponse;
        int m_sampleCount;
        bool m_dirty;

        // Direct-form state (head taps, and the reference path).
        float *m_shiftRegister;
        int m_shiftOffset;

        // Head: last BlockSize inputs, circular.
        std::vector<float> m_headHistory;
        int m_headPos;

        // Tail (UPOLS). FFT size is 2 * BlockSize.
        int m_partitions;
        std::vector<cfloat> m_twiddles;
        std::vector<cfloat> m_tailSpectra;   // m_partitions * (2B) filter spectra
        std::vector<cfloat> m_inputSpectra;  // m_partitions * (2B) delay line
        int m_inputSpectraHead;
        std::vector<float> m_prevBlock;
        std::vector<float> m_currBlock;
        std::vector<float> m_tailOut;        // tail output for the current block
        std::vector<cfloat> m_scratch;
        int m_blockPos;
};

#endif /* ATG_ENGINE_SIM_CONVOLUTION_FILTER_H */
