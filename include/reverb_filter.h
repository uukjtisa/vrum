#ifndef ATG_ENGINE_SIM_REVERB_FILTER_H
#define ATG_ENGINE_SIM_REVERB_FILTER_H

// VRUM: the "outdoor" listening perspective.
//
// The synthesizer's output is a close-mic signal: it is what the exhaust
// radiates, with no space around it. Real footage of a car never sounds like
// that -- what you hear is the exhaust plus the reflections off whatever it is
// driving past, and the higher frequencies stripped out by distance. That is
// most of why a recording sounds like a car "out there" rather than an engine
// on a bench, and none of it is something the engine model itself can produce.
//
// Schroeder/Freeverb topology: parallel damped comb filters build a dense tail,
// series allpass filters smear it so it stops sounding like discrete echoes.
// Damping inside the comb feedback loop is what makes it read as outdoors
// rather than as a tiled room -- each reflection loses top end, the way air and
// soft ground actually absorb.
//
// THE THREE THINGS THAT MAKE OR BREAK THIS (all three were wrong in the first
// version, which came out as a flutter echo that sounded like the audio was
// looping):
//
//   1. Echo DENSITY. A reverb is only a reverb when the repeats are too close
//      together to count. The first version used 4 combs and, worse, the four
//      LONGEST of Freeverb's set (1422-1617 samples = 32-37 ms). Four taps
//      spaced 32-37 ms apart is the textbook definition of a flutter echo, not
//      a tail. All 8 combs, starting at 1116, is the minimum that reads as a
//      space.
//   2. INPUT GAIN, and it must TRACK THE FEEDBACK. A comb loop has a
//      steady-state gain of 1/(1-f) -- 25x at f=0.96, but only 3.6x at f=0.72.
//      With no input scaling at all the wet path came back two orders of
//      magnitude hot and sounded like the audio was looping. With Freeverb's
//      FIXED 0.015 it then went the other way at short room sizes and landed
//      ~18 dB under the dry, i.e. inaudible. Derive it from f (see
//      m_inputGain) and the wet return stays near unity at any room size.
//   3. FEEDBACK matched to the surface. Open ground is short and heavily
//      damped; a tunnel is long and barely damped, because concrete reflects
//      high frequencies that grass and air absorb. One "reverb amount" knob
//      cannot express both -- hence the discrete modes.
//
// Also: a reverb announces itself on TRANSIENTS AND DECAYS, and a running
// engine is a sustained periodic tone with neither. Expect it to be most
// obvious on backfires, throttle lifts and the overrun, and least obvious at
// steady cruise. That is physics, not a bug.
class ReverbFilter {
    public:
        ReverbFilter();
        ~ReverbFilter();

        void initialize(float sampleRate);
        void destroy();
        void reset();

        // roomSize 0..1 -> reflection decay; damping 0..1 -> how fast the tail
        // loses its top end.
        void setRoomSize(float roomSize);
        void setDamping(float damping);

        // 0..1 level of the discrete early reflections mixed into the wet
        // signal. This is the parameter that separates "a space" from "a
        // tunnel": a tunnel's walls are close and hard, so you hear individual
        // slap-backs on top of the tail, not just a diffuse wash.
        void setEarlyLevel(float level);

        // Returns the wet (reverberated) signal only; the caller mixes it.
        float f(float sample);

    protected:
        static constexpr int CombCount = 8;
        static constexpr int AllpassCount = 4;

        static constexpr int EarlyTapCount = 4;

        struct Comb {
            float *buffer = nullptr;
            int length = 0;
            int offset = 0;
            float store = 0.0f;   // one-pole state, the "damping" memory
        };

        struct Allpass {
            float *buffer = nullptr;
            int length = 0;
            int offset = 0;
        };

        Comb m_combs[CombCount];
        Allpass m_allpasses[AllpassCount];

        // Discrete slap-backs off the nearest hard surfaces, before the diffuse
        // tail arrives.
        float *m_earlyBuffer = nullptr;
        int m_earlyLength = 0;
        int m_earlyOffset = 0;
        int m_earlyTaps[EarlyTapCount] = { 0, 0, 0, 0 };
        float m_earlyLevel = 0.0f;

        float m_feedback = 0.80f;
        float m_damp1 = 0.4f;
        float m_damp2 = 0.6f;

        // Input scaling, recomputed whenever the feedback changes.
        //
        // A comb loop has a steady-state gain of 1/(1-f), so a FIXED input gain
        // (Freeverb uses 0.015) makes the wet level swing wildly with room
        // size -- at the short, damped settings the first "outdoor" tuning
        // used, the wet path came back roughly 18 dB under the dry and was
        // simply inaudible. Deriving it from the feedback instead keeps the wet
        // return at about unity whatever the room is, which is what makes
        // reverbMix behave like an actual wet/dry control.
        float m_inputGain = 0.015f;
};

#endif /* ATG_ENGINE_SIM_REVERB_FILTER_H */
