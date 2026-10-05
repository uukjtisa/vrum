#ifndef ATG_ENGINE_SIM_WAV_WRITER_H
#define ATG_ENGINE_SIM_WAV_WRITER_H

#include <cinttypes>
#include <fstream>
#include <string>

// Minimal 16-bit PCM RIFF/WAVE writer, used to capture exactly the samples that
// reach the speakers so they can be measured offline (scripts/analyze_sound.py).
//
// The header carries byte counts that aren't known until the recording stops, so
// it is written with placeholders up front and patched during close().
class WavWriter {
    public:
        WavWriter();
        ~WavWriter();

        // Creates any missing parent directories. Returns false if the file
        // could not be opened, in which case the writer stays closed.
        bool open(const std::string &path, int sampleRate = 44100, int channelCount = 1);

        void write(const int16_t *samples, int count);

        // Patches the RIFF/data sizes and closes. Safe to call when not open.
        void close();

        bool isOpen() const { return m_file.is_open(); }
        const std::string &getPath() const { return m_path; }
        unsigned int getSampleCount() const { return m_sampleCount; }
        double getDuration() const {
            return (m_sampleRate > 0)
                ? (double)m_sampleCount / m_sampleRate
                : 0.0;
        }

    protected:
        void writeHeader();
        // Rewrites the RIFF/data sizes in place, then seeks back to the end.
        void patchSizes();

        std::ofstream m_file;
        std::string m_path;
        unsigned int m_sampleCount;
        int m_sampleRate;
        int m_channelCount;
};

#endif /* ATG_ENGINE_SIM_WAV_WRITER_H */
