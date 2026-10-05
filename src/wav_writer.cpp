#include "../include/wav_writer.h"

#include <filesystem>

namespace {
    void writeU32(std::ofstream &f, uint32_t v) {
        f.write(reinterpret_cast<const char *>(&v), 4);
    }

    void writeU16(std::ofstream &f, uint16_t v) {
        f.write(reinterpret_cast<const char *>(&v), 2);
    }

    constexpr int BitsPerSample = 16;

    // Byte offsets of the two size fields that can only be filled in at close().
    constexpr std::streamoff RiffSizeOffset = 4;
    constexpr std::streamoff DataSizeOffset = 40;
}

WavWriter::WavWriter() {
    m_sampleCount = 0;
    m_sampleRate = 0;
    m_channelCount = 0;
}

WavWriter::~WavWriter() {
    close();
}

bool WavWriter::open(const std::string &path, int sampleRate, int channelCount) {
    close();

    std::error_code ec;
    const std::filesystem::path fsPath(path);
    if (fsPath.has_parent_path()) {
        // Non-throwing overload: a pre-existing directory is not an error.
        std::filesystem::create_directories(fsPath.parent_path(), ec);
    }

    m_file.open(path, std::ios::binary | std::ios::trunc);
    if (!m_file.is_open()) {
        return false;
    }

    m_path = path;
    m_sampleCount = 0;
    m_sampleRate = sampleRate;
    m_channelCount = channelCount;

    writeHeader();

    return true;
}

void WavWriter::writeHeader() {
    const uint16_t blockAlign =
        static_cast<uint16_t>(m_channelCount * BitsPerSample / 8);

    m_file.write("RIFF", 4);
    writeU32(m_file, 0);                // patched in close()
    m_file.write("WAVE", 4);

    m_file.write("fmt ", 4);
    writeU32(m_file, 16);               // PCM fmt chunk size
    writeU16(m_file, 1);                // format = PCM
    writeU16(m_file, static_cast<uint16_t>(m_channelCount));
    writeU32(m_file, static_cast<uint32_t>(m_sampleRate));
    writeU32(m_file, static_cast<uint32_t>(m_sampleRate) * blockAlign);
    writeU16(m_file, blockAlign);
    writeU16(m_file, BitsPerSample);

    m_file.write("data", 4);
    writeU32(m_file, 0);                // patched in close()
}

void WavWriter::write(const int16_t *samples, int count) {
    if (!m_file.is_open() || count <= 0) return;

    m_file.write(
        reinterpret_cast<const char *>(samples),
        static_cast<std::streamsize>(count) * sizeof(int16_t));

    m_sampleCount += count;

    // Patch the sizes after every block rather than only in close(), so the file
    // on disk is a valid WAV at all times. Closing the app with the X button does
    // not unwind through close(), and a capture left running that way would
    // otherwise be a multi-megabyte file that every tool reads as 0 samples.
    // ~40 blocks/sec, so the extra seeks are irrelevant beside the audio work.
    patchSizes();
}

void WavWriter::patchSizes() {
    const uint32_t dataBytes =
        static_cast<uint32_t>(m_sampleCount) * (BitsPerSample / 8);

    const std::streampos end = m_file.tellp();

    m_file.seekp(DataSizeOffset, std::ios::beg);
    writeU32(m_file, dataBytes);

    m_file.seekp(RiffSizeOffset, std::ios::beg);
    writeU32(m_file, 36 + dataBytes);

    m_file.seekp(end);
}

void WavWriter::close() {
    if (!m_file.is_open()) return;

    patchSizes();
    m_file.close();
}
