#define NOMINMAX
#include "MovieRecorder.h"
#include <cstdio>
#include <filesystem>
#include <stdexcept>
#include <vector>
#include <windows.h>

void require(bool value, const char *message)
{
    if (!value)
        throw std::runtime_error(message);
}
int main(int argc, char **argv)
{
    wchar_t executable[32768];
    if (!SearchPathW(nullptr, L"ffmpeg.exe", nullptr, 32768, executable, nullptr))
        return 77;
    try
    {
        require(argc == 2, "Pass an output directory");
        const auto directory = std::filesystem::u8path(argv[1]);
        std::filesystem::create_directories(directory);
        MovieRecorder recorder;
        bool rejected = false;
        try
        {
            recorder.start((directory / "invalid.mp4").u8string(), 0, 15, 30, false);
        }
        catch (const std::exception &)
        {
            rejected = true;
        }
        require(rejected && !recorder.active(), "Invalid dimensions were accepted");
        std::vector<uint8_t> pixels(17 * 15 * 4, 255);
        for (int y = 0; y < 15; ++y)
            for (int x = 0; x < 17; ++x)
            {
                const int at = (y * 17 + x) * 4;
                pixels[at] = y < 7 ? 255 : 0;
                pixels[at + 1] = 0;
                pixels[at + 2] = y < 7 ? 0 : 255;
            }
        for (bool flip : {false, true})
        {
            const auto output = directory / (flip ? u8"bottom-up.mp4" : u8"movie space & \u00fc.mp4");
            recorder.start(output.u8string(), 17, 15, flip ? 60 : 30, flip);
            rejected = false;
            try
            {
                recorder.write(pixels.data(), 4);
            }
            catch (const std::exception &)
            {
                rejected = true;
            }
            require(rejected, "Mismatched frame size was accepted");
            for (int frame = 0; frame < 30; ++frame)
                recorder.write(pixels.data(), pixels.size());
            require(recorder.frames() == 30, "Recording frame count differs");
            recorder.stop();
            recorder.stop();
            require(!recorder.active() && std::filesystem::file_size(output) > 100, "Movie was not finalized");
        }
        // Startup failures must be surfaced even if CreateProcess itself succeeds.
        rejected = false;
        try
        {
            recorder.start((directory / "missing-directory" / "movie.mp4").u8string(), 17, 15, 30, false);
            for (int frame = 0; frame < 30; ++frame)
                recorder.write(pixels.data(), pixels.size());
            recorder.stop();
        }
        catch (const std::exception &)
        {
            rejected = true;
        }
        require(rejected && !recorder.active(), "Encoder failure was not reported and cleaned up");
        // The same recorder must remain usable after an encoder failure.
        recorder.start((directory / "restart.mp4").u8string(), 17, 15, 30, false);
        recorder.write(pixels.data(), pixels.size());
        recorder.stop();
        const DWORD length = GetEnvironmentVariableW(L"PATH", nullptr, 0);
        std::vector<wchar_t> savedPath(length);
        GetEnvironmentVariableW(L"PATH", savedPath.data(), length);
        SetEnvironmentVariableW(L"PATH", L"");
        rejected = false;
        try
        {
            recorder.start((directory / "missing-encoder.mp4").u8string(), 17, 15, 30, false);
        }
        catch (const std::exception &)
        {
            rejected = true;
        }
        SetEnvironmentVariableW(L"PATH", savedPath.data());
        require(rejected && !recorder.active(), "Missing FFmpeg was not reported");
        std::puts("Movie recorder tests passed");
        return 0;
    }
    catch (const std::exception &e)
    {
        std::fprintf(stderr, "%s\n", e.what());
        return 1;
    }
}
