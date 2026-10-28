#pragma once
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

// Streams RGBA8 frames to an independent FFmpeg process; no FFmpeg SDK dependency.
class MovieRecorder
{
  public:
    MovieRecorder();
    ~MovieRecorder();
    void start(const std::string &path, int width, int height, int fps, bool bottomUp);
    void write(const uint8_t *rgba, size_t bytes);
    void stop(); // Closes stdin, waits for the MP4 trailer, and reports encoder failures.
    bool active() const;
    int width() const;
    int height() const;
    int fps() const;
    size_t frames() const;
    const std::string &path() const;

  private:
    struct Impl;
    std::unique_ptr<Impl> impl;
};
