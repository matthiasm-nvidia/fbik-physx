#pragma once
#include "Camera.h"
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace viewer
{
class RtxView
{
  public:
    RtxView();
    ~RtxView();
    RtxView(const RtxView &) = delete;
    RtxView &operator=(const RtxView &) = delete;

    void initialize(const std::string &executableDirectory);
    // A failed load preserves the previously loaded scene whenever the SDK permits recovery.
    void load(const std::string &path, int width, int height);
    void render(const Camera &camera, int width, int height);
    void startSimulation();
    void stopSimulation();
    void stepSimulation();
    bool simulating() const;
    bool loaded() const;
    const std::vector<uint8_t> &pixels() const;
    int width() const;
    int height() const;
    Vec3 center() const;
    float radius() const;
    bool zUp() const;
    const Camera *initialCamera() const;
    const std::string &path() const;

  private:
    struct Impl;
    std::unique_ptr<Impl> impl;
};
} // namespace viewer
