#pragma once
#include "Camera.h"
#include "Fbik.h"
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
    void startAnimation();
    void stopAnimation();
    bool animating() const;
    double animationStart() const;
    double animationEnd() const;
    double animationTime() const;
    void seekAnimation(double seconds);
    void setAnimationLoop(bool loop);
    void setCharacterMode(fbik::Mode mode);
    fbik::Mode characterMode() const;
    size_t characterJointCount() const;
    Vec3 characterCenter() const;
    float characterRadius() const;
    void startSimulation();
    void stopSimulation();
    void stepSimulation();
    bool simulating() const;
    // A successful pick also prepares physics when stopped, without starting animation.
    bool beginDrag(const Camera::Ray &ray);
    void moveDrag(const Camera::Ray &ray);
    void endDrag();
    bool dragging() const;
    size_t boneColliderCount() const;
    const std::vector<fbik::CollisionExclusion> &collisionExclusions() const;
    size_t boneJointCount() const;
    bool boneCollidersVisible() const;
    void showBoneColliders(bool visible);
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
