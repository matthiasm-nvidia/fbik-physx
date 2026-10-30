#pragma once
#include "StageData.h"
#include "Fbik.h"
#include "CameraMath.h"
#include <memory>

namespace viewer
{
// Process runtime must outlive all simulations and be initialized before USD population.
class PhysicsRuntime
{
  public:
    PhysicsRuntime();
    ~PhysicsRuntime();
    PhysicsRuntime(const PhysicsRuntime &) = delete;
    PhysicsRuntime &operator=(const PhysicsRuntime &) = delete;
};

class PhysicsSimulation
{
  public:
    PhysicsSimulation(ovstage_instance_t *stage, ovstage_ordinal_t ordinal);
    ~PhysicsSimulation();
    PhysicsSimulation(const PhysicsSimulation &) = delete;
    PhysicsSimulation &operator=(const PhysicsSimulation &) = delete;
    void setKinematicTargets(const std::vector<fbik::BodyTarget> &targets, ovstage_ordinal_t &ordinal);
    // Updates spring goals only; never writes simulated body transforms.
    void setFollowTargets(const std::vector<fbik::BodyTarget> &targets, bool discontinuity = false);
    void setFollowSettings(const fbik::FollowSettings &settings);
    // Consume only a dedicated, sealed input ordinal (e.g. Module::setMode).
    void applyControlChanges(ovstage_ordinal_t ordinal);
    // A viewer may defer output across catch-up ticks, then publish once before rendering/control changes.
    void step(ovstage_ordinal_t &ordinal, bool publishOutput = true);
    void publish(ovstage_ordinal_t &ordinal);
    void stop(ovstage_ordinal_t &ordinal);
    bool beginDrag(Vec3 origin, Vec3 direction, ovstage_ordinal_t &ordinal);
    void moveDrag(Vec3 origin, Vec3 direction, ovstage_ordinal_t &ordinal);
    void endDrag(ovstage_ordinal_t &ordinal);
    bool dragging() const;
    static constexpr float timeStep = 1.0f / 60.0f;

  private:
    struct Impl;
    std::unique_ptr<Impl> impl;
};
} // namespace viewer
