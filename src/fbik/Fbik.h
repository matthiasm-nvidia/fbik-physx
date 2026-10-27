#pragma once
#include <ovstage/ovstage.h>
#include <cstddef>
#include <memory>
#include <array>
#include <string>
#include <vector>

namespace fbik
{
struct InitResult
{
    size_t characters = 0, skeletons = 0, convexMeshes = 0, capsules = 0, leaves = 0, degenerate = 0;
};

using Matrix4 = std::array<double,16>; // USD row-vector convention; translation in 12..14.
struct Character { std::string skeletonPath, characterPath; std::vector<std::string> joints; };
struct Pose {
    std::string skeletonPath;
    Matrix4 characterWorld;
    std::vector<Matrix4> bones; // Character-space transforms, in Character::joints order.
};
struct BodyTarget { std::string path; Matrix4 local, world; };

// Owns only generated, in-memory USD content. The caller owns the stage and
// ordinal; initialize before attaching it to physics. Shapes are fitted once; updateTargets maps evaluated poses to kinematic bodies.
class Module
{
  public:
    Module();
    ~Module();
    Module(const Module &) = delete;
    Module &operator=(const Module &) = delete;
    // Only weights strictly above minSkinWeight contribute vertices to a bone hull.
    InitResult init(ovstage_instance_t *scene, ovstage_ordinal_t &ordinal, float minSkinWeight = 0.3f);
    const std::vector<Character> &characters() const;
    std::vector<BodyTarget> updateTargets(const std::vector<Pose> &poses) const;
    void showColliders(bool visible, ovstage_ordinal_t &ordinal);

  private:
    struct Impl;
    std::unique_ptr<Impl> impl;
};
} // namespace fbik
