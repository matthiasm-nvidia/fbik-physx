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
    size_t characters = 0, skeletons = 0, capsules = 0, leaves = 0, degenerate = 0;
    size_t joints = 0;
    size_t collapsedBones = 0; // Descendants absorbed into hand/head bodies (including leaves).
    size_t hinges = 0; // Remaining connections are D6 ball joints with swing/twist limits.
    size_t connectedExclusions = 0, overlapExclusions = 0;
};

enum class Mode { Follow, Ragdoll, QuasiStatic };
enum class ExclusionReason { Connected, RestOverlap };
struct CollisionExclusion {
    std::string body0, body1, bone0, bone1;
    ExclusionReason reason;
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
// ordinal; initialize before attaching it to physics. Shapes and joints are built
// once. updateTargets maps animation to bodies; readPoses maps physics back to bones.
class Module
{
  public:
    Module();
    ~Module();
    Module(const Module &) = delete;
    Module &operator=(const Module &) = delete;
    // Starts in Follow mode. Only weights strictly above minSkinWeight contribute to a shape.
    // Hands/wrists and heads absorb descendant skin and omit descendant physics joints.
    // Joint frames/limits use the rest pose; the physics wrapper applies fbik:swingLimit cones.
    // Connected bodies and capsules overlapping in the rest pose are filtered once at init.
    // jointDamping is an angular velocity decay rate in 1/s; zero disables damping.
    InitResult init(ovstage_instance_t *scene, ovstage_ordinal_t &ordinal, float minSkinWeight = 0.3f,
                    float jointDamping = 5.0f);
    const std::vector<Character> &characters() const;
    const std::vector<CollisionExclusion> &collisionExclusions() const;
    std::vector<BodyTarget> updateTargets(const std::vector<Pose> &poses) const;
    // Authors a sealed control ordinal; the caller must submit it to physics before stepping.
    // A repeated mode is a no-op. Transitions clear velocities and release bodies awake.
    // QuasiStatic disables gravity; the physics wrapper clears velocities each tick
    // on bodies tagged fbik:quasiStatic while leaving contacts and joints active.
    void setMode(Mode mode, ovstage_ordinal_t &ordinal);
    Mode mode() const;
    // Read after physics publishes its poses. Reference poses preserve character placement
    // and local transforms of bones without bodies; capture them on leaving Follow.
    std::vector<Pose> readPoses(const std::vector<Pose> &referencePoses, ovstage_ordinal_t ordinal) const;
    void showColliders(bool visible, ovstage_ordinal_t &ordinal);

  private:
    struct Impl;
    std::unique_ptr<Impl> impl;
};
} // namespace fbik
