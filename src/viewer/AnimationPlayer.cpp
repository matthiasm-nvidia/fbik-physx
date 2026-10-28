#include "AnimationPlayer.h"
#include "SkeletonPose.h"
#include <set>
#include <algorithm>
namespace viewer {
namespace {
void applyTime(ovstage_instance_t *s, ovstage_ordinal_t &ordinal, double seconds)
{
    if (!std::isfinite(seconds)) throw std::runtime_error("Invalid animation time.");
    auto op=ovstage_population_apply_usd_time(s,++ordinal,seconds);
    stage::check(op.status);
    ovstage_population_op_wait_result_t result{};
    const auto status=ovstage_population_wait_op(s,op.op_index,OVSTAGE_TIMEOUT_INFINITE,&result);
    if (result.error_op_id_count) {
        const auto error=ovstage_population_get_last_op_error(result.error_op_ids[0]);
        throw std::runtime_error(std::string(error.ptr,error.length));
    }
    stage::check(status);
    stage::seal(s,ordinal);
    stage::hierarchy(s,ordinal);
}
}

std::vector<fbik::Pose> sampleAnimation(ovstage_instance_t *s, ovstage_ordinal_t &ordinal,
                                      double seconds, const std::vector<fbik::Character> &characters)
{
    applyTime(s, ordinal, seconds);
    const auto data=stage::read(s,ordinal,{"omni:fabric:worldMatrix","joints","restTransforms","bindTransforms",
        "skel:animationSource","translations","rotations","scales"});
    std::vector<fbik::Pose> poses;
    for (const auto &character : characters) {
        const auto sk=skeleton::readSkeleton(data,character.skeletonPath,character.characterPath);
        if (sk.names!=character.joints) throw std::runtime_error("Animated skeleton joint order changed.");
        fbik::Pose pose; pose.skeletonPath=sk.path;
        pose.characterWorld=skeleton::packed(skeleton::world(data.at(sk.character)));
        for (const auto &bone : sk.jointPose) pose.bones.push_back(skeleton::packed(sk.toCharacter*bone));
        poses.push_back(std::move(pose));
    }
    return poses;
}
struct AnimationPlayer::Impl {
    ovstage_instance_t *stage = nullptr;
    stage::Query samples;
    struct Rig {
        skeleton::Skeleton skeleton;
        std::string source;
        std::vector<int> mapping;
    };
    std::vector<Rig> rigs;
};
AnimationPlayer::AnimationPlayer() : impl(std::make_unique<Impl>()) {}
AnimationPlayer::~AnimationPlayer() = default;
void AnimationPlayer::init(ovstage_instance_t *s, ovstage_ordinal_t ordinal,
                           const std::vector<fbik::Character> &characters)
{
    if (impl->stage) throw std::runtime_error("Animation player is already initialized.");
    impl->stage = s;
    std::set<std::string> paths;
    if (!characters.empty()) {
        const auto data = stage::read(s, ordinal, {"omni:fabric:worldMatrix", "joints", "restTransforms", "bindTransforms",
                                                  "skel:animationSource"});
        for (const auto &character : characters) {
            Impl::Rig rig;
            rig.skeleton = skeleton::readSkeleton(data, character.skeletonPath, character.characterPath);
            skeleton::require(rig.skeleton.names == character.joints, "Animated skeleton joint order changed.");
            const auto &source = skeleton::inherited(data, character.skeletonPath, "skel:animationSource").paths;
            if (!source.empty()) {
                rig.source = source.front();
                for (const auto &name : skeleton::get(data.at(rig.source), "joints").tokens) {
                    const auto it = std::find(character.joints.begin(), character.joints.end(), name);
                    rig.mapping.push_back(it == character.joints.end() ? -1 : int(it - character.joints.begin()));
                }
                paths.insert(rig.source);
            }
            paths.insert(character.skeletonPath); paths.insert(character.characterPath);
            impl->rigs.push_back(std::move(rig));
        }
    }
    impl->samples.init(s, {paths.begin(), paths.end()}, {"translations", "rotations", "scales", "omni:fabric:worldMatrix"});
}
std::vector<fbik::Pose> AnimationPlayer::sample(ovstage_ordinal_t &ordinal, double seconds)
{
    if (!impl->stage) throw std::runtime_error("Animation player is not initialized.");
    applyTime(impl->stage, ordinal, seconds);
    const auto data = impl->samples.read(ordinal);
    std::vector<fbik::Pose> poses;
    for (const auto &rig : impl->rigs) {
        const auto &sk = rig.skeleton;
        auto local = sk.restLocal;
        if (const auto sample = data.find(rig.source); sample != data.end())
            skeleton::applyAnimation(local, sample->second, rig.mapping);
        const auto characterWorld = skeleton::world(data.at(sk.character));
        const auto toCharacter = skeleton::inverse(characterWorld) * skeleton::world(data.at(sk.path));
        std::vector<skeleton::Matrix> joints;
        fbik::Pose pose; pose.skeletonPath = sk.path; pose.characterWorld = skeleton::packed(characterWorld);
        for (size_t j = 0; j < local.size(); ++j) {
            joints.push_back(sk.parents[j] < 0 ? local[j] : joints[size_t(sk.parents[j])] * local[j]);
            pose.bones.push_back(skeleton::packed(toCharacter * joints.back()));
        }
        poses.push_back(std::move(pose));
    }
    return poses;
}

}
