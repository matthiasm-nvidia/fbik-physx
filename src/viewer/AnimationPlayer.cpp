#include "AnimationPlayer.h"
#include "SkeletonPose.h"
namespace viewer {
std::vector<fbik::Pose> sampleAnimation(ovstage_instance_t *s, ovstage_ordinal_t &ordinal,
                                      double seconds, const std::vector<fbik::Character> &characters)
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
}
