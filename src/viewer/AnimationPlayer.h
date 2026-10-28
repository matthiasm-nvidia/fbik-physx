#pragma once
#include "Fbik.h"
namespace viewer {
// Skeleton topology and bindings are fixed for this scene; only sampled values are read per tick.
class AnimationPlayer {
  public:
    AnimationPlayer();
    ~AnimationPlayer();
    void init(ovstage_instance_t *stage, ovstage_ordinal_t ordinal, const std::vector<fbik::Character> &characters);
    std::vector<fbik::Pose> sample(ovstage_ordinal_t &ordinal, double seconds);
  private:
    struct Impl;
    std::unique_ptr<Impl> impl;
};
// Samples USD and evaluates FK; physics output is never used as animation input.
std::vector<fbik::Pose> sampleAnimation(ovstage_instance_t *stage, ovstage_ordinal_t &ordinal,
                                      double seconds, const std::vector<fbik::Character> &characters);
}
