#pragma once
#include "Fbik.h"
namespace viewer {
// Samples USD and evaluates FK; physics output is never used as animation input.
std::vector<fbik::Pose> sampleAnimation(ovstage_instance_t *stage, ovstage_ordinal_t &ordinal,
                                      double seconds, const std::vector<fbik::Character> &characters);
}
