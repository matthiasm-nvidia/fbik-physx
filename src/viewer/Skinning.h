#pragma once
#include "Fbik.h"
namespace viewer {
// Viewer-owned CPU linear skinning for the native renderer. Never modifies source assets.
class Skinning {
public:
    Skinning(); ~Skinning();
    void init(ovstage_instance_t *, ovstage_ordinal_t, const std::vector<fbik::Character> &);
    void update(const std::vector<fbik::Pose> &, ovstage_ordinal_t &);
private:
    struct Impl; std::unique_ptr<Impl> impl;
};
}
