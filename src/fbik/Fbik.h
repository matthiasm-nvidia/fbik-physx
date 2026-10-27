#pragma once
#include <ovstage/ovstage.h>
#include <cstddef>
#include <memory>

namespace fbik
{
struct InitResult
{
    size_t characters = 0, skeletons = 0, boxes = 0, capsules = 0, leaves = 0, degenerate = 0;
};

// Owns only generated, in-memory USD content. The caller owns the stage and
// ordinal; initialize before attaching it to physics. Initial pose only for now.
class Module
{
  public:
    Module();
    ~Module();
    Module(const Module &) = delete;
    Module &operator=(const Module &) = delete;
    InitResult init(ovstage_instance_t *scene, ovstage_ordinal_t &ordinal);
    void showColliders(bool visible, ovstage_ordinal_t &ordinal);

  private:
    struct Impl;
    std::unique_ptr<Impl> impl;
};
} // namespace fbik
