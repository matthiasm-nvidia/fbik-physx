#pragma once
#include "Camera.h"
#include "StageData.h"
#include <optional>

namespace viewer
{
inline constexpr const char *cameraAttributeNames[] = {"projection",
                                                       "focalLength",
                                                       "horizontalAperture",
                                                       "verticalAperture",
                                                       "horizontalApertureOffset",
                                                       "verticalApertureOffset",
                                                       "clippingRange"};

struct StageCamera
{
    std::string path;
    Camera camera;
};
// Called before the viewer adds its own camera. First usable camera by USD path wins.
std::optional<StageCamera> readStageCamera(ovstage_instance_t *stage, ovstage_ordinal_t ordinal);
std::optional<StageCamera> readStageCamera(const stage::Snapshot &snapshot);
} // namespace viewer
