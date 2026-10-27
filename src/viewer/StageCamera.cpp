#include "StageCamera.h"
#include <cmath>
#include <cstdio>
#include <limits>
#include <stdexcept>

namespace viewer
{
namespace
{
float scalar(const stage::Prim &prim, const char *name, float fallback)
{
    const auto it = prim.find(name);
    if (it == prim.end())
        return fallback;
    const auto &v = it->second;
    if (!v.token.empty() || v.numbers.size() != 1 || !std::isfinite(v.numbers[0]) ||
        std::abs(v.numbers[0]) > std::numeric_limits<float>::max())
        throw std::runtime_error(std::string("Invalid camera ") + name);
    return float(v.numbers[0]);
}
Camera convert(const stage::Prim &prim)
{
    Camera camera;
    camera.init();
    const auto matrix = prim.find("omni:fabric:worldMatrix");
    if (matrix == prim.end() || matrix->second.numbers.size() != 16)
        throw std::runtime_error("Missing camera world transform");
    const auto &m = matrix->second.numbers;
    for (double v : m)
        if (!std::isfinite(v) || std::abs(v) > std::numeric_limits<float>::max())
            throw std::runtime_error("Invalid camera world transform");
    camera.pos = Vec3(float(m[12]), float(m[13]), float(m[14]));
    // USD cameras look along local -Z, with local +Y up, regardless of stage up axis.
    camera.forward = Vec3(-float(m[8]), -float(m[9]), -float(m[10]));
    camera.up = Vec3(float(m[4]), float(m[5]), float(m[6]));
    if (!(camera.forward.normalize() > 0) || !(camera.up.normalize() > 0))
        throw std::runtime_error("Degenerate camera transform");
    camera.right = camera.forward.cross(camera.up);
    if (!(camera.right.normalize() > 1e-6f))
        throw std::runtime_error("Degenerate camera transform");
    camera.up = camera.right.cross(camera.forward);
    camera.up.normalize();
    const auto projection = prim.find("projection");
    if (projection != prim.end())
    {
        const auto &value = projection->second;
        if (!value.numbers.empty() || (value.token != "perspective" && value.token != "orthographic"))
            throw std::runtime_error("Unsupported camera projection");
        camera.orthographic = value.token == "orthographic";
    }
    const float focal = scalar(prim, "focalLength", 50);
    const float aperture = scalar(prim, "verticalAperture", 15.2908f);
    if (!(focal > 0) || !(aperture > 0) || !(scalar(prim, "horizontalAperture", 20.955f) > 0))
        throw std::runtime_error("Camera aperture and focal length must be positive");
    camera.fov = float(2 * std::atan(double(aperture) / (2 * focal)) * 180 / 3.141592653589793);
    camera.orthographicHeight = aperture * 0.1f; // USD aperture units are tenths of a stage unit.
    camera.horizontalOffset = scalar(prim, "horizontalApertureOffset", 0) / aperture;
    camera.verticalOffset = scalar(prim, "verticalApertureOffset", 0) / aperture;
    const auto clip = prim.find("clippingRange");
    camera.nearClip = 1;
    camera.farClip = 1000000;
    if (clip != prim.end())
    {
        const auto &range = clip->second.numbers;
        if (range.size() != 2 || !std::isfinite(range[0]) || !std::isfinite(range[1]) || range[0] <= 0 ||
            range[1] <= range[0] || range[1] > std::numeric_limits<float>::max())
            throw std::runtime_error("Invalid camera clipping range");
        camera.nearClip = float(range[0]);
        camera.farClip = float(range[1]);
    }
    if (!(camera.fov > 0 && camera.fov < 180) || !(camera.orthographicHeight > 0) ||
        !std::isfinite(camera.horizontalOffset) || !std::isfinite(camera.verticalOffset))
        throw std::runtime_error("Invalid camera lens");
    return camera;
}
} // namespace
std::optional<StageCamera> readStageCamera(const stage::Snapshot &snapshot)
{
    for (const auto &[path, prim] : snapshot)
    {
        const auto type = prim.find("usd-prim-type");
        if (type == prim.end() || type->second.token != "Camera")
            continue;
        try
        {
            return StageCamera{path, convert(prim)};
        }
        catch (const std::runtime_error &e)
        {
            std::fprintf(stderr, "Skipping USD camera %s: %s\n", path.c_str(), e.what());
        }
    }
    return std::nullopt;
}
std::optional<StageCamera> readStageCamera(ovstage_instance_t *stage, ovstage_ordinal_t ordinal)
{
    std::vector<std::string> names{"usd-prim-type", "omni:fabric:worldMatrix"};
    for (const auto *name : cameraAttributeNames)
        names.emplace_back(name);
    return readStageCamera(stage::read(stage, ordinal, names));
}
} // namespace viewer
