#pragma once

#include "CameraMath.h"

namespace viewer
{

class Camera
{
  public:
    struct Ray { Vec3 origin, direction; };
    Ray screenRay(float x, float y, float width, float height) const;
    void init();
    void reset_view();
    void look_at(const Vec3 &position, const Vec3 &target);

    void handle_mouse_view(int dx, int dy);
    void handle_mouse_orbit(int dx, int dy, const Vec3 &center);
    void handle_mouse_translate(int dx, int dy, float scale = 0.01f);
    void handle_key_down(int key);
    void handle_wheel(float rotation);

    Vec3 pos;
    Vec3 forward;
    Vec3 right;
    Vec3 up;
    Vec3 orbitUp{0.0f, 1.0f, 0.0f};
    float speed = 0.1f;
    float fov = 40.0f;
    bool orthographic = false;
    float orthographicHeight = 1;
    // Lens offsets as fractions of the vertical aperture. Preserve vertical framing on resize.
    float horizontalOffset = 0, verticalOffset = 0;
    float nearClip = 0.001f, farClip = 1e9f;

};


} // namespace viewer
