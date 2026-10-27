#include "Camera.h"

#include <GLFW/glfw3.h>

#include <algorithm>
#include <cmath>

namespace viewer
{
namespace
{
constexpr float k_degrees_per_pixel = 0.5f;
constexpr float k_pi = 3.14159265358979323846f;

} // namespace

void Camera::init()
{
    up = Vec3(0.0f, 1.0f, 0.0f);
    speed = 0.1f;
    fov = 40.0f;
    orthographic = false;
    orthographicHeight = 1;
    horizontalOffset = verticalOffset = 0;
    nearClip = 0.001f;
    farClip = 1e9f;
    reset_view();
}

void Camera::reset_view()
{
    if (up.z == 0.0f)
    {
        pos = Vec3(0.0f, 2.0f, 10.0f);
        forward = Vec3(0.0f, 0.0f, -1.0f);
    }
    else
    {
        pos = Vec3(10.0f, 2.0f, 0.0f);
        forward = Vec3(-1.0f, 0.0f, 0.0f);
    }
    right = forward.cross(up);
    right.normalize();
}

void Camera::look_at(const Vec3 &position, const Vec3 &target)
{
    pos = position;
    forward = target - pos;
    forward.normalize();
    up = Vec3(0.0f, 1.0f, 0.0f);
    right = forward.cross(up);
    right.normalize();
    up = right.cross(forward);
    up.normalize();
}

void Camera::handle_mouse_view(int dx, int dy)
{
    forward.normalize();
    right = forward.cross(up);
    right.normalize();

    Quat qx(k_pi * static_cast<float>(-dx) * k_degrees_per_pixel / 180.0f, up);
    forward = qx.rotate(forward);
    // Pitch around the yawed right axis so the render basis stays orthonormal.
    right = forward.cross(up);
    right.normalize();
    Quat qy(k_pi * static_cast<float>(-dy) * k_degrees_per_pixel / 180.0f, right);
    forward = qy.rotate(forward);
    forward.normalize();
    up = right.cross(forward);
    up.normalize();
}

void Camera::handle_mouse_translate(int dx, int dy, float scale)
{
    pos -= right * scale * static_cast<float>(dx);
    pos += up * scale * static_cast<float>(dy);
}

void Camera::handle_mouse_orbit(int dx, int dy, const Vec3 &center)
{
    Mat33 initial_basis;
    initial_basis.column0 = right;
    initial_basis.column1 = forward;
    initial_basis.column2 = up;

    up.normalize();
    right.normalize();

    Quat q(static_cast<float>(-dx) * 0.01f, up);
    q = q * Quat(static_cast<float>(-dy) * 0.01f, right);
    q.normalize();

    forward = q.rotate(forward);
    up = q.rotate(up);
    right = q.rotate(right);

    right -= orbitUp * right.dot(orbitUp);
    right.normalize();
    up = right.cross(forward);
    up.normalize();
    forward = up.cross(right);
    forward.normalize();

    Mat33 rotated_basis;
    rotated_basis.column0 = right;
    rotated_basis.column1 = forward;
    rotated_basis.column2 = up;

    const Mat33 delta = rotated_basis * initial_basis.getTranspose();
    pos = center + delta * (pos - center);
}

void Camera::handle_key_down(int key)
{
    if (key == GLFW_KEY_W)
        pos += forward * speed;
    if (key == GLFW_KEY_S)
        pos -= forward * speed;
    if (key == GLFW_KEY_A)
        pos -= right * speed;
    if (key == GLFW_KEY_D)
        pos += right * speed;
    if (key == GLFW_KEY_E)
        pos -= up * speed;
    if (key == GLFW_KEY_Q)
        pos += up * speed;
}

void Camera::handle_wheel(float rotation)
{
    if (orthographic)
        orthographicHeight *= std::exp(-0.1f * std::clamp(rotation, -10.0f, 10.0f));
    else
        pos += rotation * forward * speed;
}

} // namespace viewer

viewer::Camera::Ray viewer::Camera::screenRay(float x, float y, float width, float height) const
{
    if (width <= 0 || height <= 0) return {pos, forward};
    const float sx = (2*x/width - 1)*width/height + 2*horizontalOffset;
    const float sy = 1 - 2*y/height + 2*verticalOffset;
    if (orthographic)
        return {pos + (right*sx + up*sy)*(orthographicHeight*0.5f), forward};
    Vec3 direction = forward + (right*sx + up*sy)*std::tan(fov*3.14159265f/360);
    direction.normalize();
    return {pos, direction};
}
