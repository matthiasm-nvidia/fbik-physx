#pragma once
#include "MeshTools/Vec.h"

namespace fbik::collision
{
struct Capsule
{
    MeshTools::Vec3 p0, p1; // Centers of the two hemispherical ends.
    float radius;
};
Capsule restCapsule(const MeshTools::AffineTransform &pose, float length, float radius);
bool overlaps(const Capsule &a, const Capsule &b);
}
