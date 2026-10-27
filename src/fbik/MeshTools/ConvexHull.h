#pragma once

#include "Vec.h"

#include <cstdint>
#include <vector>

namespace MeshTools
{

// Convex polyhedron as a vertex list plus one index loop per face. Face f owns the
// indices [face_starts[f], face_starts[f] + face_sizes[f]), wound counter-clockwise seen
// from outside, so face normals point outward and volume() comes out positive.
class ConvexShape
{
public:
    void clear();

    bool empty() const
    {
        return vertices.empty() || face_sizes.empty();
    }

    int numFaces() const
    {
        return static_cast<int>(face_sizes.size());
    }

    // Signed volume of the origin tetrahedra of every fan triangle of every face. The
    // origin cancels out for a closed shape, so the position of the shape does not matter.
    float volume() const;

    std::vector<Vec3> vertices;
    std::vector<std::uint32_t> indices;
    std::vector<std::uint32_t> face_starts;
    std::vector<std::uint32_t> face_sizes;
};

// Refine an origin-centered OBB by support planes of points in its local frame.
// padding keeps nearly planar/linear inputs cookable; it is in the same units as points.
void refineConvexHull(const std::vector<Vec3>& points, const Vec3& halfExtents,
                      float padding, ConvexShape& out);
} // namespace MeshTools
