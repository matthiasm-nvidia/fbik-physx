// Adapted from decomposer/src/ConvexShape.cpp: OBB refinement and plane clipping.
#include "ConvexHull.h"



#include <cmath>
#include <utility>

namespace MeshTools
{
namespace
{
// Cut tolerance, relative to the size of the shape being cut.
constexpr float kCutEpsilonScale = 1.0e-4f;

// Corner c of a box takes the upper extent along axis a when bit a of c is set. Face loops
// are counter-clockwise seen from outside for a right-handed axis frame.
constexpr int kBoxFaces[6][4] = {
    {0, 4, 6, 2}, // -axis0
    {1, 3, 7, 5}, // +axis0
    {0, 1, 5, 4}, // -axis1
    {2, 6, 7, 3}, // +axis1
    {0, 2, 3, 1}, // -axis2
    {4, 5, 7, 6}, // +axis2
};

// Edge of the shape being cut that crosses the plane, and the vertex created on it.
struct Cut
{
    std::uint32_t i0;
    std::uint32_t i1;
    std::uint32_t new_id;
};

// Vertex of the kept part of a face, and whether it lies in the cut plane.
struct LoopVertex
{
    std::uint32_t id;
    bool on_plane;
};

// Directed edge of the closing face.
struct RingEdge
{
    std::uint32_t from;
    std::uint32_t to;
};

int findCut(const std::vector<Cut>& cuts, std::uint32_t i0, std::uint32_t i1)
{
    for (std::size_t i = 0; i < cuts.size(); ++i)
    {
        if ((cuts[i].i0 == i0 && cuts[i].i1 == i1) || (cuts[i].i0 == i1 && cuts[i].i1 == i0))
            return static_cast<int>(i);
    }
    return -1;
}

int findRingEdge(const std::vector<RingEdge>& ring, std::uint32_t from)
{
    for (std::size_t i = 0; i < ring.size(); ++i)
    {
        if (ring[i].from == from)
            return static_cast<int>(i);
    }
    return -1;
}

void addFace(ConvexShape& shape, std::uint32_t start)
{
    shape.face_starts.push_back(start);
    shape.face_sizes.push_back(static_cast<std::uint32_t>(shape.indices.size()) - start);
}

// Spherical Fibonacci lattice: constant z steps combined with a golden angle turn spread
// the directions evenly without clustering at the poles.
Vec3 fibonacciDirection(int index, int count)
{
    const float golden_angle = Pi * (3.0f - std::sqrt(5.0f));
    const float z = 1.0f - 2.0f * (static_cast<float>(index) + 0.5f) / static_cast<float>(count);
    const float radius = std::sqrt(Max(0.0f, 1.0f - z * z));
    const float angle = golden_angle * static_cast<float>(index);
    return Vec3(radius * std::cos(angle), radius * std::sin(angle), z);
}

} // namespace

void ConvexShape::clear()
{
    vertices.clear();
    indices.clear();
    face_starts.clear();
    face_sizes.clear();
}

float ConvexShape::volume() const
{
    float six_volume = 0.0f;
    for (std::size_t face = 0; face < face_sizes.size(); ++face)
    {
        const std::uint32_t start = face_starts[face];
        const std::uint32_t size = face_sizes[face];
        const Vec3& v0 = vertices[indices[start]];
        for (std::uint32_t i = 1; i + 1 < size; ++i)
        {
            const Vec3& v1 = vertices[indices[start + i]];
            const Vec3& v2 = vertices[indices[start + i + 1]];
            six_volume += v0.dot(v1.cross(v2));
        }
    }
    return six_volume / 6.0f;
}

bool splitBelow(const ConvexShape& in, const Plane& plane, ConvexShape& out)
{
    out.clear();
    if (in.empty())
        return false;

    Bounds3 bounds(Empty);
    for (const Vec3& vertex : in.vertices)
        bounds.include(vertex);

    const float diagonal = bounds.getDimensions().length();
    const float eps = kCutEpsilonScale * (diagonal > 0.0f ? diagonal : 1.0f);

    // One sign per vertex rather than one per face corner, so a vertex is classified the
    // same way in every face that uses it. A vertex within the tolerance of the plane counts
    // as lying in it and is kept without cutting any of the edges it sits on, which is what
    // keeps a support plane touching the shape from producing slivers.
    std::vector<float> heights(in.vertices.size());
    std::vector<int> signs(in.vertices.size());
    int num_below = 0;
    int num_above = 0;
    for (std::size_t i = 0; i < in.vertices.size(); ++i)
    {
        heights[i] = plane.height(in.vertices[i]);
        signs[i] = heights[i] < -eps ? -1 : (heights[i] > eps ? 1 : 0);
        num_below += signs[i] < 0 ? 1 : 0;
        num_above += signs[i] > 0 ? 1 : 0;
    }

    if (num_below == 0) // the plane leaves nothing below it
        return false;
    if (num_above == 0) // the plane misses the shape
    {
        out = in;
        return true;
    }

    std::vector<int> new_index(in.vertices.size(), -1);
    std::vector<Cut> cuts;
    std::vector<RingEdge> ring;
    std::vector<LoopVertex> loop;

    for (int face = 0; face < in.numFaces(); ++face)
    {
        const std::uint32_t start = in.face_starts[face];
        const int size = static_cast<int>(in.face_sizes[face]);

        loop.clear();
        bool has_below = false;
        for (int j = 0; j < size; ++j)
        {
            const std::uint32_t i0 = in.indices[start + j];
            const std::uint32_t i1 = in.indices[start + (j + 1) % size];
            has_below = has_below || signs[i0] < 0;

            if (signs[i0] <= 0)
            {
                if (new_index[i0] < 0)
                {
                    new_index[i0] = static_cast<int>(out.vertices.size());
                    out.vertices.push_back(in.vertices[i0]);
                }
                loop.push_back({static_cast<std::uint32_t>(new_index[i0]), signs[i0] == 0});
            }

            if (signs[i0] * signs[i1] >= 0)
                continue;

            int cut = findCut(cuts, i0, i1);
            if (cut < 0)
            {
                // The ends are strictly on opposite sides, so their heights differ by more
                // than 2 * eps and the division is safe.
                const float t = heights[i0] / (heights[i0] - heights[i1]);
                cut = static_cast<int>(cuts.size());
                cuts.push_back({i0, i1, static_cast<std::uint32_t>(out.vertices.size())});
                out.vertices.push_back(in.vertices[i0] + (in.vertices[i1] - in.vertices[i0]) * t);
            }
            loop.push_back({cuts[cut].new_id, true});
        }

        // Every edge of the kept loop that lies in the plane bounds the hole the plane left,
        // so the closing face is those edges reversed. A face that never reaches below the
        // plane only touches it, so its loop is an open chain with no edge from end to start.
        const std::size_t num_edges = has_below ? loop.size() : (loop.empty() ? 0 : loop.size() - 1);
        for (std::size_t j = 0; j < num_edges; ++j)
        {
            const LoopVertex& a = loop[j];
            const LoopVertex& b = loop[(j + 1) % loop.size()];
            if (a.on_plane && b.on_plane)
                ring.push_back({b.id, a.id});
        }

        // Only a face reaching below the plane keeps any area.
        if (!has_below)
            continue;
        if (loop.size() < 3)
        {
            out.clear();
            return false;
        }

        const std::uint32_t face_start = static_cast<std::uint32_t>(out.indices.size());
        for (const LoopVertex& vertex : loop)
            out.indices.push_back(vertex.id);
        addFace(out, face_start);
    }

    // The ring edges form one loop around the hole, so following them from any of their ends
    // visits each of them once and closes the shape with a single face. Anything else means
    // the tolerance misclassified a vertex somewhere, and the shape cannot be trusted.
    if (ring.size() < 3)
    {
        out.clear();
        return false;
    }

    const std::uint32_t closing_start = static_cast<std::uint32_t>(out.indices.size());
    std::uint32_t vertex = ring[0].from;
    for (std::size_t i = 0; i < ring.size(); ++i)
    {
        // A vertex visited twice means the edges form more than one loop, and following them
        // would emit one of those loops repeatedly instead of a single closing face.
        for (std::size_t j = closing_start; j < out.indices.size(); ++j)
        {
            if (out.indices[j] == vertex)
            {
                out.clear();
                return false;
            }
        }
        out.indices.push_back(vertex);

        const int edge = findRingEdge(ring, vertex);
        if (edge < 0)
        {
            out.clear();
            return false;
        }
        vertex = ring[edge].to;
    }

    if (vertex != ring[0].from) // the edges do not form a single loop
    {
        out.clear();
        return false;
    }
    addFace(out, closing_start);
    return true;
}

void refineConvexHull(const std::vector<Vec3>& positions, const Vec3& halfExtents,
                      float padding, ConvexShape& out)
{
    ConvexShape hull;
    if (positions.empty()) { out.clear(); return; }
    for (int c=0;c<8;++c)
        hull.vertices.emplace_back((c&1)?halfExtents.x:-halfExtents.x,
                                   (c&2)?halfExtents.y:-halfExtents.y,
                                   (c&4)?halfExtents.z:-halfExtents.z);
    for (const auto& face:kBoxFaces) {
        const auto start=static_cast<std::uint32_t>(hull.indices.size());
        for (int corner:face) hull.indices.push_back(corner);
        addFace(hull,start);
    }
    // At most 70 faces / 136 vertices for a nondegenerate convex polyhedron,
    // below PhysX's 255-vertex limit without further simplification.
    constexpr int directions=64;
    ConvexShape clipped;
    for(int i=0;i<directions;++i) {
        const Vec3 direction=fibonacciDirection(i,directions);
        float support=-MaxFloat;
        for(const auto& p:positions) support=Max(support,p.dot(direction));
        if(splitBelow(hull,Plane(direction,support+padding),clipped)) std::swap(hull,clipped);
    }
    out=std::move(hull);
}
} // namespace MeshTools
