#pragma once
#include "StageData.h"
#include "MeshTools/Vec.h"
#include <array>
#include <cmath>
#include <map>
#include <stdexcept>

// Shared initial-pose decoding; playback/evaluation is called by the viewer.
namespace viewer::skeleton {
namespace st = viewer::stage;
using namespace MeshTools;
using MeshTools::Vec3;
using MeshTools::Mat33;
using MeshTools::Quat;
using MeshTools::Identity;
using MeshTools::Zero;
using Matrix = AffineTransform;
inline void require(bool condition, const std::string &message)
{
    if (!condition) throw std::runtime_error("FBIK: " + message);
}
inline const st::Value &get(const st::Prim &prim, const std::string &name)
{
    static const st::Value empty;
    auto it = prim.find(name);
    return it == prim.end() ? empty : it->second;
}
inline std::string parent(const std::string &path)
{
    auto slash = path.rfind('/');
    return slash == std::string::npos || slash == 0 ? "/" : path.substr(0, slash);
}
inline const st::Value &inherited(const st::Snapshot &data, std::string path, const std::string &name)
{
    for (;; path = parent(path))
    {
        auto it = data.find(path);
        if (it != data.end() && it->second.count(name)) return it->second.at(name);
        if (path == "/") break;
    }
    static const st::Value empty;
    return empty;
}
inline Matrix matrix(const std::vector<double> &v, size_t offset = 0)
{
    require(v.size() >= offset + 16, "Missing transform matrix.");
    for (size_t i = offset; i < offset + 16; ++i) require(std::isfinite(v[i]), "Non-finite transform.");
    const auto p = v.data() + offset;
    return Matrix(Mat33(Vec3(float(p[0]),float(p[1]),float(p[2])),
                        Vec3(float(p[4]),float(p[5]),float(p[6])),
                        Vec3(float(p[8]),float(p[9]),float(p[10]))),
                  Vec3(float(p[12]),float(p[13]),float(p[14])));
}
inline Matrix inverse(const Matrix &m)
{
    require(std::abs(m.A.getDeterminant()) > 1e-18f, "Singular transform.");
    return m.getInverse();
}
inline Matrix world(const st::Prim &p)
{
    const auto &v = get(p, "omni:fabric:worldMatrix").numbers;
    return v.empty() ? Matrix(Identity) : matrix(v);
}
struct Skeleton
{
    std::string path, character;
    std::vector<std::string> names;
    std::vector<int> parents;
    std::vector<std::vector<int>> children;
    std::vector<Matrix> jointPose, inverseBind, restLocal;
    Matrix toCharacter{Identity};
    std::vector<std::vector<Vec3>> points;
};
inline void applyAnimation(std::vector<Matrix> &local, const st::Prim &a, const std::vector<int> &mapping)
{
    const auto &ts = get(a,"translations").numbers, &qs = get(a,"rotations").numbers, &ss = get(a,"scales").numbers;
    require((ts.empty() || ts.size()==mapping.size()*3) && (qs.empty() || qs.size()==mapping.size()*4) &&
            (ss.empty() || ss.size()==mapping.size()*3), "Invalid animation transform counts.");
    for (size_t i=0; i<mapping.size(); ++i)
    {
        if (mapping[i] < 0) continue;
        auto &m = local[size_t(mapping[i])];
        Vec3 scale(m.A.column0.magnitude(),m.A.column1.magnitude(),m.A.column2.magnitude());
        require(scale.x > 0 && scale.y > 0 && scale.z > 0, "Degenerate joint scale.");
        Mat33 rotation(m.A.column0/scale.x,m.A.column1/scale.y,m.A.column2/scale.z);
        if (!qs.empty())
        {
            Quat q{float(qs[4*i]),float(qs[4*i+1]),float(qs[4*i+2]),float(qs[4*i+3])};
            const float length = std::sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w);
            require(std::isfinite(length) && length > 1e-8f, "Invalid animated quaternion.");
            rotation = Mat33(q*(1.0f/length));
        }
        if (!ss.empty()) scale = Vec3(float(ss[3*i]),float(ss[3*i+1]),float(ss[3*i+2]));
        m.A = Mat33(rotation.column0*scale.x,rotation.column1*scale.y,rotation.column2*scale.z);
        if (!ts.empty()) m.b = Vec3(float(ts[3*i]),float(ts[3*i+1]),float(ts[3*i+2]));
    }
}
inline Skeleton readSkeleton(const st::Snapshot &data, const std::string &path, const std::string &character)
{
    Skeleton sk;
    sk.path = path; sk.character = character;
    const auto &prim = data.at(path);
    sk.names = get(prim, "joints").tokens;
    require(!sk.names.empty(), "Skeleton has no joints: " + path);
    sk.children.resize(sk.names.size()); sk.points.resize(sk.names.size());
    std::map<std::string, int> indices;
    for (size_t i = 0; i < sk.names.size(); ++i)
    {
        const auto &name = sk.names[i];
        require(!name.empty() && name[0] != '/' && !indices.count(name), "Invalid joint order: " + path);
        int p = -1;
        const auto slash = name.rfind('/');
        if (slash != std::string::npos)
        {
            auto it = indices.find(name.substr(0, slash));
            require(it != indices.end(), "Joint parent must precede its children: " + name);
            p = it->second;
            sk.children[p].push_back(int(i));
        }
        sk.parents.push_back(p); indices[name] = int(i);
    }
    const auto &rest = get(prim, "restTransforms").numbers;
    const auto &bind = get(prim, "bindTransforms").numbers;
    require(rest.size() == sk.names.size()*16 || bind.size() == sk.names.size()*16,
            "Skeleton needs rest or bind transforms: " + path);
    std::vector<Matrix> local;
    for (size_t i = 0; i < sk.names.size(); ++i)
    {
        if (!rest.empty()) local.push_back(matrix(rest, i*16));
        else local.push_back(sk.parents[i] < 0 ? matrix(bind,i*16) : inverse(matrix(bind,sk.parents[i]*16))*matrix(bind,i*16));
        if (!bind.empty()) sk.inverseBind.push_back(inverse(matrix(bind,i*16)));
    }
    sk.restLocal = local;
    const auto &animation = inherited(data, path, "skel:animationSource").paths;
    require(animation.size() <= 1, "Multiple animation sources: " + path);
    if (!animation.empty())
    {
        require(data.count(animation[0]) != 0, "Missing animation source: " + animation[0]);
        const auto &a = data.at(animation[0]);
        const auto &names = get(a, "joints").tokens;
        std::vector<int> mapping;
        for (const auto &name : names) mapping.push_back(indices.count(name) ? indices.at(name) : -1);
        applyAnimation(local, a, mapping);
    }
    for (size_t i=0; i<local.size(); ++i)
        sk.jointPose.push_back(sk.parents[i] < 0 ? local[i] : sk.jointPose[sk.parents[i]]*local[i]);
    sk.toCharacter = inverse(world(data.at(character))) * world(prim);
    return sk;
}

inline int integer(double v)
{
    require(std::isfinite(v) && v >= 0 && v <= 2147483647 && std::floor(v) == v, "Invalid skin index or element size.");
    return int(v);
}
// Flatten indexed primvars by influence group, respecting constant vs vertex interpolation.
inline std::vector<double> influences(const st::Snapshot &data, const std::string &path, const std::string &name,
                               size_t vertices, size_t &elementSize)
{
    const auto &size = inherited(data,path,name+":usd-metadata:elementSize").numbers;
    elementSize = size.empty() ? 1 : size_t(integer(size[0]));
    require(elementSize > 0 && elementSize <= 256, "Invalid skin elementSize: " + path);
    const auto &interpolation = inherited(data,path,name+":usd-metadata:interpolation").token;
    require(interpolation=="vertex" || interpolation=="constant", "Unsupported skin interpolation: " + path);
    const auto &values = inherited(data,path,name).numbers;
    const auto &indices = inherited(data,path,name+":indices").numbers;
    const size_t groups = interpolation=="constant" ? 1 : vertices;
    require(values.size()%elementSize==0 && (indices.empty() ? values.size()==groups*elementSize : indices.size()==groups),
            "Skin primvar length mismatch: " + path);
    std::vector<double> result;
    result.reserve(vertices*elementSize);
    for (size_t v=0; v<vertices; ++v)
    {
        size_t group = interpolation=="constant" ? 0 : v;
        if (!indices.empty()) group = size_t(integer(indices[group]));
        require((group+1)*elementSize <= values.size(), "Indexed skin primvar out of range.");
        result.insert(result.end(), values.begin()+group*elementSize, values.begin()+(group+1)*elementSize);
    }
    return result;
}
inline std::array<double,16> packed(const Matrix &m) {
    return {m.A.column0.x,m.A.column0.y,m.A.column0.z,0,
            m.A.column1.x,m.A.column1.y,m.A.column1.z,0,
            m.A.column2.x,m.A.column2.y,m.A.column2.z,0,m.b.x,m.b.y,m.b.z,1};
}
inline Matrix unpacked(const std::array<double,16> &m) {return matrix(std::vector<double>(m.begin(),m.end()));}
} // namespace viewer::skeleton
