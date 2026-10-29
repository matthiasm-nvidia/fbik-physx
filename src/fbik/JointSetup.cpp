#include "JointSetup.h"
#include <algorithm>
#include <cctype>
#include <limits>

namespace fbik::joints
{
namespace
{
bool has(const std::string &name, const char *part) { return name.find(part)!=std::string::npos; }
Vec3 unit(Vec3 v, Vec3 fallback)
{
    return v.magnitudeSquared()>1e-12f ? v.normalized() : fallback;
}
int side(const std::string &name)
{
    if (has(name,"left") || name.rfind("l_",0)==0 || name.rfind("l.",0)==0 ||
        (name.size()>2 && (name.substr(name.size()-2)=="_l" || name.substr(name.size()-2)==".l"))) return -1;
    if (has(name,"right") || name.rfind("r_",0)==0 || name.rfind("r.",0)==0 ||
        (name.size()>2 && (name.substr(name.size()-2)=="_r" || name.substr(name.size()-2)==".r"))) return 1;
    return 0;
}
std::string anatomicalName(const std::string &path)
{
    auto name=boneName(path);
    name.erase(std::remove_if(name.begin(),name.end(),[](unsigned char c) { return !std::isalnum(c); }),name.end());
    return name;
}
bool descendant(const Skeleton &sk, size_t j, size_t ancestor)
{
    for (int p=sk.parents[j];p>=0;p=sk.parents[p]) if (size_t(p)==ancestor) return true;
    return false;
}
Vec3 incoming(const Skeleton &sk, const std::vector<Matrix> &rest, size_t j, Vec3 fallback)
{
    for (int p=sk.parents[j];p>=0;p=sk.parents[p]) {
        const auto v=rest[j].b-rest[p].b;
        if (v.magnitudeSquared()>1e-12f) return v.normalized();
    }
    return fallback;
}
Vec3 outgoing(const Skeleton &sk, const std::vector<Matrix> &rest, size_t j, Vec3 fallback)
{
    Vec3 sum(Zero);
    // Use the nearest nonzero descendant on each branch, traversing zero-length helpers.
    std::vector<int> pending=sk.children[j];
    while (!pending.empty()) {
        const int c=pending.back();pending.pop_back();
        const auto v=rest[c].b-rest[j].b;
        if (v.magnitudeSquared()>1e-12f) sum+=v;
        else pending.insert(pending.end(),sk.children[c].begin(),sk.children[c].end());
    }
    return unit(sum,fallback);
}
Mat33 basis(Vec3 x, Vec3 hint)
{
    x=unit(x,{1,0,0});
    Vec3 y=hint-x*hint.dot(x);
    if (y.magnitudeSquared()<1e-8f) {
        const Vec3 alternate=std::abs(x.y)<.9f ? Vec3(0,1,0) : Vec3(0,0,1);
        y=alternate-x*alternate.dot(x);
    }
    y.normalize();
    return Mat33(x,y,x.cross(y));
}
} // namespace

std::string boneName(const std::string &path)
{
    auto name=path.substr(path.find_last_of('/')+1);
    std::transform(name.begin(),name.end(),name.begin(),[](unsigned char c) { return char(std::tolower(c)); });
    return name;
}
std::vector<Matrix> restPose(const Skeleton &sk)
{
    std::vector<Matrix> poses;
    for (size_t j=0;j<sk.names.size();++j)
        poses.push_back(sk.parents[j]<0 ? sk.toCharacter*sk.restLocal[j] : poses[sk.parents[j]]*sk.restLocal[j]);
    return poses;
}
CharacterFrame characterFrame(const Skeleton &sk, const std::vector<Matrix> &rest,
                              const std::vector<Vec3> &skin, const Matrix &characterWorld)
{
    Vec3 minimum(MaxFloat,MaxFloat,MaxFloat),maximum(-MaxFloat,-MaxFloat,-MaxFloat);
    const auto include=[&](Vec3 point) {
        point=characterWorld.transform(point);
        for (int a=0;a<3;++a) { minimum[a]=std::min(minimum[a],point[a]);maximum[a]=std::max(maximum[a],point[a]); }
    };
    for (const auto &v:skin) include(v);
    if (skin.empty()) for (const auto &pose:rest) include(pose.b);
    const Vec3 extent=maximum-minimum;
    // A rotated T-pose can be wider than it is tall along global Y/Z too.
    // Use head-to-feet separation to disambiguate the upright axis when available.
    Vec3 heads(Zero),feet(Zero);
    size_t headCount=0,footCount=0;
    for (size_t j=0;j<sk.names.size();++j) {
        const auto name=boneName(sk.names[j]);
        const auto parent=sk.parents[j]>=0 ? boneName(sk.names[sk.parents[j]]) : std::string();
        if (has(name,"head") && !has(parent,"head")) { heads+=rest[j].b;++headCount; }
        if ((has(name,"foot") || has(name,"ankle")) && !has(name,"toe")) { feet+=rest[j].b;++footCount; }
    }
    Vec3 height=extent;
    if (headCount && footCount) {
        const Vec3 direction=characterWorld.A*(heads/float(headCount)-feet/float(footCount));
        if (std::max(std::abs(direction.y),std::abs(direction.z))>1e-6f)
            height={std::abs(direction.x),std::abs(direction.y),std::abs(direction.z)};
    }
    // Up still points toward increasing values on global Y or Z.
    const int upAxis=height.y>=height.z ? 1 : 2;
    const Vec3 up=upAxis==1 ? Vec3(0,1,0) : Vec3(0,0,1);
    Vec3 forward(Zero);
    for (size_t foot=0;foot<sk.names.size();++foot) {
        const auto name=boneName(sk.names[foot]);
        if ((!has(name,"foot") && !has(name,"ankle")) || has(name,"toe")) continue;
        int toe=-1,bestDepth=std::numeric_limits<int>::max();bool bestBase=false;
        for (size_t j=foot+1;j<sk.names.size();++j) {
            const auto toeName=anatomicalName(sk.names[j]);
            if (!has(toeName,"toe") || !descendant(sk,j,foot)) continue;
            const bool base=has(toeName,"toebase");
            int depth=0;for (int p=int(j);p!=int(foot);p=sk.parents[p]) ++depth;
            if (toe<0 || (base && !bestBase) || (base==bestBase && depth<bestDepth))
            { toe=int(j);bestDepth=depth;bestBase=base; }
        }
        if (toe>=0) {
            Vec3 direction=characterWorld.A*(rest[toe].b-rest[foot].b);
            direction-=up*direction.dot(up);
            forward+=unit(direction,Vec3(Zero));
        }
    }
    if (forward.magnitudeSquared()<1e-6f) {
        // Incomplete rigs: paired left/right landmarks can still resolve the sign.
        Vec3 right(Zero);
        for (size_t j=0;j<sk.names.size();++j) {
            auto leftName=boneName(sk.names[j]);
            if (!has(leftName,"left")) continue;
            leftName.replace(leftName.find("left"),4,"right");
            for (size_t k=0;k<sk.names.size();++k) if (boneName(sk.names[k])==leftName)
                right+=characterWorld.A*(rest[k].b-rest[j].b);
        }
        right-=up*right.dot(up);
        forward=up.cross(right);
    }
    if (forward.magnitudeSquared()<1e-6f) {
        const int other=upAxis==1 ? 2 : 1;
        const int shortest=extent.x<extent[other] ? 0 : other;
        forward=Vec3(Zero);forward[shortest]=1;
    }
    // Return the frame in character coordinates, keeping it orthonormal under scale.
    const auto toCharacter=viewer::skeleton::inverse(characterWorld);
    const auto z=unit(toCharacter.A*up,{0,1,0});
    auto y=toCharacter.A*forward;y-=z*y.dot(z);y.normalize();
    return {y.cross(z),y,z};
}
Setup setup(const Skeleton &sk, const std::vector<Matrix> &rest, const CharacterFrame &frame, size_t j)
{
    const auto name=anatomicalName(sk.names[j]);
    const auto parent=sk.parents[j]>=0 ? anatomicalName(sk.names[sk.parents[j]]) : std::string();
    const bool knee=has(name,"knee") || has(name,"shin") || has(name,"calf") || has(name,"lowerleg") ||
                    (has(name,"leg") && (has(parent,"leg") || has(parent,"thigh")) && !has(name,"upper"));
    const bool elbow=has(name,"elbow") || has(name,"forearm") || has(name,"lowerarm");
    const Vec3 upper=incoming(sk,rest,j,frame.up);
    const Vec3 lower=outgoing(sk,rest,j,upper);
    Setup result;
    if (knee || elbow) {
        Vec3 fallback=knee ? frame.right : frame.up;
        fallback=basis(upper,fallback).column1; // Hinge must be perpendicular to the limb.
        Vec3 axis=upper.cross(lower);
        if (axis.magnitudeSquared()<std::pow(std::sin(5*Pi/180),2)) axis=fallback;
        else { axis.normalize();if (axis.dot(fallback)<0) axis=-axis; }
        result.parentFrame=basis(axis,upper);
        result.childFrame=basis(axis,lower); // Retains the initial bend; straight is zero.
        int armSide=side(boneName(sk.names[j]));
        if (!armSide) armSide=upper.dot(frame.right)>=0 ? 1 : -1;
        result.limits.hinge=true;
        // About character-right, a negative rotation takes a downward shin
        // away from forward (knee flexion). A positive rotation hyperextends it.
        result.limits.lower=knee ? -60.f : (armSide<0 ? -120.f : 0.f);
        result.limits.upper=knee || armSide<0 ? 0.f : 120.f;
        result.limits.swing=0;
    }
    else {
        if (has(name,"head")) result.limits={false,-20,20,30};
        else if (has(name,"arm") && !has(parent,"arm")) result.limits={false,-60,60,100};
        else if (has(name,"leg") && !has(parent,"leg")) result.limits={false,-30,30,60};
        const Vec3 longitudinal=has(name,"head") ? upper : lower;
        result.parentFrame=result.childFrame=basis(longitudinal,frame.up);
    }
    return result;
}
} // namespace fbik::joints
