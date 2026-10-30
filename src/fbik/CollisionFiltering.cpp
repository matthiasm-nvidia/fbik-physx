#include "CollisionFiltering.h"
#include <algorithm>

namespace fbik::collision
{
using namespace MeshTools;
Capsule restCapsule(const AffineTransform &pose, float length, float radius)
{
    const auto halfAxis=pose.A.column0*(0.5f*length);
    return {pose.b-halfAxis,pose.b+halfAxis,
            radius*std::max(pose.A.column1.magnitude(),pose.A.column2.magnitude())};
}
bool overlaps(const Capsule &a, const Capsule &b)
{
    const Vec3 u=a.p1-a.p0,v=b.p1-b.p0,w=a.p0-b.p0;
    const auto dot=[](const Vec3 &x,const Vec3 &y) {
        return double(x.x)*y.x+double(x.y)*y.y+double(x.z)*y.z;
    };
    const double uu=dot(u,u),vv=dot(v,v),uv=dot(u,v),uw=dot(u,w),vw=dot(v,w);
    const auto clamp=[](double t) { return std::clamp(t,0.0,1.0); };
    double s=0,t=0;
    // Closest points on the center segments, including zero-length spheres.
    if (uu==0) {
        if (vv>0) t=clamp(vw/vv);
    } else if (vv==0) {
        s=clamp(-uw/uu);
    } else {
        const double denominator=uu*vv-uv*uv;
        if (denominator>0) s=clamp((uv*vw-uw*vv)/denominator);
        t=(uv*s+vw)/vv;
        if (t<0) { t=0;s=clamp(-uw/uu); }
        else if (t>1) { t=1;s=clamp((uv-uw)/uu); }
    }
    const double dx=w.x+s*u.x-t*v.x,dy=w.y+s*u.y-t*v.y,dz=w.z+s*u.z-t*v.z;
    const double radius=double(a.radius)+b.radius;
    // Only allow floating-point roundoff at contact; no bounds padding.
    return dx*dx+dy*dy+dz*dz<=radius*radius*(1+1e-5);
}
}
