#include "Fbik.h"
#include "MeshTools/Geometry.h"
#include "MeshTools/ConvexHull.h"
#include "StageData.h"
#include "SkeletonPose.h"
#include <ovx/path_dictionary/path_dictionary_utils.h>
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <locale>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>

namespace fbik
{
namespace
{
namespace st = viewer::stage;
using namespace MeshTools;
using namespace viewer::skeleton;
void populationWait(ovstage_instance_t *stage, ovstage_population_enqueue_result_t op)
{
    st::check(op.status);
    ovstage_population_op_wait_result_t result{};
    const auto status = ovstage_population_wait_op(stage, op.op_index, OVSTAGE_TIMEOUT_INFINITE, &result);
    if (result.error_op_id_count)
    {
        const auto error = ovstage_population_get_last_op_error(result.error_op_ids[0]);
        throw std::runtime_error("FBIK USD: " + std::string(error.ptr, error.length));
    }
    st::check(status);
}
std::string quote(const std::string &s)
{
    std::string result = "\"";
    for (char c : s) { if (c == '"' || c == '\\') result += '\\'; result += c; }
    return result + '"';
}
void vector(std::ostream &out, const Vec3 &v) { out << '(' << v.x << ',' << v.y << ',' << v.z << ')'; }
void pose(std::ostream &out, const Vec3 &p, const Quat &q)
{
    out << "double3 xformOp:translate = "; vector(out, p);
    out << "\nquatf xformOp:orient = (" << q.w << ',' << q.x << ',' << q.y << ',' << q.z << ")\n";
}
void collectSkin(const st::Snapshot &data, const std::string &path, Skeleton &sk, float minSkinWeight)
{
    const auto &prim = data.at(path);
    const auto &iv = inherited(data,path,"primvars:skel:jointIndices").numbers;
    const auto &wv = inherited(data,path,"primvars:skel:jointWeights").numbers;
    if (iv.empty() && wv.empty()) return; // Hierarchy fallback handles a skeleton without skin.
    require(!iv.empty() && !wv.empty(), "Incomplete skin weights: " + path);
    require(sk.inverseBind.size()==sk.names.size(), "Skinned skeleton requires bind transforms: " + sk.path);
    const auto &method = inherited(data,path,"primvars:skel:skinningMethod").token;
    require(method.empty() || method=="classicLinear", "Only classicLinear skinning is supported: " + path);
    require(get(prim,"skel:blendShapeTargets").paths.empty(), "Blend shapes are not supported yet: " + path);
    const auto &points = get(prim,"points").numbers;
    require(points.size()%3==0, "Invalid mesh points: " + path);
    size_t ni=0,nw=0;
    const auto indices = influences(data,path,"primvars:skel:jointIndices",points.size()/3,ni);
    const auto weights = influences(data,path,"primvars:skel:jointWeights",points.size()/3,nw);
    require(ni==nw, "Joint index/weight element sizes differ: " + path);
    auto order = inherited(data,path,"skel:joints").tokens;
    if (order.empty()) order = sk.names;
    std::vector<size_t> remap;
    for (const auto &name : order)
    {
        auto it = std::find(sk.names.begin(),sk.names.end(),name);
        require(it!=sk.names.end(), "Skin joint not in skeleton: " + name);
        remap.push_back(size_t(it-sk.names.begin()));
    }
    const auto &geom = inherited(data,path,"primvars:skel:geomBindTransform").numbers;
    const Matrix geomBind = geom.empty() ? Matrix(Identity) : matrix(geom);
    std::vector<Matrix> skinning;
    for (size_t i=0; i<sk.names.size(); ++i)
        skinning.push_back(sk.toCharacter * sk.jointPose[i] * sk.inverseBind[i] * geomBind);
    for (size_t v=0; v<points.size()/3; ++v)
    {
        Vec3 p{float(points[3*v]),float(points[3*v+1]),float(points[3*v+2])};
        Vec3 posed(Zero);
        require(std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z), "Non-finite skin vertex.");
        std::set<size_t> bones;
        double total=0;
        for (size_t k=0; k<ni; ++k)
        {
            const size_t offset=v*ni+k;
            const double w=weights[offset];
            require(std::isfinite(w) && w>=0, "Invalid skin weight.");
            if (w==0) continue;
            const size_t index=size_t(integer(indices[offset]));
            require(index<remap.size(), "Joint index out of range.");
            const size_t joint=remap[index];
            posed += skinning[joint].transform(p)*float(w);
            total += w;
            // Evaluate the complete skinned position, but filter hull membership only.
            if (w > minSkinWeight) bones.insert(joint);
        }
        if (total==0) continue;
        require(std::abs(total-1)<0.002, "Skin weights must sum to one: " + path);
        for (size_t joint : bones) sk.points[joint].push_back(posed);
    }
}
void openPath(std::ostream &out, const std::string &path)
{
    size_t offset=1;
    while (offset<path.size())
    {
        const auto end=path.find('/',offset);
        out << "over " << quote(path.substr(offset,end-offset)) << "\n{\n";
        if (end==std::string::npos) break;
        offset=end+1;
    }
}
void closePath(std::ostream &out, const std::string &path)
{
    for (char c : path) if (c=='/') out << "}\n";
}
const std::vector<std::string> attributes = {
    "usd-prim-type", "omni:fabric:worldMatrix", "usd-metadata:metersPerUnit", "joints", "bindTransforms", "restTransforms",
    "translations", "rotations", "scales", "skel:skeleton", "skel:animationSource", "skel:joints", "points",
    "primvars:skel:geomBindTransform", "primvars:skel:skinningMethod", "skel:blendShapeTargets",
    "primvars:skel:jointIndices", "primvars:skel:jointWeights", "primvars:skel:jointIndices:indices", "primvars:skel:jointWeights:indices",
    "primvars:skel:jointIndices:usd-metadata:elementSize", "primvars:skel:jointWeights:usd-metadata:elementSize",
    "primvars:skel:jointIndices:usd-metadata:interpolation", "primvars:skel:jointWeights:usd-metadata:interpolation"};
} // namespace

struct Module::Impl
{
    ovstage_instance_t *stage=nullptr;
    ovstage_population_usd_reference_handle_t layer{};
    bool hasLayer=false;
    std::vector<std::string> roots;
    std::vector<Character> characters;
    struct Body { std::string path, skeleton; size_t joint; Matrix inBone; };
    std::vector<Body> bodies;
};
Module::Module() : impl(std::make_unique<Impl>()) {}
Module::~Module() = default;
InitResult Module::init(ovstage_instance_t *scene, ovstage_ordinal_t &ordinal, float minSkinWeight)
{
    require(std::isfinite(minSkinWeight) && minSkinWeight >= 0 && minSkinWeight <= 1, "Minimum skin weight must be between zero and one.");
    require(scene != nullptr, "init needs a populated stage.");
    require(!impl->stage || impl->stage==scene, "Use one module per stage.");
    impl->stage=scene;
    if (impl->hasLayer)
    {
        populationWait(scene,ovstage_population_remove_usd_reference(scene,impl->layer));
        impl->hasLayer=false;
        populationWait(scene,ovstage_population_apply_usd_changes(scene,++ordinal));
        st::seal(scene,ordinal); st::hierarchy(scene,ordinal);
    }
    impl->roots.clear();
    impl->characters.clear();
    impl->bodies.clear();
    const auto data=st::read(scene,ordinal,attributes);
    InitResult result;
    std::map<std::string,std::vector<Skeleton>> characters;
    for (const auto &[path,prim] : data)
    {
        if (get(prim,"usd-prim-type").token!="Skeleton") continue;
        std::string root=parent(path);
        while (root!="/" && (!data.count(root) || get(data.at(root),"usd-prim-type").token!="SkelRoot")) root=parent(root);
        if (root=="/") continue;
        characters[root].push_back(readSkeleton(data,path,root));
    }
    double units=0.01;
    if (data.count("/"))
    {
        const auto &v=get(data.at("/"),"usd-metadata:metersPerUnit").numbers;
        if (!v.empty()) units=v[0];
    }
    require(std::isfinite(units) && units>0,"Invalid stage units.");
    std::ostringstream usd; usd.imbue(std::locale::classic()); usd << std::setprecision(9) << "#usda 1.0\n";
    std::string openRoot;
    for (auto &[root,skeletons] : characters)
    {
        ++result.characters;
        for (auto &sk : skeletons)
        {
            ++result.skeletons;
            impl->characters.push_back({sk.path, sk.character, sk.names});
            for (const auto &[path,prim] : data)
            {
                if (get(prim,"usd-prim-type").token!="Mesh" || path.rfind(root+"/",0)!=0) continue;
                const auto &binding=inherited(data,path,"skel:skeleton").paths;
                if (binding.size()==1 && binding[0]==sk.path) collectSkin(data,path,sk,minSkinWeight);
            }
        }
        std::string name="__FbikBoneColliders";
        while (data.count(root+"/"+name)) name+="_";
        impl->roots.push_back(root+"/"+name);
        // Keep shared ancestors open: USDA cannot redeclare /World for each character.
        std::string common = openRoot;
        while (!common.empty() && root != common && root.rfind(common + "/", 0) != 0)
            common = parent(common) == "/" ? "" : parent(common);
        for (size_t i = common.size(); i < openRoot.size(); ++i)
            if (openRoot[i] == '/') usd << "}\n";
        openPath(usd, root.substr(common.size()));
        openRoot = root;
        usd << "def Xform " << quote(name) << "\n{\nuniform token visibility = \"invisible\"\n";
        const auto rootWorld=world(data.at(root));
        const float scale=std::max({rootWorld.A.column0.magnitude(),rootWorld.A.column1.magnitude(),rootWorld.A.column2.magnitude()});
        require(scale>0,"Character has zero scale.");
        const float minSize=float(0.001/units)/scale; // Minimum full hull thickness: 1 mm in world space.
        for (size_t si=0; si<skeletons.size(); ++si)
        {
            auto &sk=skeletons[si];
            usd << "def Scope \"Skeleton_" << si << "\"\n{\n";
            for (size_t j=0; j<sk.names.size(); ++j)
            {
                if (sk.children[j].empty()) { ++result.leaves; continue; }
                const bool mesh=!sk.points[j].empty();
                ConvexShape hull;
                Vec3 center(Zero),size(Zero); Quat rotation(Identity); float length=0, radius=0;
                if (mesh)
                {
                    const auto obb=computeOBBFromPoints(sk.points[j]);
                    center=obb.trans.p; rotation=obb.trans.q; size=obb.halfExtents*2;
                    size.x=std::max(size.x,minSize); size.y=std::max(size.y,minSize); size.z=std::max(size.z,minSize);
                    const auto toLocal=inverse(Matrix(Mat33(rotation),center));
                    std::vector<Vec3> localPoints;localPoints.reserve(sk.points[j].size());
                    for(const auto &point:sk.points[j]) localPoints.push_back(toLocal.transform(point));
                    refineConvexHull(localPoints,size*.5f,minSize*.5f,hull);
                    require(!hull.empty() && hull.volume()>0,"Cannot create convex bone mesh.");
                    ++result.convexMeshes;
                }
                else
                {
                    const Vec3 a=sk.toCharacter.transform(sk.jointPose[j].b);
                    Vec3 b(Zero);
                    for (int child : sk.children[j]) b+=sk.toCharacter.transform(sk.jointPose[child].b);
                    b/=float(sk.children[j].size());
                    const Vec3 delta=b-a; length=delta.magnitude();
                    if (length<minSize*0.001f) { ++result.degenerate; continue; }
                    const Vec3 z=delta/length;
                    Vec3 x=(std::abs(z.y)<0.9f ? Vec3(0,1,0) : Vec3(1,0,0)).cross(z); x.normalize();
                    rotation=Quat(Mat33(x,z.cross(x),z)); center=(a+b)*0.5f; radius=length*0.1f;
                    ++result.capsules;
                }
                usd << "def " << (mesh ? "Mesh" : "Capsule") << " \"Joint_" << j << "\" (prepend apiSchemas = [\"PhysicsCollisionAPI\", \"PhysicsRigidBodyAPI\""
                    << (mesh ? ", \"PhysicsMeshCollisionAPI\", \"PhysxConvexHullCollisionAPI\"" : "") << "])\n{\n"
                    << "custom token fbik:joint = " << quote(sk.names[j]) << "\ncustom rel fbik:skeleton = <" << sk.path << ">\n"
                    << "bool physics:rigidBodyEnabled = true\nbool physics:kinematicEnabled = true\n"
                    << "bool physics:collisionEnabled = true\ncolor3f[] primvars:displayColor = " << (mesh ? "[(0.15,0.65,0.95)]" : "[(0.95,0.65,0.1)]") << "\n";
                Mat33 axes(rotation);

                impl->bodies.push_back({root+"/"+name+"/Skeleton_"+std::to_string(si)+"/Joint_"+std::to_string(j),
                    sk.path,j,inverse(sk.toCharacter*sk.jointPose[j])*Matrix(axes,center)});
                pose(usd,center,rotation);
                if (mesh)
                {
                    usd << "uniform token subdivisionScheme = \"none\"\nuniform token physics:approximation = \"convexHull\"\n"
                        << "int physxConvexHullCollision:hullVertexLimit = 255\npoint3f[] points = [";
                    for(size_t v=0;v<hull.vertices.size();++v) { if(v) usd << ',';vector(usd,hull.vertices[v]); }
                    usd << "]\nint[] faceVertexCounts = [";
                    for(size_t f=0;f<hull.face_sizes.size();++f) {if(f) usd << ',';usd << hull.face_sizes[f];}
                    usd << "]\nint[] faceVertexIndices = [";
                    for(size_t k=0;k<hull.indices.size();++k) {if(k) usd << ',';usd << hull.indices[k];}
                    usd << "]\nuniform token[] xformOpOrder = [\"xformOp:translate\",\"xformOp:orient\"]\n";
                }
                else usd << "double height = " << length << "\ndouble radius = " << radius
                         << "\nuniform token axis = \"Z\"\nuniform token[] xformOpOrder = [\"xformOp:translate\",\"xformOp:orient\"]\n";
                usd << "}\n";
            }
            usd << "}\n";
        }
        usd << "}\n";
    }
    closePath(usd, openRoot);
    if (!characters.empty())
    {
        const auto content=usd.str();
        populationWait(scene,ovstage_population_add_usd_reference_from_string(scene,st::str(content.c_str()),st::str("/"),&impl->layer));
        impl->hasLayer=true;
        populationWait(scene,ovstage_population_apply_usd_changes(scene,++ordinal));
        st::seal(scene,ordinal); st::hierarchy(scene,ordinal);
    }
    return result;
}
const std::vector<Character> &Module::characters() const { return impl->characters; }
std::vector<BodyTarget> Module::updateTargets(const std::vector<Pose> &poses) const
{
    std::map<std::string,const Pose*> lookup;
    for (const auto &p : poses) {
        require(!lookup.count(p.skeletonPath), "Duplicate skeleton pose.");
        lookup[p.skeletonPath]=&p;
    }
    require(lookup.size()==impl->characters.size(), "Expected one pose for each skeleton.");
    for (const auto &c : impl->characters)
        require(lookup.count(c.skeletonPath) && lookup.at(c.skeletonPath)->bones.size()==c.joints.size(),
                "Pose joint count/order must match init.");
    std::vector<BodyTarget> targets;
    for (const auto &b : impl->bodies) {
        const auto &p=*lookup.at(b.skeleton);
        const auto local=unpacked(p.bones[b.joint])*b.inBone;
        targets.push_back({b.path,packed(local),packed(unpacked(p.characterWorld)*local)});
    }
    return targets;
}
void Module::showColliders(bool visible, ovstage_ordinal_t &ordinal)
{
    if (impl->roots.empty()) return;
    auto *s=impl->stage;
    const auto name=st::str(visible ? "inherited" : "invisible");
    ovx_token_t token{};
    auto *dict=ovstage_get_path_dictionary(s);
    auto status=path_dictionary_create_tokens_from_strings(dict,&name,1,&token);
    if (status.status!=OVX_API_SUCCESS)
    {
        path_dictionary_release_error(dict,status.error);
        throw std::runtime_error("FBIK: cannot create visibility token.");
    }
    ++ordinal;
    for (const auto &path : impl->roots)
    {
        const auto value=st::str(path.c_str());
        ovstage_predicate_t predicate{}; predicate.attribute.string=st::str("usd-path");
        predicate.op=OVSTAGE_FILTER_OP_IN; predicate.values=&value; predicate.value_count=1;
        ovstage_filter_t filter{&predicate,1}; ovstage_query_handle_t query{};
        st::wait(s,ovstage_query(s,&filter,nullptr,0,&query));
        try
        {
            int64_t shape=1; DLTensor tensor{}; tensor.data=&token; tensor.device={kDLCPU,0};
            tensor.ndim=1; tensor.shape=&shape; tensor.dtype={kDLUInt,64,1};
            ovstage_write_data_t payload{}; payload.tensors=&tensor; payload.tensor_count=1; payload.semantic=OVSTAGE_SEMANTIC_TOKEN_ID;
            ovx_string_or_token_t attribute{}; attribute.string=st::str("visibility");
            st::wait(s,ovstage_write_attribute(s,query,attribute,ordinal,payload,OVSTAGE_PRIM_MODE_UPSERT));
        }
        catch (...) { ovstage_release_query(s,query); throw; }
        st::wait(s,ovstage_release_query(s,query));
    }
    st::seal(s,ordinal);
}
} // namespace fbik
