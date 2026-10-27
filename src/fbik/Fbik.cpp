#include "Fbik.h"
#include "MeshTools/Geometry.h"
#include "StageData.h"
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
using Matrix = AffineTransform;
void require(bool condition, const std::string &message)
{
    if (!condition) throw std::runtime_error("FBIK: " + message);
}
const st::Value &get(const st::Prim &prim, const std::string &name)
{
    static const st::Value empty;
    auto it = prim.find(name);
    return it == prim.end() ? empty : it->second;
}
std::string parent(const std::string &path)
{
    auto slash = path.rfind('/');
    return slash == std::string::npos || slash == 0 ? "/" : path.substr(0, slash);
}
const st::Value &inherited(const st::Snapshot &data, std::string path, const std::string &name)
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
Matrix matrix(const std::vector<double> &v, size_t offset = 0)
{
    require(v.size() >= offset + 16, "Missing transform matrix.");
    for (size_t i = offset; i < offset + 16; ++i) require(std::isfinite(v[i]), "Non-finite transform.");
    const auto p = v.data() + offset;
    return Matrix(Mat33(Vec3(float(p[0]),float(p[1]),float(p[2])),
                        Vec3(float(p[4]),float(p[5]),float(p[6])),
                        Vec3(float(p[8]),float(p[9]),float(p[10]))),
                  Vec3(float(p[12]),float(p[13]),float(p[14])));
}
Matrix inverse(const Matrix &m)
{
    require(std::abs(m.A.getDeterminant()) > 1e-18f, "Singular transform.");
    return m.getInverse();
}
Matrix world(const st::Prim &p)
{
    const auto &v = get(p, "omni:fabric:worldMatrix").numbers;
    return v.empty() ? Matrix(Identity) : matrix(v);
}
int integer(double v)
{
    require(std::isfinite(v) && v >= 0 && v <= 2147483647 && std::floor(v) == v, "Invalid skin index or element size.");
    return int(v);
}
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
struct Skeleton
{
    std::string path, character;
    std::vector<std::string> names;
    std::vector<int> parents;
    std::vector<std::vector<int>> children;
    std::vector<Matrix> jointPose, inverseBind;
    Matrix toCharacter{Identity};
    std::vector<std::vector<Vec3>> points;
};
Skeleton readSkeleton(const st::Snapshot &data, const std::string &path, const std::string &character)
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
    const auto &animation = inherited(data, path, "skel:animationSource").paths;
    require(animation.size() <= 1, "Multiple animation sources: " + path);
    if (!animation.empty())
    {
        require(data.count(animation[0]) != 0, "Missing animation source: " + animation[0]);
        const auto &a = data.at(animation[0]);
        const auto &names = get(a, "joints").tokens;
        const auto &ts = get(a,"translations").numbers, &qs = get(a,"rotations").numbers, &ss = get(a,"scales").numbers;
        require((ts.empty() || ts.size()==names.size()*3) && (qs.empty() || qs.size()==names.size()*4) &&
                (ss.empty() || ss.size()==names.size()*3), "Invalid animation transform counts.");
        for (size_t i=0; i<names.size(); ++i)
        {
            if (!indices.count(names[i])) continue;
            auto &m = local[indices.at(names[i])];
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
    for (size_t i=0; i<local.size(); ++i)
        sk.jointPose.push_back(sk.parents[i] < 0 ? local[i] : sk.jointPose[sk.parents[i]]*local[i]);
    sk.toCharacter = inverse(world(data.at(character))) * world(prim);
    return sk;
}
// Flatten indexed primvars by influence group, respecting constant vs vertex interpolation.
std::vector<double> influences(const st::Snapshot &data, const std::string &path, const std::string &name,
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
void collectSkin(const st::Snapshot &data, const std::string &path, Skeleton &sk)
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
            bones.insert(joint);
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
};
Module::Module() : impl(std::make_unique<Impl>()) {}
Module::~Module() = default;
InitResult Module::init(ovstage_instance_t *scene, ovstage_ordinal_t &ordinal)
{
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
            for (const auto &[path,prim] : data)
            {
                if (get(prim,"usd-prim-type").token!="Mesh" || path.rfind(root+"/",0)!=0) continue;
                const auto &binding=inherited(data,path,"skel:skeleton").paths;
                if (binding.size()==1 && binding[0]==sk.path) collectSkin(data,path,sk);
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
        const float minSize=float(0.001/units)/scale; // Minimum full box thickness: 1 mm in world space.
        for (size_t si=0; si<skeletons.size(); ++si)
        {
            auto &sk=skeletons[si];
            usd << "def Scope \"Skeleton_" << si << "\"\n{\n";
            for (size_t j=0; j<sk.names.size(); ++j)
            {
                if (sk.children[j].empty()) { ++result.leaves; continue; }
                const bool box=!sk.points[j].empty();
                Vec3 center(Zero),size(Zero); Quat rotation(Identity); float length=0, radius=0;
                if (box)
                {
                    const auto obb=computeOBBFromPoints(sk.points[j]);
                    center=obb.trans.p; rotation=obb.trans.q; size=obb.halfExtents*2;
                    size.x=std::max(size.x,minSize); size.y=std::max(size.y,minSize); size.z=std::max(size.z,minSize);
                    ++result.boxes;
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
                usd << "def " << (box ? "Cube" : "Capsule") << " \"Joint_" << j << "\" (prepend apiSchemas = [\"PhysicsCollisionAPI\"])\n{\n"
                    << "custom token fbik:joint = " << quote(sk.names[j]) << "\ncustom rel fbik:skeleton = <" << sk.path << ">\n"
                    << "bool physics:collisionEnabled = true\ncolor3f[] primvars:displayColor = " << (box ? "[(0.15,0.65,0.95)]" : "[(0.95,0.65,0.1)]") << "\n";
                pose(usd,center,rotation);
                if (box)
                {
                    usd << "double size = 1\ndouble3 xformOp:scale = "; vector(usd,size);
                    usd << "\nuniform token[] xformOpOrder = [\"xformOp:translate\",\"xformOp:orient\",\"xformOp:scale\"]\n";
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
