#include "Skinning.h"
#include "SkeletonPose.h"
#include <ovx/path_dictionary/path_dictionary_utils.h>
#include <algorithm>
namespace viewer {
namespace {
using namespace skeleton;
void write(ovstage_instance_t *s, ovstage_query_handle_t query, ovstage_ordinal_t ordinal,
           const char *name,void *data,DLDataType type,ovstage_attribute_semantic_t semantic,
           bool array=false,int64_t count=1)
{
    DLTensor t{};t.data=data;t.device={kDLCPU,0};t.ndim=1;t.shape=&count;t.dtype=type;
    ovstage_write_data_t payload{};payload.tensors=&t;payload.tensor_count=1;
    payload.semantic=semantic;payload.is_array=array;
    ovx_string_or_token_t attribute{};attribute.string=stage::str(name);
    stage::wait(s,ovstage_write_attribute(s,query,attribute,ordinal,payload,OVSTAGE_PRIM_MODE_UPSERT));
}
}
struct Skinning::Impl {
    ovstage_instance_t *stage=nullptr;
    ovx_token_t vertexToken=0;
    struct Mesh {
        std::string path,skeleton;
        ovstage_query_handle_t query{};
        size_t influences=0;
        bool leftHanded=false;
        std::vector<Vec3> points;
        std::vector<size_t> indices,triangles;
        std::vector<double> weights;
        std::vector<Matrix> bind; // inverse joint bind * geometry bind
    };
    std::vector<Mesh> meshes;
};
Skinning::Skinning():impl(std::make_unique<Impl>()) {}
// Queries are stage-owned and released when the scene is destroyed.
Skinning::~Skinning()=default;
void Skinning::init(ovstage_instance_t *s,ovstage_ordinal_t ordinal,const std::vector<fbik::Character> &characters)
{
    require(!impl->stage,"Skinning must be initialized once per scene.");impl->stage=s;
    auto *dict=ovstage_get_path_dictionary(s);const auto token=stage::str("vertex");
    const auto status=path_dictionary_create_tokens_from_strings(dict,&token,1,&impl->vertexToken);
    if(status.status!=OVX_API_SUCCESS) {
        path_dictionary_release_error(dict,status.error);throw std::runtime_error("Cannot create normals token.");
    }
    const auto data=stage::read(s,ordinal,{"usd-prim-type","points","faceVertexCounts","faceVertexIndices","orientation",
        "skel:skeleton","skel:joints","bindTransforms","primvars:skel:geomBindTransform",
        "primvars:skel:jointIndices","primvars:skel:jointWeights","primvars:skel:jointIndices:indices","primvars:skel:jointWeights:indices",
        "primvars:skel:jointIndices:usd-metadata:elementSize","primvars:skel:jointWeights:usd-metadata:elementSize",
        "primvars:skel:jointIndices:usd-metadata:interpolation","primvars:skel:jointWeights:usd-metadata:interpolation"});
    for(const auto &c:characters) for(const auto &[path,prim]:data) {
        if(get(prim,"usd-prim-type").token!="Mesh" || path.rfind(c.characterPath+"/",0)!=0) continue;
        const auto &binding=inherited(data,path,"skel:skeleton").paths;
        if(binding.size()!=1 || binding[0]!=c.skeletonPath || inherited(data,path,"primvars:skel:jointWeights").numbers.empty()) continue;
        Impl::Mesh mesh;mesh.path=path;mesh.skeleton=c.skeletonPath;
        const auto &points=get(prim,"points").numbers;
        for(size_t i=0;i<points.size();i+=3) mesh.points.emplace_back(float(points[i]),float(points[i+1]),float(points[i+2]));
        if(mesh.points.empty()) continue;
        size_t n=0;
        auto indices=influences(data,path,"primvars:skel:jointIndices",mesh.points.size(),mesh.influences);
        mesh.weights=influences(data,path,"primvars:skel:jointWeights",mesh.points.size(),n);
        require(n==mesh.influences,"Skin weight/index mismatch.");
        auto order=inherited(data,path,"skel:joints").tokens;if(order.empty()) order=c.joints;
        for(size_t i=0;i<indices.size();++i) {
            if(mesh.weights[i]==0) {mesh.indices.push_back(0);continue;}
            const auto index=size_t(integer(indices[i]));require(index<order.size(),"Invalid skin joint index.");
            const auto found=std::find(c.joints.begin(),c.joints.end(),order[index]);
            require(found!=c.joints.end(),"Unknown skin joint.");mesh.indices.push_back(found-c.joints.begin());
        }
        const auto &bind=get(data.at(c.skeletonPath),"bindTransforms").numbers;
        const auto &geom=inherited(data,path,"primvars:skel:geomBindTransform").numbers;
        const auto geometry=geom.empty()?Matrix(Identity):matrix(geom);
        for(size_t j=0;j<c.joints.size();++j) mesh.bind.push_back(inverse(matrix(bind,j*16))*geometry);
        const auto &counts=get(prim,"faceVertexCounts").numbers,&faces=get(prim,"faceVertexIndices").numbers;
        size_t offset=0;
        for(double count:counts) {
            const auto n=size_t(integer(count));require(offset+n<=faces.size(),"Invalid mesh topology.");
            for(size_t k=0;k<n;++k) require(size_t(integer(faces[offset+k]))<mesh.points.size(),"Invalid mesh vertex.");
            for(size_t k=1;k+1<n;++k) for(size_t corner:{size_t(0),k,k+1}) mesh.triangles.push_back(size_t(faces[offset+corner]));
            offset+=n;
        }
        mesh.leftHanded=get(prim,"orientation").token=="leftHanded";
        const auto value=stage::str(path.c_str());ovstage_predicate_t predicate{};
        predicate.attribute.string=stage::str("usd-path");predicate.op=OVSTAGE_FILTER_OP_IN;
        predicate.values=&value;predicate.value_count=1;ovstage_filter_t filter{&predicate,1};
        stage::wait(s,ovstage_query(s,&filter,nullptr,0,&mesh.query));
        impl->meshes.push_back(std::move(mesh));
    }
}
void Skinning::update(const std::vector<fbik::Pose> &poses,ovstage_ordinal_t &ordinal)
{
    if(impl->meshes.empty()) return;
    const auto data=stage::read(impl->stage,ordinal,{"omni:fabric:worldMatrix"});
    ++ordinal;
    for(const auto &mesh:impl->meshes) {
        auto pose=std::find_if(poses.begin(),poses.end(),[&](const auto &p){return p.skeletonPath==mesh.skeleton;});
        require(pose!=poses.end(),"Missing skin pose.");
        const auto toMesh=inverse(world(data.at(mesh.path)))*unpacked(pose->characterWorld);
        std::vector<Matrix> transforms;
        for(size_t j=0;j<mesh.bind.size();++j) transforms.push_back(toMesh*unpacked(pose->bones[j])*mesh.bind[j]);
        std::vector<Vec3> points(mesh.points.size(),Vec3(Zero)),normals(mesh.points.size(),Vec3(Zero));
        for(size_t v=0;v<points.size();++v) {
            double total=0;
            for(size_t j=0;j<mesh.influences;++j) {
                const auto k=v*mesh.influences+j;
                points[v]+=transforms[mesh.indices[k]].transform(mesh.points[v])*float(mesh.weights[k]);total+=mesh.weights[k];
            }
            if(total==0) points[v]=mesh.points[v];
        }
        for(size_t t=0;t<mesh.triangles.size();t+=3) {
            const auto a=mesh.triangles[t],b=mesh.triangles[t+1],c=mesh.triangles[t+2];
            auto n=(points[b]-points[a]).cross(points[c]-points[a]);if(mesh.leftHanded)n=-n;
            normals[a]+=n;normals[b]+=n;normals[c]+=n;
        }
        Vec3 low(INFINITY,INFINITY,INFINITY),high(-INFINITY,-INFINITY,-INFINITY);
        for(size_t i=0;i<points.size();++i) {
            if(normals[i].magnitudeSquared()>1e-16f)normals[i].normalize();else normals[i]=Vec3(0,1,0);
            for(int a=0;a<3;++a){low[a]=std::min(low[a],points[i][a]);high[a]=std::max(high[a],points[i][a]);}
        }
        write(impl->stage,mesh.query,ordinal,"normals:usd-metadata:interpolation",&impl->vertexToken,{kDLUInt,64,1},OVSTAGE_SEMANTIC_TOKEN_ID);
        write(impl->stage,mesh.query,ordinal,"points",points.data(),{kDLFloat,32,3},OVSTAGE_SEMANTIC_POINT,true,points.size());
        write(impl->stage,mesh.query,ordinal,"normals",normals.data(),{kDLFloat,32,3},OVSTAGE_SEMANTIC_NORMAL,true,normals.size());
        double extent[]={low.x,low.y,low.z,high.x,high.y,high.z};
        write(impl->stage,mesh.query,ordinal,"extent",extent,{kDLFloat,64,6},OVSTAGE_SEMANTIC_NONE);
    }
    stage::seal(impl->stage,ordinal);
}
}

