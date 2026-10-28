#include "Fbik.h"
#include "MeshTools/ConvexHull.h"
#include <PxShape.h>
#include <map>
#include "AnimationPlayer.h"
#include "Skinning.h"
#include "PhysicsSimulation.h"
#include "SkeletonPose.h"
#include <ovstage/ovstage_config.h>
#include <ovphysx/ovphysx.h>
#include <ovphysx/experimental/OvStageOutput.hpp>
#include <filesystem>
#include <iostream>
#include <cmath>
#include <stdexcept>
#include <algorithm>

namespace st = viewer::stage;
void require(bool value,const char *message) { if (!value) throw std::runtime_error(message); }
void check(ovphysx_result_t r)
{
    if (r.status!=OVPHYSX_API_SUCCESS) { const auto e=ovphysx_get_last_error();throw std::runtime_error(std::string(e.ptr,e.length)); }
}
void checkHull(const MeshTools::ConvexShape &hull,const std::vector<MeshTools::Vec3> &points)
{
    require(hull.volume()>0 && hull.vertices.size()<=255,"Invalid convex hull volume or vertex count");
    std::map<std::pair<unsigned,unsigned>,int> edges;
    for(size_t f=0;f<hull.face_sizes.size();++f) {
        const auto start=hull.face_starts[f],count=hull.face_sizes[f];
        require(count>=3 && start+count<=hull.indices.size(),"Invalid hull face");
        const auto a=hull.vertices.at(hull.indices[start]);
        MeshTools::Vec3 normal(MeshTools::Zero);
        for(size_t j=1;j+1<count;++j)
            normal+=(hull.vertices.at(hull.indices[start+j])-a).cross(hull.vertices.at(hull.indices[start+j+1])-a);
        normal.normalize();
        for(const auto &p:points) {
            if(normal.dot(p-a)>=.002) std::cerr << "Hull containment: points=" << points.size() << " face=" << f << " distance=" << normal.dot(p-a) << "\n";
            require(normal.dot(p-a)<.002,"Hull cut away a source point");
        }
        for(size_t j=0;j<count;++j) {
            const auto u=hull.indices[start+j],v=hull.indices[start+(j+1)%count];
            ++edges[{u,v}];
        }
    }
    for(const auto &[edge,count]:edges) require(count==1 && edges[{edge.second,edge.first}]==1,"Hull is not closed");
}
void testHullRefinement()
{
    using namespace MeshTools;
    const std::vector<Vec3> points={{1,0,0},{-1,0,0},{0,1,0},{0,-1,0},{0,0,1},{0,0,-1}};
    ConvexShape hull;refineConvexHull(points,Vec3(1,1,1),.0005f,hull);
    checkHull(hull,points);
    require(hull.volume()<4 && hull.numFaces()>6,"Support planes did not refine the OBB");
    for(const auto &flat:std::vector<std::vector<Vec3>>{{{0,0,0}},{{-1,0,0},{1,0,0}},{{-1,-1,0},{1,-1,0},{0,1,0}}}) {
        refineConvexHull(flat,Vec3(1,1,.0005f),.0005f,hull);checkHull(hull,flat);
    }
}
struct Scene
{
    ovstage_instance_t *stage=nullptr;
    ovstage_ordinal_t ordinal=1;
    ~Scene() { if(stage) ovstage_destroy_instance(stage); }
    void load(const std::filesystem::path &file)
    {
        ovstage_instance_desc_t desc{};desc.name="fbik-test";
        st::check(ovstage_create_instance(&desc,&stage));
        ovstage_population_desc_t population{};population.domains=OVSTAGE_POPULATION_DOMAIN_ALL;
        ovstage_population_selector_t selector{};
        selector.prim_predicate.kind=OVSTAGE_POPULATION_PRIM_PREDICATE_ALL;
        std::vector<ovx_string_t> properties;
    for (const char *name : {"joints", "bindTransforms", "restTransforms", "translations", "rotations", "scales",
             "points", "skel:skeleton", "skel:animationSource", "skel:joints", "primvars:skel:geomBindTransform",
             "primvars:skel:jointIndices", "primvars:skel:jointWeights", "primvars:skel:jointIndices:indices",
             "primvars:skel:jointWeights:indices", "primvars:skel:skinningMethod", "skel:blendShapeTargets",
             "fbik:joint", "fbik:skeleton"}) properties.push_back(st::str(name));
        selector.property_predicate=ovstage_population_property_predicate_has_name(properties.data(), properties.size());
        const ovx_string_t metadata[]={st::str("interpolation"),st::str("elementSize")};
        selector.property_metadata_paths=metadata;selector.property_metadata_path_count=2;
        population.selectors=&selector;population.selector_count=1;
        const auto units=st::str("metersPerUnit");population.stage_metadata_paths=&units;population.stage_metadata_path_count=1;
        const auto path=file.u8string();
        auto op=ovstage_population_open_usd_from_file_with_desc(stage,st::str(path.c_str()),ordinal,0,&population,1);
        st::check(op.status);ovstage_population_op_wait_result_t result{};
        st::check(ovstage_population_wait_op(stage,op.op_index,OVSTAGE_TIMEOUT_INFINITE,&result));
        require(result.error_op_id_count==0,"Failed to populate fixture");
        st::seal(stage,ordinal);st::hierarchy(stage,ordinal);
    }
};
struct Physics
{
    ovphysx_handle_t handle=OVPHYSX_INVALID_HANDLE;
    ~Physics() { if(handle) {ovphysx_detach_ovstage(handle);ovphysx_destroy_instance(handle);} }
};
void testRagdoll(const std::filesystem::path &root)
{
    namespace sk=viewer::skeleton;
    Scene scene;scene.load(root/"tests/data/ragdoll.usda");
    fbik::Module module;const auto stats=module.init(scene.stage,scene.ordinal);
    require(stats.capsules==4 && stats.joints==2 && stats.degenerate==2,"Ragdoll topology mismatch");
    const auto reference=viewer::sampleAnimation(scene.stage,scene.ordinal,0,module.characters());
    viewer::AnimationPlayer animation;
    animation.init(scene.stage,scene.ordinal,module.characters());
    const auto cached=animation.sample(scene.ordinal,0);
    for(size_t c=0;c<reference.size();++c)
        require(cached.at(c).characterWorld==reference[c].characterWorld && cached[c].bones==reference[c].bones,
                "Cached static/multiple-skeleton pose differs from the reference");
    const auto targets=module.updateTargets(reference);
    const auto setup=st::read(scene.stage,scene.ordinal,{"usd-prim-type","physics:body0","physics:body1",
        "physics:localPos0","physics:localPos1","physics:jointEnabled"});
    const auto checkAnchors=[&](double tolerance) {
        const auto state=st::read(scene.stage,scene.ordinal,{"omni:fabric:worldMatrix"});
        for (const auto &[path,prim]:setup) {
            if(sk::get(prim,"usd-prim-type").token!="PhysicsSphericalJoint") continue;
            const auto &body0=prim.at("physics:body0").paths.at(0),&body1=prim.at("physics:body1").paths.at(0);
            require(body0.find("Joint_0")!=std::string::npos && body1.find("Joint_2")!=std::string::npos,
                    "Joint did not bridge the omitted parent body");
            const auto point=[&](const char *name,const std::string &body) {
                const auto &p=prim.at(name).numbers;
                return sk::world(state.at(body)).transform({float(p[0]),float(p[1]),float(p[2])});
            };
            require((point("physics:localPos0",body0)-point("physics:localPos1",body1)).magnitude()<tolerance,
                    "Joint anchors separated");
        }
    };
    checkAnchors(1e-5);
    viewer::PhysicsSimulation physics(scene.stage,scene.ordinal);
    auto moving=reference;
    for(auto &pose:moving) for(auto &bone:pose.bones) bone[13]+=.1;
    auto shuffledTargets=module.updateTargets(moving);
    std::reverse(shuffledTargets.begin(),shuffledTargets.end());
    physics.setKinematicTargets(shuffledTargets,scene.ordinal);
    physics.step(scene.ordinal);
    const auto moved=st::read(scene.stage,scene.ordinal,{"omni:fabric:worldMatrix"});
    for (const auto &target:shuffledTargets) for(size_t k=0;k<16;++k)
        require(std::abs(moved.at(target.path).at("omni:fabric:worldMatrix").numbers[k]-target.world[k])<.001,
                "Batched targets were assigned to the wrong bodies");
    physics.setKinematicTargets(targets,scene.ordinal);
    physics.step(scene.ordinal);
    for (int run=0;run<2;++run) {
        module.setMode(fbik::Mode::Ragdoll,scene.ordinal);
        physics.applyControlChanges(scene.ordinal);
        const auto modeOrdinal=scene.ordinal;
        module.setMode(fbik::Mode::Ragdoll,scene.ordinal);
        require(scene.ordinal==modeOrdinal,"Repeated mode should not write or reset velocities");
        const auto before=module.readPoses(reference,scene.ordinal);
        for(size_t i=0;i<before.size();++i) for(size_t j=0;j<before[i].bones.size();++j)
            for(int k=0;k<16;++k) require(std::abs(before[i].bones[j][k]-reference[i].bones[j][k])<1e-4,
                                          "Entering ragdoll changed the pose");
        for(int frame=0;frame<180;++frame) {
            physics.step(scene.ordinal);
            checkAnchors(.03);
            if(frame==0) {
                const auto poses=module.readPoses(reference,scene.ordinal);
                const double drop=reference[0].bones[0][13]-poses[0].bones[0][13];
                require(drop>0 && drop<.01,"Release retained kinematic velocity or failed to wake bodies");
            }
            if(frame==29) {
                const auto state=st::read(scene.stage,scene.ordinal,{"omni:fabric:worldMatrix"});
                const auto &m=state.at(targets[0].path).at("omni:fabric:worldMatrix").numbers;
                viewer::Vec3 origin(float(m[12]),float(m[13]),float(m[14]+6));
                require(physics.beginDrag(origin,{0,0,-1},scene.ordinal),"Dynamic character could not be picked");
                physics.moveDrag(origin+viewer::Vec3(0,.5f,0),{0,0,-1},scene.ordinal);
                const auto flags=st::read(scene.stage,scene.ordinal,{"physics:kinematicEnabled","physics:jointEnabled"});
                for(const auto &t:targets)
                    require(flags.at(t.path).at("physics:kinematicEnabled").numbers[0]==0,"Dragging reset ragdoll mode");
                for(const auto &[path,prim]:setup) if(sk::get(prim,"usd-prim-type").token=="PhysicsSphericalJoint")
                    require(flags.at(path).at("physics:jointEnabled").numbers[0]==1,"Dragging disabled a bone joint");
            }
            if(frame==49) physics.endDrag(scene.ordinal);
            if(frame==14) {
                const auto poses=module.readPoses(reference,scene.ordinal);
                require(poses[0].bones[0][13]<reference[0].bones[0][13]-.1,"Ragdoll did not fall under gravity");
                for(size_t j=0;j<poses[0].bones.size();++j) for(int k=0;k<16;++k)
                    require(std::abs(poses[0].bones[j][k]-poses[1].bones[j][k])<.001,
                            "Overlapping bone colliders pushed each other apart");
            }
        }
        const auto poses=module.readPoses(reference,scene.ordinal);
        for(const auto &pose:poses) {
            const auto expectedGap=sk::unpacked(pose.bones[0])*sk::inverse(sk::unpacked(reference[0].bones[0]))*
                                   sk::unpacked(reference[0].bones[1]);
            const auto expectedLeaf=sk::unpacked(pose.bones[2])*sk::inverse(sk::unpacked(reference[0].bones[2]))*
                                    sk::unpacked(reference[0].bones[3]);
            for(int k=0;k<16;++k) {
                require(std::abs(sk::packed(expectedGap)[k]-pose.bones[1][k])<1e-4,"Bodyless parent did not follow ancestor");
                require(std::abs(sk::packed(expectedLeaf)[k]-pose.bones[3][k])<1e-4,"Leaf did not follow simulated parent");
            }
        }
        const auto state=st::read(scene.stage,scene.ordinal,{"omni:fabric:worldMatrix"});
        for(const auto &t:targets) {
            const auto &m=state.at(t.path).at("omni:fabric:worldMatrix").numbers;
            require(m[13]>.05 && m[13]<2,"Ragdoll failed to settle on the environment");
        }
        module.setMode(fbik::Mode::Follow,scene.ordinal);
        physics.applyControlChanges(scene.ordinal);
        physics.setKinematicTargets(targets,scene.ordinal);
        for(int i=0;i<4;++i) physics.step(scene.ordinal);
        const auto restored=module.readPoses(reference,scene.ordinal);
        for(size_t i=0;i<restored.size();++i) for(size_t j=0;j<restored[i].bones.size();++j)
            for(int k=0;k<16;++k) require(std::abs(restored[i].bones[j][k]-reference[i].bones[j][k])<.001,
                                          "Returning to Follow failed to restore/hold targets");
    }
    physics.stop(scene.ordinal);
    module.setMode(fbik::Mode::Ragdoll,scene.ordinal);
    const auto repeat=module.init(scene.stage,scene.ordinal);
    require(module.mode()==fbik::Mode::Follow && repeat.joints==stats.joints,"Reinit did not reset mode and joints");
    std::cout << "Ragdoll transitions, joint anchors, skipped bones, collision filtering and floor contact passed.\n";
}
int main(int argc,char **argv)
{
    int code=0;bool initialized=false;
    try
    {
        testHullRefinement();
        require(argc==2,"Expected repository root");
        const auto root=std::filesystem::absolute(argv[1]);
        const auto binaries=(root/"deps/ovstage/bin").u8string();
        auto entry=ovstage_config_entry_binary_package_root_path(st::str(binaries.c_str()));
        ovstage_config_t config{&entry,1};st::check(ovstage_initialize(&config));initialized=true;
        viewer::PhysicsRuntime runtime;
        testRagdoll(root);
        {
            Scene scene;scene.load(root/"tests/data/characters.usda");
            fbik::Module module;
            auto stats=module.init(scene.stage,scene.ordinal);
            require(stats.characters==2 && stats.skeletons==3,"Multiple characters/skeletons were not discovered");
            require(stats.convexMeshes==1 && stats.capsules==3 && stats.leaves==6 && stats.degenerate==1,"Wrong OBB/capsule/leaf counts");
            auto first=st::read(scene.stage,scene.ordinal,{"usd-prim-type","omni:fabric:worldMatrix","fbik:joint","height","radius","physics:kinematicEnabled"});
            const std::string a="/World/A/__FbikBoneColliders/Skeleton_0/";
            require(first.at(a+"Joint_0").at("usd-prim-type").token=="Mesh","Skin collider was not a convex mesh");
            require(first.at(a+"Joint_1").at("usd-prim-type").token=="Capsule","Non-leaf with End in its name was skipped");
            require(!first.count(a+"Joint_2") && !first.count(a+"Joint_3") && !first.count(a+"Joint_4") && !first.count(a+"Joint_5"),"Leaf or degenerate segment has collider");
            const auto &capsule=first.at(a+"Joint_1");
            require(std::abs(capsule.at("height").numbers[0]-2)<1e-5 && std::abs(capsule.at("radius").numbers[0]-.2)<1e-5,"Fallback capsule geometry mismatch");
            require(std::abs(capsule.at("omni:fabric:worldMatrix").numbers[13]-4)<1e-5,"Character scale was not respected");
            const auto &branch=first.at("/World/B/__FbikBoneColliders/Skeleton_0/Joint_0");
            require(std::abs(branch.at("height").numbers[0]-2)<1e-5 &&
                    std::abs(branch.at("omni:fabric:worldMatrix").numbers[13]-1)<1e-5,
                    "Capsule does not point to the average child position");
            const auto &box=first.at(a+"Joint_0").at("omni:fabric:worldMatrix").numbers;
            require(std::abs(box[12]-3.7)<.001,"Skin remap, geomBind, multi-mesh union or character transform incorrect");
            for (const auto &[path,prim] : first)
                if(path.find("__FbikBoneColliders")!=std::string::npos)
                {
                    auto it=prim.find("physics:kinematicEnabled");
                    if (prim.count("fbik:joint")) require(it!=prim.end() && !it->second.numbers.empty() && it->second.numbers[0]==1,"Bone is not kinematic");
                }
            auto repeat=module.init(scene.stage,scene.ordinal);
            auto second=st::read(scene.stage,scene.ordinal,{"usd-prim-type","omni:fabric:worldMatrix","fbik:joint","height","radius","physics:kinematicEnabled"});
            require(repeat.convexMeshes==stats.convexMeshes && repeat.capsules==stats.capsules && second.size()==first.size(),"init created duplicates");
            module.showColliders(true,scene.ordinal);
            module.showColliders(false,scene.ordinal);
            Physics physics;ovphysx_create_args args=OVPHYSX_CREATE_ARGS_DEFAULT;
            check(ovphysx_create_instance(&args,&physics.handle));check(ovphysx_attach_ovstage(physics.handle,scene.stage,scene.ordinal));
            for(int i=0;i<60;++i) check(ovphysx_step_sync(physics.handle,1.0f/60));
            const float from[]={3.4f,0,3},direction[]={0,0,-1};
            const ovphysx_scene_query_hit_t *hits=nullptr;uint32_t count=0;
            check(ovphysx_raycast(physics.handle,from,direction,10,false,OVPHYSX_SCENE_QUERY_MODE_CLOSEST,&hits,&count));
            require(count==1,"Generated hidden hull has no PhysX shape");
            ovphysx_string_t path{};const auto collider=hits[0].collision;
            check(ovphysx_scene_query_get_paths_from_ids(physics.handle,&collider,1,&path,1,&count));
            require(std::string(path.ptr,path.length).find("__FbikBoneColliders")!=std::string::npos,"Ray missed generated collider");
            void *shape=nullptr;
            check(ovphysx_get_physx_ptr(physics.handle,path,OVPHYSX_PHYSX_TYPE_SHAPE,&shape));
            require(shape && static_cast<physx::PxShape*>(shape)->getGeometry().getType()==physx::PxGeometryType::eCONVEXMESH,
                    "PhysX did not cook a convex mesh body shape");
            auto output=ovphysx::utils::writeWorldTransformsToOvstage(physics.handle,scene.stage,++scene.ordinal);
            require(output.ok(),"Could not publish probe position");st::seal(scene.stage,scene.ordinal);
            auto state=st::read(scene.stage,scene.ordinal,{"omni:fabric:worldMatrix"});
            const auto y=state.at("/World/Probe").at("omni:fabric:worldMatrix").numbers[13];
            std::cout << "Probe contacting generated collider at y=" << y << '\n';
            require(y>.3 && y<2,"Dynamic probe did not collide with kinematic bone hull");
        }
        {
            Scene scene;scene.load(root/"resources/scenes/basic_walk.usda");
            fbik::Module module;auto stats=module.init(scene.stage,scene.ordinal);
            require(stats.characters==1 && stats.skeletons==1 && stats.convexMeshes==61 && stats.leaves==16 && stats.joints==60,
                    "SOMA collider/joint count mismatch");
            viewer::AnimationPlayer animation;
            animation.init(scene.stage,scene.ordinal,module.characters());
            for (double time:{0.0, .017, .5, 4.123, 7.9, 0.0}) {
                const auto expected=viewer::sampleAnimation(scene.stage,scene.ordinal,time,module.characters());
                const auto actual=animation.sample(scene.ordinal,time);
                require(actual.size()==expected.size(),"Cached animation lost a skeleton");
                for(size_t c=0;c<actual.size();++c) {
                    require(actual[c].characterWorld==expected[c].characterWorld,"Cached animation changed character placement");
                    require(actual[c].bones==expected[c].bones,"Cached animation changed the sampled pose");
                }
            }
            const auto data=st::read(scene.stage,scene.ordinal,{"omni:fabric:worldMatrix","fbik:joint"});
            for(const auto &[path,prim]:data)
                if(prim.count("fbik:joint"))
                {
                    const auto &m=prim.at("omni:fabric:worldMatrix").numbers;
                    require(m.size()==16 && m[12]>0 && m[12]<4 && m[13]>-1 && m[13]<3,"SOMA collider outside expected character bounds");
                }
            {
                Physics physics;ovphysx_create_args args=OVPHYSX_CREATE_ARGS_DEFAULT;
                check(ovphysx_create_instance(&args,&physics.handle));
                check(ovphysx_attach_ovstage(physics.handle,scene.stage,scene.ordinal));
                check(ovphysx_step_sync(physics.handle,1.0f/60));
                size_t cooked=0;
                for(const auto &[path,prim]:data) if(prim.count("fbik:joint")) {
                    void *shape=nullptr;ovphysx_string_t name{path.c_str(),path.size()};
                    check(ovphysx_get_physx_ptr(physics.handle,name,OVPHYSX_PHYSX_TYPE_SHAPE,&shape));
                    require(shape && static_cast<physx::PxShape*>(shape)->getGeometry().getType()==physx::PxGeometryType::eCONVEXMESH,
                            "SOMA bone is missing a cooked convex shape");
                    ++cooked;
                }
                require(cooked==61,"Not all SOMA convex meshes were cooked");
            }
            const auto initial=st::read(scene.stage,scene.ordinal,{"omni:fabric:worldMatrix"});
            viewer::Skinning skinning;skinning.init(scene.stage,scene.ordinal,module.characters());
            skinning.update(viewer::sampleAnimation(scene.stage,scene.ordinal,0,module.characters()),scene.ordinal);
            const std::string mesh="/World/Character/Soma/c_geometry_grp/MainMesh/Meshes/c_skin_mid";
            const auto initialPoints=st::read(scene.stage,scene.ordinal,{"points"}).at(mesh).at("points").numbers;
            double previousX=0;
            for (int run=0;run<2;++run) {
                viewer::PhysicsSimulation physics(scene.stage,scene.ordinal);
                for (int frame=1;frame<=420;++frame) {
                    auto poses=viewer::sampleAnimation(scene.stage,scene.ordinal,frame/60.0,module.characters());
                    auto targets=module.updateTargets(poses);
                    physics.setKinematicTargets(targets,scene.ordinal);
                    physics.step(scene.ordinal);
                    if (frame==180) {
                        skinning.update(poses,scene.ordinal);
                        const auto points=st::read(scene.stage,scene.ordinal,{"points"}).at(mesh).at("points").numbers;
                        double difference=0;for(size_t i=0;i<points.size();++i) difference=std::max(difference,std::abs(points[i]-initialPoints[i]));
                        require(difference>5,"Walk did not deform skin vertices in mesh space");
                        const auto state=st::read(scene.stage,scene.ordinal,{"omni:fabric:worldMatrix"});
                        for (const auto &target:targets) {
                            const auto &actual=state.at(target.path).at("omni:fabric:worldMatrix").numbers;
                            for (int axis=0;axis<16;++axis)
                                require(std::abs(actual[axis]-target.world[axis])<.002,"Bone did not reach kinematic target");
                        }
                    }
                }
                auto state=st::read(scene.stage,scene.ordinal,{"omni:fabric:worldMatrix"});
                const double x=state.at("/World/Box").at("omni:fabric:worldMatrix").numbers[12];
                std::cout << "Walk run " << run << ": box x=" << x << '\n';
                require(x<-.8,"Walking character did not push the cube away");
                if(run) require(std::abs(x-previousX)<.01,"Animation restart is not repeatable");
                previousX=x;
                physics.stop(scene.ordinal);
                skinning.update(viewer::sampleAnimation(scene.stage,scene.ordinal,0,module.characters()),scene.ordinal);
                const auto restoredPoints=st::read(scene.stage,scene.ordinal,{"points"}).at(mesh).at("points").numbers;
                for(size_t i=0;i<initialPoints.size();++i)
                    require(std::abs(initialPoints[i]-restoredPoints[i])<.001,"Stop did not restore skin");
                state=st::read(scene.stage,scene.ordinal,{"omni:fabric:worldMatrix"});
                for (const auto &path : {"/World/Box", "/World/Character"}) {
                    const auto &a=initial.at(path).at("omni:fabric:worldMatrix").numbers;
                    const auto &b=state.at(path).at("omni:fabric:worldMatrix").numbers;
                    for (size_t i=0;i<a.size();++i) require(std::abs(a[i]-b[i])<1e-6,"Animation stop did not reset scene");
                }
            }
            {
                const auto reference=viewer::sampleAnimation(scene.stage,scene.ordinal,0,module.characters());
                viewer::PhysicsSimulation physics(scene.stage,scene.ordinal);
                physics.setKinematicTargets(module.updateTargets(reference),scene.ordinal);
                physics.step(scene.ordinal);
                module.setMode(fbik::Mode::Ragdoll,scene.ordinal);
                physics.applyControlChanges(scene.ordinal);
                for(int frame=0;frame<180;++frame) physics.step(scene.ordinal);
                const auto fallen=module.readPoses(reference,scene.ordinal);
                skinning.update(fallen,scene.ordinal);
                const auto points=st::read(scene.stage,scene.ordinal,{"points"}).at(mesh).at("points").numbers;
                double difference=0;
                for(size_t i=0;i<points.size();++i) {
                    require(std::isfinite(points[i]),"Ragdoll skin contains non-finite vertices");
                    difference=std::max(difference,std::abs(points[i]-initialPoints[i]));
                }
                require(difference>20,"Ragdoll did not deform the visible skin");
                const auto state=st::read(scene.stage,scene.ordinal,{"usd-prim-type","omni:fabric:worldMatrix",
                    "physics:body0","physics:body1","physics:localPos0","physics:localPos1"});
                namespace sk=viewer::skeleton;
                double maxError=0;
                for(const auto &[path,prim]:state) if(sk::get(prim,"usd-prim-type").token=="PhysicsSphericalJoint") {
                    const auto point=[&](const char *body,const char *position) {
                        const auto &p=prim.at(position).numbers;
                        return sk::world(state.at(prim.at(body).paths[0])).transform({float(p[0]),float(p[1]),float(p[2])});
                    };
                    maxError=std::max(maxError,double((point("physics:body0","physics:localPos0")-
                                                     point("physics:body1","physics:localPos1")).magnitude()));
                }
                std::cout << "SOMA ragdoll maximum attachment error: " << maxError << " m\n";
                require(maxError<.05,"SOMA ragdoll joints separated");
                physics.stop(scene.ordinal);
            }
            std::cout << "SOMA: " << stats.convexMeshes << " convex meshes, " << stats.joints << " joints, " << stats.leaves << " leaves omitted\n";
        }
        std::cout << "FBIK discovery, bindings, OBBs, fallback capsules, kinematic collision, animation, skinning and repeated init passed.\n";
    }
    catch(const std::exception &e) { std::cerr << e.what() << '\n';code=1; }
    if(initialized) ovstage_shutdown();
    return code;
}
