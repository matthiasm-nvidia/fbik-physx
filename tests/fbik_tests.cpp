#include "Fbik.h"
#include "JointSetup.h"
#include "CollisionFiltering.h"
#include <PxShape.h>
#include <PxRigidDynamic.h>
#include <geometry/PxCapsuleGeometry.h>
#include <extensions/PxD6Joint.h>
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
void testCapsuleOverlap()
{
    using namespace MeshTools;
    using namespace fbik::collision;
    const Capsule horizontal={{-2,0,0},{2,0,0},.1f};
    struct Case { Capsule other; bool overlap; };
    const std::vector<Case> cases={
        {{{-1,.2f,0},{3,.2f,0},.1f},true}, // Parallel, touching along the sides.
        {{{3,.21f,0},{-1,.21f,0},.1f},false}, // Reversed endpoints, separated sides.
        {{{2.2f,0,0},{4,0,0},.1f},true}, // Collinear end caps touch.
        {{{2.21f,0,0},{4,0,0},.1f},false},
        {{{0,-1,.19f},{0,1,.19f},.1f},true}, // Closest points inside both segments.
        {{{0,-1,.21f},{0,1,.21f},.1f},false},
        {{{2.1f,.1f,0},{3,1,0},.1f},true}, // Closest points at both endpoints.
        {{{2.2f,.2f,0},{3,1,0},.1f},false},
        {{{0,.15f,0},{0,.15f,0},.1f},true}, // Capsule versus sphere.
        {{{2.3f,0,0},{2.3f,0,0},.1f},false},
        {{{-2,.21f,0},{2,.210001f,0},.1f},false}}; // Nearly parallel.
    const AffineTransform transform(Mat33(Quat(.73f,Vec3(1,2,3).normalized()))*3,Vec3(4,5,6));
    const auto moved=[&](const Capsule &c) {
        return Capsule{transform.transform(c.p0),transform.transform(c.p1),3*c.radius};
    };
    for (const auto &c:cases) {
        require(overlaps(horizontal,c.other)==c.overlap && overlaps(c.other,horizontal)==c.overlap,
                "Capsule segment overlap or symmetry is incorrect");
        require(overlaps(moved(horizontal),moved(c.other))==c.overlap,
                "Capsule overlap changed under common rotation, translation and scale");
    }
    const Capsule sphere={{0,0,0},{0,0,0},.5f};
    require(overlaps(sphere,{{1,0,0},{1,0,0},.5f}),"Touching spheres should overlap");
    require(!overlaps(sphere,{{.8f,.8f,0},{.8f,.8f,0},.5f}),"Sphere test used overlapping bounds");
    const auto capsule=restCapsule(transform,4,.1f);
    require((capsule.p0-moved(horizontal).p0).magnitude()<1e-5 &&
            (capsule.p1-moved(horizontal).p1).magnitude()<1e-5 && std::abs(capsule.radius-.3f)<1e-6,
            "Rest-pose capsule lost orientation, translation or scale");
    const AffineTransform tiny(Mat33(Identity)*.0001f,Vec3(Zero));
    const auto a=restCapsule(tiny,4,.1f),b=restCapsule(tiny*AffineTransform(Mat33(Identity),Vec3(0,.21f,0)),4,.1f);
    require(!overlaps(a,b),"Small separated capsules were filtered");
}
void testJointFrames()
{
    namespace sk=viewer::skeleton;
    using namespace MeshTools;
    sk::Skeleton rig;
    rig.names={"Root","Root/LeftFoot","Root/LeftFoot/LeftToeBase","Root/RightFoot","Root/RightFoot/RightToeBase","Root/Head"};
    rig.parents={-1,0,1,0,3,0};rig.children={{1,3,5},{2},{},{4},{},{}};
    std::vector<sk::Matrix> rest;
    for (const Vec3 p:std::vector<Vec3>{{0,0,2},{-.3f,0,0},{-.4f,.5f,.05f},{.3f,0,0},{.4f,.5f,.05f},{0,0,2}})
        rest.emplace_back(Mat33(Identity),p);
    // Wide T-poses must retain up even when yaw puts arm span on global Y/Z.
    for (const float width:{.8f,2.f}) for (const float yaw:{0.f,Pi/2,Pi,3*Pi/2}) for (const bool yUp:{false,true}) {
        const std::vector<Vec3> skin={{-width,-.2f,0},{width,.5f,2}};
        const Mat33 rotation=Mat33(Quat(yUp ? -Pi/2 : 0,Vec3(1,0,0)))*Mat33(Quat(yaw,Vec3(0,0,1)));
        const sk::Matrix world(rotation*2,Vec3(3,4,5));
        const auto frame=fbik::joints::characterFrame(rig,rest,skin,world);
        require((frame.up-Vec3(0,0,1)).magnitude()<1e-5,"Y/Z-up frame inference failed");
        require((frame.forward-Vec3(0,1,0)).magnitude()<1e-5,"Foot directions did not resolve facing/sign");
        require((frame.right-Vec3(1,0,0)).magnitude()<1e-5,"Character frame is not right handed");
    }
    const auto wide=fbik::joints::characterFrame(rig,rest,{{-2,-.2f,0},{2,.5f,2}},sk::Matrix(Identity));
    require((wide.up-Vec3(0,0,1)).magnitude()<1e-5,"T-pose arm span was mistaken for up");
    // Joint frames must retain the measured initial bend and fall back stably near straight.
    sk::Skeleton limb;limb.parents={-1,0,1};limb.children={{1},{2},{}};
    const fbik::joints::CharacterFrame frame={{1,0,0},{0,1,0},{0,0,1}};
    for (const bool knee:{false,true}) for (const int side:{-1,1}) for (float bend:{0.f,1.f,25.f}) {
        limb.names={"root",knee ? (side<0 ? "root/LeftShin" : "root/RightShin") :
                                             (side<0 ? "root/LeftForeArm" : "root/RightForeArm"),"root/tip"};
        const Vec3 upper=knee ? Vec3(0,0,-1) : Vec3(float(side),0,0);
        const Vec3 axis=knee ? frame.right : frame.up;
        const float angle=(knee ? -1 : side)*bend*Pi/180;
        const Vec3 lower=Mat33(Quat(angle,axis))*upper;
        const std::vector<sk::Matrix> bones={{Mat33(Identity),Vec3(Zero)},{Mat33(Identity),upper},{Mat33(Identity),upper+lower}};
        const auto setup=fbik::joints::setup(limb,bones,frame,1);
        require(setup.limits.hinge,"Knee/elbow not recognized as hinge");
        require((setup.parentFrame.column0-axis).magnitude()<1e-5,"Hinge sign or straight-pose fallback is wrong");
        const Quat relative(setup.parentFrame.getTranspose()*setup.childFrame);
        require(std::abs(2*std::atan2(relative.x,relative.w)-angle)<1e-5,"Initial hinge bend was reset to zero");
        require(setup.limits.lower==(knee ? -60 : (side<0 ? -120 : 0)) &&
                setup.limits.upper==(knee || side<0 ? 0 : 120),"Hinge limits have the wrong sign");
        if (knee) {
            const auto flexed=Mat33(Quat(setup.limits.lower*Pi/180,setup.parentFrame.column0))*upper;
            const auto hyperextended=Mat33(Quat(20*Pi/180,setup.parentFrame.column0))*upper;
            require(flexed.dot(frame.forward)<-.5f && hyperextended.dot(frame.forward)>.3f,
                    "Knee allows forward hyperextension instead of backward flexion");
        }
    }
    // A bent, twisted upper arm must use its measured plane instead of global up.
    limb.names={"root","root/RightForeArm","root/RightForeArm/hand"};
    const Vec3 axis=Mat33(Quat(.6f,Vec3(1,0,0)))*frame.up;
    const Vec3 upper(1,0,0),lower=Mat33(Quat(.4f,axis))*upper;
    const std::vector<sk::Matrix> bent={{Mat33(Identity),Vec3(Zero)},{Mat33(Identity),upper},{Mat33(Identity),upper+lower}};
    require((fbik::joints::setup(limb,bent,frame,1).parentFrame.column0-axis).magnitude()<1e-5,"Measured elbow bend plane was ignored");
    struct BallCase { const char *parent,*child; int twist,swing; };
    for (const auto &c:std::vector<BallCase>{
            {"Chest","LeftShoulder",5,5},{"Chest","Clavicle",5,5},{"LeftShoulder","LeftArm",60,100},
            {"Chest","RIGHT_UPPER_ARM",60,100},{"LeftArm","LeftArmTwist",5,5},{"LeftArm","LeftUpperArmHelper",5,5},
            {"Spine1","Spine2",5,5},{"Spine2","Chest",5,5},{"Chest","Torso",5,5},{"Chest","Neck",5,5},
            {"Neck","hEaD",20,30},{"Head","EyeHelper",5,5},{"LeftForeArm","LeftHand",5,5},
            {"LeftShin","LeftFoot",5,5},{"LeftFoot","LeftToeBase",5,5},{"Root","RightAnkle",5,5},
            {"Hips","LeftLeg",30,60},{"Hips","RIGHT_UP_LEG",30,60},{"LeftLeg","LeftUpperLegHelper",5,5},
            {"ArmAncestor/Chest","Internal",5,5},{"ArmAncestor/Chest","LeftArm",60,100},
            {"LegAncestor/Hips","RightLeg",30,60},{"LegAncestor/Hips","PelvisHelper",5,5}}) {
        limb.names[0]=c.parent;limb.names[1]=std::string(c.parent)+"/"+c.child;
        const auto setup=fbik::joints::setup(limb,bent,frame,1);
        require(!setup.limits.hinge && setup.limits.lower==-c.twist && setup.limits.upper==c.twist &&
                setup.limits.swing==c.swing,"Wrong anatomical ball-joint limits");
    }
    std::cout << "Character frames, signed hinge axes, bent-pose zero and anatomical limit presets passed.\n";
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
             "fbik:joint", "fbik:skeleton", "fbik:swingLimit", "fbik:quasiStatic", "fbik:followBody"}) properties.push_back(st::str(name));
        selector.property_predicate=ovstage_population_property_predicate_has_name(properties.data(), properties.size());
        const ovx_string_t metadata[]={st::str("interpolation"),st::str("elementSize")};
        selector.property_metadata_paths=metadata;selector.property_metadata_path_count=2;
        population.selectors=&selector;population.selector_count=1;
        const auto units=st::str("metersPerUnit");population.stage_metadata_paths=&units;population.stage_metadata_path_count=1;
        const auto path=file.u8string();
        auto op=ovstage_population_open_usd_from_file_with_desc(stage,st::str(path.c_str()),ordinal,0,&population,1);
        st::check(op.status);ovstage_population_op_wait_result_t result{};
        const auto status=ovstage_population_wait_op(stage,op.op_index,OVSTAGE_TIMEOUT_INFINITE,&result);
        if (result.error_op_id_count) {
            const auto error=ovstage_population_get_last_op_error(result.error_op_ids[0]);
            throw std::runtime_error("Fixture: "+std::string(error.ptr,error.length));
        }
        st::check(status);
        require(result.error_op_id_count==0,"Failed to populate fixture");
        st::seal(stage,ordinal);st::hierarchy(stage,ordinal);
    }
};
struct Physics
{
    ovphysx_handle_t handle=OVPHYSX_INVALID_HANDLE;
    ~Physics() { if(handle) {ovphysx_detach_ovstage(handle);ovphysx_destroy_instance(handle);} }
};
void testCapsules(const std::filesystem::path &root)
{
    namespace sk=viewer::skeleton;
    using namespace MeshTools;
    Scene scene;scene.load(root/"tests/data/capsules.usda");
    fbik::Module module;
    const auto stats=module.init(scene.stage,scene.ordinal);
    require(stats.capsules==3 && stats.joints==1,"Capsule body topology is incorrect");
    const auto state=st::read(scene.stage,scene.ordinal,{"usd-prim-type","axis","radius","height","extent","omni:fabric:worldMatrix"});
    const std::string bone="/World/Character/__FbikBoneColliders/Skeleton_0/Joint_0";
    const std::string head="/World/Character/__FbikBoneColliders/Skeleton_0/Joint_1";
    const std::string sparse="/World/Sparse/__FbikBoneColliders/Skeleton_0/Joint_0";
    const auto &capsule=state.at(bone),&sphere=state.at(head),&point=state.at(sparse);
    require(capsule.at("usd-prim-type").token=="Capsule" && capsule.at("axis").token=="X","Capsule was not authored along X");
    require(std::abs(capsule.at("radius").numbers[0]-1)<1e-5 &&
            std::abs(capsule.at("height").numbers[0]-6)<1e-5,"Rotated capsule radius or cylinder length is wrong");
    const auto body=sk::world(capsule),terminal=sk::world(sphere);
    require((body.b-Vec3(7.6f,7.8f,6)).magnitude()<1e-4,"Capsule did not use the projected bounds center and character transform");
    require((body.A.column0-Vec3(1.2f,1.6f,0)).magnitude()<1e-4,"Capsule X axis did not follow the bone");
    require((terminal.b-Vec3(16.2f,10.1f,0)).magnitude()<1e-4 &&
            (terminal.A.column0-body.A.column0).magnitude()<1e-4,"Terminal head did not use its incoming direction");
    require(std::abs(sphere.at("radius").numbers[0]-.5)<1e-5 && sphere.at("height").numbers[0]==0,
            "Short capsule did not reduce to a sphere");
    require(std::abs(point.at("radius").numbers[0]-.0005)<1e-8 && point.at("height").numbers[0]==0,
            "Sparse capsule did not retain a positive minimum radius");
    const std::vector<double> expectedExtent={-4,-1,-1,4,1,1};
    const auto &extent=capsule.at("extent").numbers;
    require(extent.size()==expectedExtent.size(),"Missing capsule render bounds");
    for (size_t i=0;i<extent.size();++i) require(std::abs(extent[i]-expectedExtent[i])<1e-5,"Wrong capsule render bounds");
    module.showColliders(true,scene.ordinal);
    {
        Physics physics;ovphysx_create_args args=OVPHYSX_CREATE_ARGS_DEFAULT;
        check(ovphysx_create_instance(&args,&physics.handle));check(ovphysx_attach_ovstage(physics.handle,scene.stage,scene.ordinal));
        check(ovphysx_step_sync(physics.handle,1.0f/60));
        for (const auto &path:{bone,head,sparse}) {
            void *shape=nullptr;
            check(ovphysx_get_physx_ptr(physics.handle,{path.c_str(),path.size()},OVPHYSX_PHYSX_TYPE_SHAPE,&shape));
            require(shape && static_cast<physx::PxShape*>(shape)->getGeometry().getType()==physx::PxGeometryType::eCAPSULE,
                    "PhysX did not create a native capsule, including the sphere case");
            const auto &geometry=static_cast<const physx::PxCapsuleGeometry&>(static_cast<physx::PxShape*>(shape)->getGeometry());
            const float radius=path==bone ? 2.f : path==head ? 1.f : .0005f;
            const float halfHeight=path==bone ? 6.f : 0.f;
            require(std::abs(geometry.radius-radius)<1e-5 && std::abs(geometry.halfHeight-halfHeight)<1e-5,
                    "PhysX capsule dimensions lost scaling or used full length as halfHeight");
        }
    }
    const auto repeat=module.init(scene.stage,scene.ordinal);
    require(repeat.capsules==stats.capsules && repeat.joints==stats.joints,
            "Reinitialization changed capsule topology");
    std::cout << "Capsule fitting, bounds, terminal/sparse spheres, native PhysX shapes and reinitialization passed.\n";
}
void testSelfCollision(const std::filesystem::path &root)
{
    namespace sk=viewer::skeleton;
    Scene scene;scene.load(root/"tests/data/self_collision.usda");
    fbik::Module module;
    module.init(scene.stage,scene.ordinal);
    const std::string base="/World/Character/__FbikBoneColliders/Skeleton_0/Joint_";
    const std::string a=base+"0",b=base+"1",c=base+"2",d=base+"3",e=base+"4";
    const std::string other="/World/Other/__FbikBoneColliders/Skeleton_0/Joint_0";
    const auto excluded=[&](const std::string &p,const std::string &q) {
        for (const auto &pair:module.collisionExclusions())
            if ((pair.body0==p && pair.body1==q) || (pair.body0==q && pair.body1==p)) return true;
        return false;
    };
    require(excluded(a,b),"Rest capsule overlap was not filtered");
    require(!excluded(a,c),"Initial animation overlap was incorrectly made permanent");
    require(!excluded(a,e),"Disjoint capsules with overlapping bounds were filtered");
    require(!excluded(a,d),"Separated capsules were filtered");
    require(!excluded(a,other),"Different characters were filtered");
    const auto stats=module.init(scene.stage,scene.ordinal);
    require(excluded(a,b) && !excluded(a,d) && !excluded(a,e) && !excluded(a,c) && !excluded(a,other),"Reinitialization changed capsule filtering");
    require(stats.connectedExclusions==0 && stats.overlapExclusions==module.collisionExclusions().size(),
            "Collision exclusion report has incorrect counts/reasons");
    const auto state=st::read(scene.stage,scene.ordinal,{"usd-prim-type","physics:filteredPairs"});
    size_t pairCount=0;
    for (const auto &[path,prim]:state) {
        require(sk::get(prim,"usd-prim-type").token!="PhysicsCollisionGroup","Blanket bone collision group remains");
        for (const auto &target:sk::get(prim,"physics:filteredPairs").paths) {
            require(excluded(path,target),"Authored pair filter differs from the report");++pairCount;
        }
    }
    require(pairCount==module.collisionExclusions().size(),"Pair filters missing or duplicated in USD");
    Physics physics;ovphysx_create_args args=OVPHYSX_CREATE_ARGS_DEFAULT;
    check(ovphysx_create_instance(&args,&physics.handle));check(ovphysx_attach_ovstage(physics.handle,scene.stage,scene.ordinal));
    check(ovphysx_step_sync(physics.handle,1.f/60));
    std::map<std::string,physx::PxRigidDynamic*> bodies;
    for (const auto &path:{a,b,c,d,e,other}) {
        void *actor=nullptr;check(ovphysx_get_physx_ptr(physics.handle,{path.c_str(),path.size()},OVPHYSX_PHYSX_TYPE_ACTOR,&actor));
        auto *body=actor ? static_cast<physx::PxRigidActor*>(actor)->is<physx::PxRigidDynamic>() : nullptr;
        require(body!=nullptr,"Missing bone actor for collision test");bodies[path]=body;
    }
    const auto separation=[&](const std::string &partner) {
        int index=0;
        for (const auto &[path,body]:bodies) {
            body->setRigidBodyFlag(physx::PxRigidBodyFlag::eKINEMATIC,false);
            body->setActorFlag(physx::PxActorFlag::eDISABLE_GRAVITY,true);
            body->setLinearVelocity({0,0,0});body->setAngularVelocity({0,0,0});
            auto pose=body->getGlobalPose();
            pose.p=physx::PxVec3(100+10.f*index++,0,0);body->setGlobalPose(pose);
        }
        auto pose0=bodies.at(a)->getGlobalPose(),pose1=bodies.at(partner)->getGlobalPose();
        pose0.p={0,0,0};pose1.p={.5f,0,0};
        bodies.at(a)->setGlobalPose(pose0);bodies.at(partner)->setGlobalPose(pose1);
        for (int frame=0;frame<20;++frame) check(ovphysx_step_sync(physics.handle,1.f/60));
        return (bodies.at(a)->getGlobalPose().p-bodies.at(partner)->getGlobalPose().p).magnitude();
    };
    require(std::abs(separation(b)-.5f)<1e-4,"Rest-excluded bodies generated collision response");
    require(separation(d)>.95f,"Separated rest capsules did not collide");
    require(separation(e)>.95f,"Capsules with overlapping rest bounds did not collide");
    require(separation(c)>.95f,"Unfiltered bones in the same character did not collide");
    require(separation(other)>.95f,"Bones in different characters did not collide");
    std::cout << "Rest capsule filters, overlapping-bounds regression, initialization-pose independence, native self/cross-character collisions passed.\n";
}
void testJointDamping(const std::filesystem::path &root)
{
    using namespace physx;
    std::vector<float> undamped;
    for (const float damping:{0.f,5.f}) {
        Scene scene;scene.load(root/"tests/data/joint_damping.usda");
        fbik::Module module;
        const auto stats=damping==0 ? module.init(scene.stage,scene.ordinal,.3f,0)
                                    : module.init(scene.stage,scene.ordinal);
        require(stats.joints==2 && stats.hinges==1,"Damping fixture lost its ball joint or hinge");
        const auto before=scene.ordinal;
        bool rejected=false;
        try { module.init(scene.stage,scene.ordinal,.3f,-1); }
        catch (const std::runtime_error &) { rejected=true; }
        require(rejected && scene.ordinal==before,"Invalid damping changed the initialized scene");
        module.setMode(fbik::Mode::Ragdoll,scene.ordinal);
        Physics physics;ovphysx_create_args args=OVPHYSX_CREATE_ARGS_DEFAULT;
        check(ovphysx_create_instance(&args,&physics.handle));check(ovphysx_attach_ovstage(physics.handle,scene.stage,scene.ordinal));
        check(ovphysx_step_sync(physics.handle,1.f/60));
        size_t sample=0;
        for (const std::string name:{"Ball","Hinge"}) {
            const auto path="/World/"+name+"/__FbikBoneColliders/Skeleton_0/Attachment_1";
            void *pointer=nullptr;
            check(ovphysx_get_physx_ptr(physics.handle,{path.c_str(),path.size()},OVPHYSX_PHYSX_TYPE_JOINT,&pointer));
            auto *joint=pointer ? static_cast<PxJoint*>(pointer)->is<PxD6Joint>() : nullptr;
            require(joint!=nullptr,"Generated damping joint was not realized");
            PxRigidActor *actor0=nullptr,*actor1=nullptr;joint->getActors(actor0,actor1);
            auto *parent=actor0->is<PxRigidDynamic>(),*child=actor1->is<PxRigidDynamic>();
            require(parent && child,"Damping fixture is missing dynamic bodies");
            const auto pose0=parent->getGlobalPose(),pose1=child->getGlobalPose();
            const auto frame=pose0*joint->getLocalPose(PxJointActorIndex::eACTOR0);
            const auto reset=[&] {
                parent->setGlobalPose(pose0);child->setGlobalPose(pose1);
                for (auto *body:{parent,child}) {
                    body->setLinearVelocity(PxVec3(0));body->setAngularVelocity(PxVec3(0));
                    body->setLinearDamping(0);body->setAngularDamping(0);body->setSleepThreshold(0);
                }
            };
            const auto step=[&] {
                for (int i=0;i<30;++i) check(ovphysx_step_sync(physics.handle,1.f/60));
            };
            const PxD6Drive::Enum drives[]={PxD6Drive::eTWIST,PxD6Drive::eSWING1,PxD6Drive::eSWING2};
            const PxVec3 axes[]={{1,0,0},{0,1,0},{0,0,1}};
            for (int i=0;i<(name=="Hinge" ? 1 : 3);++i) {
                const auto drive=joint->getDrive(drives[i]);
                require(drive.stiffness==0 && std::abs(drive.damping-damping)<1e-4 &&
                        (drive.flags & PxD6JointDriveFlag::eACCELERATION),
                        "Native angular damping has wrong units, drive mode or stiffness");
                reset();
                const auto axis=frame.q.rotate(axes[i]);
                child->setAngularVelocity(axis);
                child->setLinearVelocity(axis.cross(pose1.p-frame.p));
                step();
                const auto speed=joint->getRelativeAngularVelocity().magnitude();
                if (damping==0) { require(speed>.5f,"Undamped fixture hit an angular limit");undamped.push_back(speed); }
                else require(speed<undamped.at(sample)*.4f,"Joint damping did not reduce relative angular motion");
                ++sample;
            }
            reset();
            const PxQuat offset(.2f,frame.q.rotate(PxVec3(1,0,0)));
            child->setGlobalPose(PxTransform(frame.p+offset.rotate(pose1.p-frame.p),offset*pose1.q));
            const auto held=joint->getRelativeTransform().q;
            step();
            require(std::abs(held.dot(joint->getRelativeTransform().q))>1-1e-6f &&
                    joint->getRelativeAngularVelocity().magnitude()<1e-3f,
                    "Damping added a spring toward the rest pose");
            if (name=="Ball") {
                reset();
                const auto spin=frame.q.rotate(PxVec3(.5f,0,0));
                parent->setAngularVelocity(spin);child->setAngularVelocity(spin);
                step();
                require((parent->getAngularVelocity()-spin).magnitude()<1e-3f &&
                        (child->getAngularVelocity()-spin).magnitude()<1e-3f,
                        "Joint damping slowed common rigid rotation");
            }
        }
    }
    std::cout << "Joint damping: native gains, hinge/twist/swing decay, zero damping, free pose and common rotation passed.\n";
}
void testAngularLimits(const std::filesystem::path &root)
{
    namespace sk=viewer::skeleton;
    using namespace MeshTools;
    Scene scene;scene.load(root/"tests/data/joint_limits.usda");
    struct Case { const char *name; float angle,lower,upper; bool swing,unchanged; };
    const std::vector<Case> cases={
        {"Swing",75,0,60,true,false},{"TwistPositive",70,-30,30,false,false},{"TwistNegative",-70,-30,30,false,false},
        {"Shoulder",110,0,100,true,false},{"KneePositive",20,-60,0,false,false},{"KneeNegative",-90,-60,0,false,false},
        {"RightElbow",150,0,120,false,false},{"LeftElbow",-150,-120,0,false,false},
        {"Inside",55,0,60,true,true},{"ShoulderInside",90,0,100,true,true},
        {"InternalSwing",20,0,5,true,false},{"InternalTwist",15,-5,5,false,false},
        {"InternalInside",3,0,5,true,true},{"KneeInside",-35,-60,0,false,true}};
    std::vector<std::string> paths;
    for (const auto &c:cases) paths.push_back(std::string("/World/")+c.name);
    st::Query query;query.init(scene.stage,paths);
    for (int run=0;run<2;++run) {
        viewer::PhysicsSimulation physics(scene.stage,scene.ordinal);
        std::vector<fbik::Matrix4> poses;
        for (size_t i=0;i<cases.size();++i) {
            const auto &c=cases[i];
            const Vec3 axis=c.swing ? Vec3(0,1,1).normalized() : Vec3(1,0,0);
            poses.push_back(sk::packed(sk::Matrix(Mat33(Quat(c.angle*Pi/180,axis)),Vec3(float(i)*2,0,0))));
        }
        query.writeTransforms(++scene.ordinal,poses,&poses);st::seal(scene.stage,scene.ordinal);
        physics.applyControlChanges(scene.ordinal);
        physics.step(scene.ordinal);
        const auto moving=sk::world(st::read(scene.stage,scene.ordinal,{"omni:fabric:worldMatrix"}).at("/World/Moving"));
        require((moving.b-Vec3(30+1.f/60,2.f/60,3.f/60)).magnitude()<1e-4,"Warmup changed authored linear velocity");
        require(std::abs(std::atan2(-moving.A.column0.z,moving.A.column0.x)-Pi/120)<1e-4,
                "Warmup changed authored angular velocity or its degree/radian units");
        // Swing=75 degrees around the Y/Z diagonal lies inside the authored
        // +/-60 pyramid, but outside the requested circular 60-degree cone.
        for (int frame=1;frame<120;++frame) physics.step(scene.ordinal);
        const auto state=st::read(scene.stage,scene.ordinal,{"omni:fabric:worldMatrix"});
        for (size_t i=0;i<cases.size();++i) {
            const auto &c=cases[i];const auto m=sk::world(state.at(paths[i]));
            const float angle=(c.swing ? std::acos(std::clamp(m.A.column0.x,-1.f,1.f)) :
                                           std::atan2(m.A.column1.z,m.A.column1.y))*180/Pi;
            std::cout << c.name << " angle=" << angle << " degrees\n";
            require(angle>=c.lower-1 && angle<=c.upper+1,"Simulated joint exceeded its angular limit");
            if (c.unchanged) require(std::abs(angle-c.angle)<.05,"Limit moved an allowed pose");
            require((m.b-Vec3(float(i)*2,0,0)).magnitude()<.001,"Angular-limit joint translation was not locked");
        }
        physics.stop(scene.ordinal);
    }
    std::cout << "Hinge limits, positive/negative twist, circular swing cones and reset passed.\n";
}
void testSomaJointSetup(const std::filesystem::path &root)
{
    namespace sk=viewer::skeleton;
    using namespace MeshTools;
    Scene scene;scene.load(root/"resources/scenes/basic_walk.usda");
    fbik::Module module;module.init(scene.stage,scene.ordinal);
    const auto &character=module.characters().front();
    const auto pose=viewer::sampleAnimation(scene.stage,scene.ordinal,0,module.characters()).front();
    const auto world=sk::unpacked(pose.characterWorld);
    const auto position=[&](const std::string &name) {
        const auto it=std::find(character.joints.begin(),character.joints.end(),name);
        require(it!=character.joints.end(),"Missing SOMA bone");
        return world.transform(sk::unpacked(pose.bones[size_t(it-character.joints.begin())]).b);
    };
    const auto state=st::read(scene.stage,scene.ordinal,{"usd-prim-type","omni:fabric:worldMatrix","fbik:joint",
        "physics:body0","physics:body1","physics:localRot0","physics:lowerLimit","physics:upperLimit",
        "limit:rotX:physics:low","limit:rotX:physics:high","fbik:swingLimit"});
    size_t knees=0,internal=0,mobile=0;
    for (const auto &[path,prim]:state) {
        if(sk::get(prim,"physics:body0").paths.empty() || sk::get(prim,"physics:body1").paths.empty()) continue;
        const auto &childPath=prim.at("physics:body1").paths[0];
        const auto &joint=state.at(childPath).at("fbik:joint").token;
        const auto name=fbik::joints::boneName(joint);
        if(sk::get(prim,"usd-prim-type").token=="PhysicsRevoluteJoint") {
            if(name!="leftshin" && name!="rightshin") continue;
            ++knees;
            const std::string side=name=="leftshin" ? "Left" : "Right";
            const auto leg="Hips/"+side+"Leg";
            const auto shin=leg+"/"+side+"Shin",foot=shin+"/"+side+"Foot",toe=foot+"/"+side+"ToeBase";
            Vec3 forward=position(toe)-position(foot);forward.y=0;forward.normalize();
            const auto right=forward.cross(Vec3(0,1,0));
            const auto upper=(position(shin)-position(leg)).normalized();
            const auto &r=prim.at("physics:localRot0").numbers;
            const auto body=sk::world(state.at(prim.at("physics:body0").paths[0]));
            const auto axis=(body.A*Mat33(Quat(float(r[0]),float(r[1]),float(r[2]),float(r[3])))).column0.normalized();
            require(axis.dot(right)>.99,"Rotated SOMA arm span gave the knee the wrong hinge axis");
            const float lower=float(prim.at("physics:lowerLimit").numbers[0]);
            require(prim.at("physics:upperLimit").numbers[0]==0,"SOMA knee permits hyperextension");
            require((Mat33(Quat(lower*Pi/180,axis))*upper).dot(forward)<-.5,
                    "SOMA knee limit permits forward bending instead of backward bending");
        }
        else {
            int twist=5,swing=5;
            if(name=="leftarm" || name=="rightarm") { twist=60;swing=100; }
            else if(name=="leftleg" || name=="rightleg") { twist=30;swing=60; }
            else if(name=="head") { twist=20;swing=30; }
            require(prim.at("limit:rotX:physics:low").numbers[0]==-twist &&
                    prim.at("limit:rotX:physics:high").numbers[0]==twist &&
                    prim.at("fbik:swingLimit").numbers[0]==swing,"Wrong generated SOMA joint limits");
            if(twist==5) ++internal;else ++mobile;
        }
    }
    require(knees==2 && internal==13 && mobile==5,"SOMA joint classification missed bones");
    std::cout << "SOMA has 13 tightly limited internal joints, five mobile ball joints and two backward-bending knees.\n";
}
void testDirectRagdollStart(const std::filesystem::path &root)
{
    namespace sk=viewer::skeleton;
    Scene scene;scene.load(root/"resources/scenes/basic_walk.usda");
    fbik::Module module;
    const auto stats=module.init(scene.stage,scene.ordinal);
    require(stats.capsules==23 && stats.joints==22,"SOMA capsule topology changed");
    const auto reference=viewer::sampleAnimation(scene.stage,scene.ordinal,0,module.characters());
    module.setMode(fbik::Mode::Ragdoll,scene.ordinal);
    std::vector<fbik::Pose> previous;
    for (int run=0;run<2;++run) {
        // No preceding Kinematic step: enabled native joints need initialization.
        viewer::PhysicsSimulation physics(scene.stage,scene.ordinal);
        physics.step(scene.ordinal);
        const auto poses=module.readPoses(reference,scene.ordinal);
        const auto world=sk::unpacked(reference[0].characterWorld);
        for (size_t j=0;j<poses[0].bones.size();++j) {
            const auto before=world.transform(sk::unpacked(reference[0].bones[j]).b);
            const auto after=world.transform(sk::unpacked(poses[0].bones[j]).b);
            require((after-before).magnitude()<.1,"Direct ragdoll startup injected a large impulse");
            if (run) for(size_t k=0;k<16;++k)
                require(std::abs(poses[0].bones[j][k]-previous[0].bones[j][k])<1e-4,"Ragdoll startup/reset was not repeatable");
        }
        previous=poses;
        physics.stop(scene.ordinal);
    }
    std::cout << "Direct ragdoll startup preserves state and has repeatable first ticks.\n";
}
void testQuasiStatic(const std::filesystem::path &root)
{
    namespace sk=viewer::skeleton;
    {
        Scene scene;scene.load(root/"tests/data/quasi_static.usda");
        viewer::PhysicsSimulation physics(scene.stage,scene.ordinal);
        const auto world=[&](const char *path) {
            return sk::world(st::read(scene.stage,scene.ordinal,{"omni:fabric:worldMatrix"}).at(path));
        };
        const auto initial=sk::packed(world("/World/Free"));
        for (int frame=0;frame<30;++frame) physics.step(scene.ordinal,false);
        physics.publish(scene.ordinal);
        const auto held=sk::packed(world("/World/Free"));
        for (size_t k=0;k<16;++k)
            require(std::abs(initial[k]-held[k])<1e-5,"Quasi static retained initial velocity or gravity");
        require(world("/World/Falling").b.y<3,"Quasi static disabled gravity on an ordinary scene body");
        require(world("/World/Contact").b.y>.49,"Quasi static disabled contact correction");
        require(physics.beginDrag({0,4,6},{0,0,-1},scene.ordinal),"Quasi static body could not be grabbed");
        physics.moveDrag({1,5,6},{0,0,-1},scene.ordinal);
        for (int frame=0;frame<10;++frame) physics.step(scene.ordinal);
        physics.endDrag(scene.ordinal);
        const auto released=sk::packed(world("/World/Free"));
        std::cout << "Quasi static drag: " << released[12] << ", " << released[13] << ", " << released[14] << '\n';
        require(released[12]>.1 && released[13]>4.1,"Quasi static dragging did not move the body");
        // Disable only velocity clearing (keep gravity off) to observe whether
        // the preceding tick really discarded the mouse joint's linear/angular momentum.
        auto path=st::str("/World/Free");
        ovstage_predicate_t predicate{};predicate.attribute.string=st::str("usd-path");
        predicate.op=OVSTAGE_FILTER_OP_IN;predicate.values=&path;predicate.value_count=1;
        ovstage_filter_t filter{&predicate,1};ovstage_query_handle_t query{};
        st::wait(scene.stage,ovstage_query(scene.stage,&filter,nullptr,0,&query));
        uint8_t disabled=0;int64_t shape=1;
        DLTensor tensor{};tensor.data=&disabled;tensor.device={kDLCPU,0};tensor.ndim=1;
        tensor.shape=&shape;tensor.dtype={kDLBool,8,1};
        ovstage_write_data_t data{};data.tensors=&tensor;data.tensor_count=1;
        ovx_string_or_token_t attribute{};attribute.string=st::str("fbik:quasiStatic");
        st::wait(scene.stage,ovstage_write_attribute(scene.stage,query,attribute,++scene.ordinal,data,OVSTAGE_PRIM_MODE_UPSERT));
        st::wait(scene.stage,ovstage_release_query(scene.stage,query));
        st::seal(scene.stage,scene.ordinal);physics.applyControlChanges(scene.ordinal);
        for (int frame=0;frame<10;++frame) physics.step(scene.ordinal);
        const auto stopped=sk::packed(world("/World/Free"));
        for (size_t k=0;k<16;++k)
            require(std::abs(stopped[k]-released[k])<1e-4,"Quasi static left solver momentum after the tick");
        physics.stop(scene.ordinal);
    }
    {
        Scene scene;scene.load(root/"tests/data/ragdoll.usda");
        fbik::Module module;module.init(scene.stage,scene.ordinal);
        const auto reference=viewer::sampleAnimation(scene.stage,scene.ordinal,0,module.characters());
        const auto targets=module.updateTargets(reference);
        module.setMode(fbik::Mode::QuasiStatic,scene.ordinal);
        const auto sameOrdinal=scene.ordinal;
        module.setMode(fbik::Mode::QuasiStatic,scene.ordinal);
        require(scene.ordinal==sameOrdinal,"Repeated quasi static selection wrote controls");
        viewer::PhysicsSimulation physics(scene.stage,scene.ordinal);
        const auto checkMode=[&](bool quasiStatic,bool follow) {
            const auto state=st::read(scene.stage,scene.ordinal,{"physics:kinematicEnabled","physxRigidBody:disableGravity",
                "fbik:quasiStatic","physics:body0","physics:jointEnabled"});
            for (const auto &t:targets) {
                const auto &prim=state.at(t.path);
                require((prim.at("physics:kinematicEnabled").numbers[0]!=0)==follow,"Wrong kinematic flag after mode change");
                require((prim.at("physxRigidBody:disableGravity").numbers[0]!=0)==quasiStatic,"Wrong gravity flag after mode change");
                require((prim.at("fbik:quasiStatic").numbers[0]!=0)==quasiStatic,"Wrong velocity policy after mode change");
            }
            for (const auto &[path,prim]:state) if(!sk::get(prim,"physics:body0").paths.empty())
                require((prim.at("physics:jointEnabled").numbers[0]!=0)==!follow,"Wrong joint state after mode change");
        };
        const auto checkPose=[&](const std::vector<fbik::Pose> &expected) {
            const auto actual=module.readPoses(reference,scene.ordinal);
            for(size_t c=0;c<actual.size();++c) for(size_t j=0;j<actual[c].bones.size();++j) for(size_t k=0;k<16;++k)
                require(std::abs(actual[c].bones[j][k]-expected[c].bones[j][k])<.001,"Quasi static pose drifted or reset");
        };
        checkMode(true,false);
        for(int frame=0;frame<60;++frame) physics.step(scene.ordinal);
        checkPose(reference);
        module.setMode(fbik::Mode::Ragdoll,scene.ordinal);physics.applyControlChanges(scene.ordinal);
        checkMode(false,false);
        for(int frame=0;frame<15;++frame) physics.step(scene.ordinal);
        const auto falling=module.readPoses(reference,scene.ordinal);
        require(falling[0].bones[0][13]<reference[0].bones[0][13]-.1,"Ragdoll failed to restore gravity");
        module.setMode(fbik::Mode::QuasiStatic,scene.ordinal);physics.applyControlChanges(scene.ordinal);
        checkMode(true,false);
        for(int frame=0;frame<30;++frame) physics.step(scene.ordinal);
        checkPose(falling);
        module.setMode(fbik::Mode::Kinematic,scene.ordinal);physics.applyControlChanges(scene.ordinal);
        checkMode(false,true);
        physics.setKinematicTargets(targets,scene.ordinal);physics.step(scene.ordinal);
        checkPose(reference);
        physics.stop(scene.ordinal);
    }
    std::cout << "Quasi static gravity, velocity clearing, contacts, dragging and mode transitions passed.\n";
}
void testRagdoll(const std::filesystem::path &root)
{
    namespace sk=viewer::skeleton;
    Scene scene;scene.load(root/"tests/data/ragdoll.usda");
    fbik::Module module;const auto stats=module.init(scene.stage,scene.ordinal);
    require(stats.capsules==4 && stats.joints==2 && stats.degenerate==2,"Ragdoll topology mismatch");
    require(stats.connectedExclusions==2,"Connected filtering did not bridge omitted parent bodies");
    for (const auto &pair:module.collisionExclusions())
        require(pair.reason==fbik::ExclusionReason::Connected,"Separated characters acquired overlap exclusions");
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
            if(sk::get(prim,"physics:body0").paths.empty() || sk::get(prim,"physics:body1").paths.empty()) continue;
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
                for(const auto &[path,prim]:setup) if(!sk::get(prim,"physics:body0").paths.empty())
                    require(flags.at(path).at("physics:jointEnabled").numbers[0]==1,"Dragging disabled a bone joint");
            }
            if(frame==49) physics.endDrag(scene.ordinal);
            if(frame==14) {
                const auto poses=module.readPoses(reference,scene.ordinal);
                require(poses[0].bones[0][13]<reference[0].bones[0][13]-.1,"Ragdoll did not fall under gravity");
                for(size_t j=0;j<poses[0].bones.size();++j) for(int k=0;k<16;++k)
                    require(std::abs(poses[0].bones[j][k]-poses[1].bones[j][k])<.001,
                            "Separated characters have different local trajectories");
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
        module.setMode(fbik::Mode::Kinematic,scene.ordinal);
        physics.applyControlChanges(scene.ordinal);
        physics.setKinematicTargets(targets,scene.ordinal);
        for(int i=0;i<4;++i) physics.step(scene.ordinal);
        const auto restored=module.readPoses(reference,scene.ordinal);
        for(size_t i=0;i<restored.size();++i) for(size_t j=0;j<restored[i].bones.size();++j)
            for(int k=0;k<16;++k) require(std::abs(restored[i].bones[j][k]-reference[i].bones[j][k])<.001,
                                          "Returning to Kinematic failed to restore/hold targets");
    }
    physics.stop(scene.ordinal);
    module.setMode(fbik::Mode::Ragdoll,scene.ordinal);
    const auto repeat=module.init(scene.stage,scene.ordinal);
    require(module.mode()==fbik::Mode::Kinematic && repeat.joints==stats.joints,"Reinit did not reset mode and joints");
    std::cout << "Ragdoll transitions, joint anchors, skipped bones, collision filtering and floor contact passed.\n";
}
void testCollapsedBones(const std::filesystem::path &root)
{
    namespace sk=viewer::skeleton;
    using namespace MeshTools;
    Scene scene;scene.load(root/"tests/data/collapsed.usda");
    fbik::Module module;const auto stats=module.init(scene.stage,scene.ordinal);
    require(stats.capsules==12 && stats.joints==11 &&
            stats.collapsedBones==9 && stats.leaves==2 && stats.degenerate==0,"Collapsed topology mismatch");
    const std::string base="/World/Character/__FbikBoneColliders/Skeleton_0/";
    const auto bodyPath=[&](size_t j) { return base+"Joint_"+std::to_string(j); };
    const auto data=st::read(scene.stage,scene.ordinal,{"usd-prim-type","omni:fabric:worldMatrix","joints",
        "restTransforms","bindTransforms","points","radius","height","physics:body0","physics:body1"});
    const auto skeleton=sk::readSkeleton(data,"/World/Character/Rig","/World/Character");
    const std::map<size_t,size_t> collapsed={{4,3},{5,3},{6,3},{12,11},{13,11},{16,15},{17,15},{18,15},{19,15}};
    for (const auto &[j,owner]:collapsed) {
        require(!data.count(bodyPath(j)),"Collapsed descendant has a physics body");
        require(!data.count(base+"Attachment_"+std::to_string(j)),"Collapsed descendant has a physics joint");
    }
    for (size_t j:{1,2,7,9,10,14,21})
        require(data.at(bodyPath(j)).at("usd-prim-type").token=="Capsule","Helper, arm or unrelated bone was collapsed");
    for (size_t j:{3,11,15,20})
        require(data.at(bodyPath(j)).at("usd-prim-type").token=="Capsule","Hand/head/wrist capsule missing");
    require(data.at(base+"Attachment_3").at("physics:body0").paths[0]==bodyPath(2),"Named hand lost its helper parent");
    require(data.at(base+"Attachment_11").at("physics:body0").paths[0]==bodyPath(10),"Fallback hand lost its forearm parent");

    // Check the fitted bounds against all subtree vertices, including leaves
    // and separate remapped meshes. Weak influences must not enlarge the capsule.
    for (size_t owner:{3,11,15,20}) {
        const auto &prim=data.at(bodyPath(owner));
        const auto toBody=sk::inverse(sk::world(prim))*sk::world(data.at("/World/Character"));
        std::vector<Vec3> points;
        for(size_t j=0;j<skeleton.names.size();++j) {
            if(j!=owner && (!collapsed.count(j) || collapsed.at(j)!=owner)) continue;
            const auto &source=data.at("/World/Character/Skin_"+std::to_string(j)).at("points").numbers;
            for(size_t v=0;v<source.size();v+=3)
                points.push_back(toBody.transform({float(source[v]),float(source[v+1]),float(source[v+2])}));
        }
        if(owner==3) {
            const auto &source=data.at("/World/Character/Shared").at("points").numbers;
            for(size_t v=0;v<source.size();v+=3)
                points.push_back(toBody.transform({float(source[v]),float(source[v+1]),float(source[v+2])}));
        }
        Bounds3 bounds(Empty);
        for (const auto &point:points) bounds.include(point);
        const auto spans=bounds.getDimensions();
        require(bounds.getCenter().magnitude()<1e-4,"Descendant skin did not center the capsule");
        require(std::abs(prim.at("radius").numbers[0]-.5f*std::min(spans.y,spans.z))<1e-4 &&
                std::abs(prim.at("height").numbers[0]-std::max(0.f,spans.x-std::min(spans.y,spans.z)))<1e-4,
                "Descendant skin aggregation or weak-influence rejection changed capsule dimensions");
    }

    auto reference=viewer::sampleAnimation(scene.stage,scene.ordinal,0,module.characters());
    const auto initialTargets=module.updateTargets(reference);
    // Give fingers, jaw and eyes a non-rest input pose, including nested rotations.
    const auto original=reference[0].bones;
    const Mat33 rotation(Quat(0,0,float(std::sin(.2)),float(std::cos(.2))));
    for(size_t j=0;j<original.size();++j) if(collapsed.count(j)) {
        const size_t p=size_t(skeleton.parents[j]);
        auto local=sk::inverse(sk::unpacked(original[p]))*sk::unpacked(original[j]);
        local.A=rotation*local.A;
        reference[0].bones[j]=sk::packed(sk::unpacked(reference[0].bones[p])*local);
    }
    const auto targets=module.updateTargets(reference);
    require(targets.size()==12 && reference[0].bones.size()==23,"Visual and collision skeleton sizes are incorrect");
    for(size_t i=0;i<targets.size();++i)
        require(targets[i].local==initialTargets[i].local,"Finger/face animation changed a physics target");
    module.setMode(fbik::Mode::Ragdoll,scene.ordinal);
    // Publish deterministic body output with rotation and translation, so the test
    // detects frozen world transforms as well as overwritten descendant local angles.
    const sk::Matrix delta(rotation,{.3f,-.4f,.2f});
    std::vector<std::string> paths;
    std::vector<fbik::Matrix4> locals,worlds;
    for(const auto &target:targets) {
        paths.push_back(target.path);
        locals.push_back(sk::packed(delta*sk::unpacked(target.local)));
        worlds.push_back(sk::packed(sk::unpacked(reference[0].characterWorld)*sk::unpacked(locals.back())));
    }
    st::Query query;query.init(scene.stage,paths);
    query.writeTransforms(++scene.ordinal,locals,&worlds);st::seal(scene.stage,scene.ordinal);
    const auto result=module.readPoses(reference,scene.ordinal);
    for(size_t j:{3,11,15,20}) {
        const auto expected=sk::packed(delta*sk::unpacked(reference[0].bones[j]));
        for(size_t k=0;k<16;++k)
            require(std::abs(expected[k]-result[0].bones[j][k])<1e-4,"Hand/head output did not follow its simulated body");
    }
    for(const auto &[j,owner]:collapsed) {
        const auto p=size_t(skeleton.parents[j]);
        const auto before=sk::packed(sk::inverse(sk::unpacked(reference[0].bones[p]))*sk::unpacked(reference[0].bones[j]));
        const auto after=sk::packed(sk::inverse(sk::unpacked(result[0].bones[p]))*sk::unpacked(result[0].bones[j]));
        for(size_t k=0;k<16;++k) require(std::abs(before[k]-after[k])<1e-4,"Collapsed local joint angle changed");
    }
    query.clear();
    const auto repeat=module.init(scene.stage,scene.ordinal);
    require(repeat.collapsedBones==stats.collapsedBones && repeat.joints==stats.joints &&
            repeat.capsules==stats.capsules,"Reinit changed collapsed topology");
    std::cout << "Hand/head capsule aggregation, name fallback, helpers and preserved descendant angles passed.\n";
}
void testFollow(const std::filesystem::path &root)
{
    namespace sk=viewer::skeleton;
    using namespace MeshTools;
    Scene scene;scene.load(root/"tests/data/follow.usda");
    fbik::Module module;module.init(scene.stage,scene.ordinal);
    const auto reference=viewer::sampleAnimation(scene.stage,scene.ordinal,0,module.characters());
    const auto initial=module.updateTargets(reference);
    require(initial.size()==1,"Follow fixture must have one capsule");
    module.setMode(fbik::Mode::Follow,scene.ordinal);
    viewer::PhysicsSimulation physics(scene.stage,scene.ordinal);
    physics.setFollowTargets(initial,true);
    const auto pose=[&](const std::string &path) {
        return sk::world(st::read(scene.stage,scene.ordinal,{"omni:fabric:worldMatrix"}).at(path));
    };
    const auto body=[&] { return pose(initial[0].path); };
    const auto step=[&](int count) { for(int i=0;i<count;++i) physics.step(scene.ordinal); };
    step(30);
    const auto rest=sk::unpacked(initial[0].world);
    require((body().b-rest.b).magnitude()<.001,"Follow sagged under gravity or moved a held target");
    // Feed-forward targets must track translation and rotation, with no direct pose writes.
    auto targets=initial;
    for (int i=1;i<=60;++i) {
        const sk::Matrix target(Mat33(Quat(.4f*i/60,Vec3(0,0,1)))*rest.A,rest.b+Vec3(.3f*i/60,0,0));
        targets[0].world=sk::packed(target);
        physics.setFollowTargets(targets);
        if(i==1) require((body().b-rest.b).magnitude()<1e-6,"Follow target teleported the body");
        physics.step(scene.ordinal);
    }
    auto desired=sk::unpacked(targets[0].world);
    std::cout << "Follow moving target error: " << (body().b-desired.b).magnitude() << " m\n";
    require((body().b-desired.b).magnitude()<.02 && (body().A.column0-desired.A.column0).magnitude()<.03,
            "Follow failed to track a moving/rotating target");
    physics.setFollowTargets(initial,true);step(90);
    require((body().b-rest.b).magnitude()<.001 && (body().A.column0-rest.A.column0).magnitude()<.001,
            "Follow failed to recover a held position/orientation");
    // A static wall must win over an impossible animation position.
    targets=initial;targets[0].world[12]=1.2;
    physics.setFollowTargets(targets,true);step(120);
    std::cout << "Follow wall position: " << body().b.x << '\n';
    require(body().b.x>.65 && body().b.x<.82,"Follow ignored static contact or could not approach the wall");
    physics.setFollowTargets(initial,true);step(120);
    require((body().b-rest.b).magnitude()<.002,"Follow did not recover after clearing an obstacle");
    float undampedOvershoot=0;
    for(const float damping:{0.f,40.f}) {
        physics.setFollowSettings({400,damping});
        targets=initial;targets[0].world[12]=-.3;
        physics.setFollowTargets(targets,true);
        float overshoot=0;
        for(int i=0;i<60;++i) { step(1);overshoot=std::max(overshoot,-body().b.x-.3f); }
        if(damping==0) { undampedOvershoot=overshoot;require(overshoot>.03,"Undamped spring response was not exercised"); }
        else require(overshoot<undampedOvershoot*.2f,"Damping did not reduce oscillation");
        physics.setFollowSettings({});physics.setFollowTargets(initial,true);step(120);
    }
    // The same sustained mouse load should deform softer animation drives more.
    float soft=0;
    for(const float stiffness:{100.f,1600.f}) {
        physics.setFollowSettings({stiffness,2*std::sqrt(stiffness)});
        require(physics.beginDrag({0,2.5f,3},{0,0,-1},scene.ordinal),"Follow body could not be dragged");
        physics.moveDrag({.6f,2.5f,3},{0,0,-1},scene.ordinal);step(120);
        const float displacement=body().b.x;
        std::cout << "Follow stiffness " << stiffness << ": deflection=" << displacement << '\n';
        if(stiffness==100) { soft=displacement;require(soft>.2,"Soft follow did not yield to sustained load"); }
        else require(displacement>0 && displacement<soft*.6f,"Stiffness did not reduce sustained deformation");
        physics.endDrag(scene.ordinal);step(120);
        require((body().b-rest.b).magnitude()<.003,"Follow failed to recover after releasing a drag");
    }
    physics.setFollowSettings({});
    // A dynamic object must displace the character and receive a response itself.
    st::Query box;box.init(scene.stage,{"/World/Box"});
    const std::vector<fbik::Matrix4> boxPoses={sk::packed(sk::Matrix(Mat33(Identity),Vec3(-.2f,2.5f,0)))};
    box.writeTransforms(++scene.ordinal,boxPoses,&boxPoses);
    st::seal(scene.stage,scene.ordinal);physics.applyControlChanges(scene.ordinal);
    step(1);
    std::cout << "Follow impact: body=" << body().b.x << ", box=" << pose("/World/Box").b.x << '\n';
    require(body().b.x>.005 && pose("/World/Box").b.x<-.205,"Dynamic contact did not move both bodies");
    step(120);
    require((body().b-rest.b).magnitude()<.01,"Follow failed to recover after a dynamic impact");
    // Ragdoll releases the actual pose and disables all animation springs.
    const auto release=body();
    module.setMode(fbik::Mode::Ragdoll,scene.ordinal);physics.applyControlChanges(scene.ordinal);
    require((body().b-release.b).magnitude()<1e-6,"Ragdoll entry snapped the followed pose");
    step(30);
    require(body().b.y<release.b.y-.8,"Follow drive or disabled gravity leaked into ragdoll");
    module.setMode(fbik::Mode::Follow,scene.ordinal);physics.applyControlChanges(scene.ordinal);
    physics.setFollowTargets(initial,true);step(120);
    require((body().b-rest.b).magnitude()<.01,"Re-entering Follow failed to recover from ragdoll");
    module.setMode(fbik::Mode::Kinematic,scene.ordinal);physics.applyControlChanges(scene.ordinal);
    physics.setKinematicTargets(initial,scene.ordinal);step(1);
    require((body().b-rest.b).magnitude()<.001,"Kinematic mode failed after Follow");
    physics.stop(scene.ordinal);
    std::cout << "Follow tracking, rotation, contact, recovery, stiffness, dragging and mode transitions passed.\n";
}

void testFollowHierarchy(const std::filesystem::path &root)
{
    namespace sk=viewer::skeleton;
    using namespace MeshTools;
    Scene scene;scene.load(root/"tests/data/ragdoll.usda");
    fbik::Module module;module.init(scene.stage,scene.ordinal);
    const auto reference=viewer::sampleAnimation(scene.stage,scene.ordinal,0,module.characters());
    module.setMode(fbik::Mode::Follow,scene.ordinal);
    for(int run=0;run<2;++run) {
        viewer::PhysicsSimulation physics(scene.stage,scene.ordinal);
        physics.setFollowTargets(module.updateTargets(reference),true);
        auto desired=reference;
        for(int frame=1;frame<=90;++frame) {
            desired=reference;
            const sk::Matrix motion(Mat33(Quat(.3f*frame/90,Vec3(0,1,0))),Vec3(0,.4f*frame/90,0));
            for(auto &p:desired) p.characterWorld=sk::packed(motion*sk::unpacked(p.characterWorld));
            physics.setFollowTargets(module.updateTargets(desired));
            physics.step(scene.ordinal);
        }
        for(int frame=0;frame<60;++frame) physics.step(scene.ordinal);
        const auto actual=module.readPoses(desired,scene.ordinal);
        for(size_t c=0;c<actual.size();++c) for(size_t j=0;j<actual[c].bones.size();++j)
            require((sk::unpacked(actual[c].bones[j]).b-sk::unpacked(desired[c].bones[j]).b).magnitude()<.01,
                    "Follow hierarchy lost attachments, character transforms or omitted bones");
        physics.stop(scene.ordinal);
    }
    std::cout << "Follow scaled/rotated characters, attachment tracking, omitted bones and restart passed.\n";
}

void testWalkAnimation(const std::filesystem::path &root)
{
    namespace sk=viewer::skeleton;
    using namespace MeshTools;
    Scene scene;scene.load(root/"resources/scenes/basic_walk.usda");
    fbik::Module module;module.init(scene.stage,scene.ordinal);
    viewer::AnimationPlayer animation;animation.init(scene.stage,scene.ordinal,module.characters());
    const auto &names=module.characters().front().joints;
    const auto index=[&](const std::string &leaf) {
        for(size_t i=0;i<names.size();++i)
            if(names[i].substr(names[i].find_last_of('/')+1)==leaf) return i;
        throw std::runtime_error("Missing walk joint: "+leaf);
    };
    const size_t toes[]={index("LeftToeBase"),index("RightToeBase")};
    const size_t legs[]={index("LeftLeg"),index("RightLeg")};
    const size_t knees[]={index("LeftShin"),index("RightShin")};
    const size_t feet[]={index("LeftFoot"),index("RightFoot")};
    const auto initial=animation.sample(scene.ordinal,0).front();
    const auto held=animation.sample(scene.ordinal,1).front();
    require(initial.bones==held.bones,"Held start pose is not stationary");
    std::vector<float> stanceSpeeds;
    Vec3 previous[2];float maxKneeBend=0,maxToeStep=0;
    for(int frame=110;frame<=380;++frame) {
        const auto pose=animation.sample(scene.ordinal,frame/60.).front();
        const auto world=sk::unpacked(pose.characterWorld);
        const auto point=[&](size_t joint) { return world.transform(sk::unpacked(pose.bones[joint]).b); };
        for(int side=0;side<2;++side) {
            const auto toe=point(toes[side]);
            if(frame>110) {
                auto delta=toe-previous[side];
                maxToeStep=std::max(maxToeStep,delta.magnitude());
                if(std::max(toe.y,previous[side].y)<.055f) {
                    delta.y=0;stanceSpeeds.push_back(delta.magnitude()*60);
                }
            }
            previous[side]=toe;
            const auto upper=(point(knees[side])-point(legs[side])).normalized();
            const auto lower=(point(feet[side])-point(knees[side])).normalized();
            maxKneeBend=std::max(maxKneeBend,std::acos(std::clamp(upper.dot(lower),-1.f,1.f))*180/Pi);
        }
    }
    require(stanceSpeeds.size()>100,"Walk lacks sustained stance-foot contacts");
    std::sort(stanceSpeeds.begin(),stanceSpeeds.end());
    const float median=stanceSpeeds[stanceSpeeds.size()/2];
    std::cout << "Authored walk: median stance slip=" << median << " m/s, peak knee bend=" << maxKneeBend << " degrees\n";
    require(median<.12f,"Walk cadence does not match scene travel: stance feet slide");
    require(maxKneeBend>50 && maxKneeBend<100 && maxToeStep<.1f,"Walk lost its swing phase or has a pose discontinuity");
    const auto stopped=animation.sample(scene.ordinal,7).front();
    const auto end=animation.sample(scene.ordinal,8).front();
    require(stopped.bones==end.bones,"Held end pose is not stationary");
}

void testFollowWalk(const std::filesystem::path &root)
{
    namespace sk=viewer::skeleton;
    Scene scene;scene.load(root/"resources/scenes/basic_walk.usda");
    fbik::Module module;module.init(scene.stage,scene.ordinal);
    viewer::AnimationPlayer animation;animation.init(scene.stage,scene.ordinal,module.characters());
    auto reference=animation.sample(scene.ordinal,0);
    module.setMode(fbik::Mode::Follow,scene.ordinal);
    viewer::PhysicsSimulation physics(scene.stage,scene.ordinal);
    physics.setFollowTargets(module.updateTargets(reference),true);
    float worstAttachment=0,meanTracking=0;
    for(int frame=1;frame<=240;++frame) {
        reference=animation.sample(scene.ordinal,frame/60.);
        const auto targets=module.updateTargets(reference);
        physics.setFollowTargets(targets);physics.step(scene.ordinal);
        const auto state=st::read(scene.stage,scene.ordinal,{"omni:fabric:worldMatrix",
            "physics:body0","physics:body1","physics:localPos0","physics:localPos1"});
        float error=0;
        for(const auto &target:targets) {
            const auto actual=sk::world(state.at(target.path));
            const float distance=(actual.b-sk::unpacked(target.world).b).magnitude();
            require(std::isfinite(distance),"Follow walk produced a nonfinite pose");
            error+=distance;
        }
        meanTracking=std::max(meanTracking,error/float(targets.size()));
        for(const auto &[path,prim]:state) if(!sk::get(prim,"physics:body0").paths.empty() &&
                                               !sk::get(prim,"physics:body1").paths.empty()) {
            const auto anchor=[&](const char *body,const char *local) {
                const auto &p=prim.at(local).numbers;
                return sk::world(state.at(prim.at(body).paths[0])).transform({float(p[0]),float(p[1]),float(p[2])});
            };
            worstAttachment=std::max(worstAttachment,(anchor("physics:body0","physics:localPos0")-
                                                     anchor("physics:body1","physics:localPos1")).magnitude());
        }
    }
    std::cout << "Follow walk: worst frame mean tracking=" << meanTracking << " m, attachment=" << worstAttachment << " m\n";
    require(meanTracking<.2 && worstAttachment<.03,"Follow walk lost tracking or skeletal attachments");
    const auto held=module.updateTargets(module.readPoses(reference,scene.ordinal));
    reference=animation.sample(scene.ordinal,1);
    physics.setFollowTargets(module.updateTargets(reference),true);
    physics.publish(scene.ordinal); // No physics step: a seek must only change the goals.
    const auto afterSeek=module.updateTargets(module.readPoses(reference,scene.ordinal));
    for(size_t i=0;i<held.size();++i)
        require((sk::unpacked(held[i].world).b-sk::unpacked(afterSeek[i].world).b).magnitude()<1e-4,
                "Sampling an animated parent moved Follow bodies before simulation");
    physics.stop(scene.ordinal);
}

int main(int argc,char **argv)
{
    int code=0;bool initialized=false;
    try
    {
        testCapsuleOverlap();
        testJointFrames();
        require(argc==2,"Expected repository root");
        const auto root=std::filesystem::absolute(argv[1]);
        const auto binaries=(root/"deps/ovstage/bin").u8string();
        auto entry=ovstage_config_entry_binary_package_root_path(st::str(binaries.c_str()));
        ovstage_config_t config{&entry,1};st::check(ovstage_initialize(&config));initialized=true;
        viewer::PhysicsRuntime runtime;
        testCapsules(root);
        testSelfCollision(root);
        testJointDamping(root);
        testAngularLimits(root);
        testSomaJointSetup(root);
        testDirectRagdollStart(root);
        testQuasiStatic(root);
        testCollapsedBones(root);
        testRagdoll(root);
        testFollow(root);
        testFollowHierarchy(root);
        testWalkAnimation(root);
        testFollowWalk(root);
        {
            Scene scene;scene.load(root/"tests/data/characters.usda");
            fbik::Module module;
            auto stats=module.init(scene.stage,scene.ordinal);
            require(stats.characters==2 && stats.skeletons==3,"Multiple characters/skeletons were not discovered");
            require(stats.capsules==4 && stats.leaves==6 && stats.degenerate==1,"Wrong capsule/leaf counts");
            auto first=st::read(scene.stage,scene.ordinal,{"usd-prim-type","omni:fabric:worldMatrix","fbik:joint","height","radius","physics:kinematicEnabled"});
            const std::string a="/World/A/__FbikBoneColliders/Skeleton_0/";
            require(first.at(a+"Joint_0").at("usd-prim-type").token=="Capsule","Skin collider was not a capsule");
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
            require(repeat.capsules==stats.capsules && second.size()==first.size(),"init created duplicates");
            module.showColliders(true,scene.ordinal);
            module.showColliders(false,scene.ordinal);
            Physics physics;ovphysx_create_args args=OVPHYSX_CREATE_ARGS_DEFAULT;
            check(ovphysx_create_instance(&args,&physics.handle));check(ovphysx_attach_ovstage(physics.handle,scene.stage,scene.ordinal));
            for(int i=0;i<60;++i) check(ovphysx_step_sync(physics.handle,1.0f/60));
            const float from[]={3.4f,0,3},direction[]={0,0,-1};
            const ovphysx_scene_query_hit_t *hits=nullptr;uint32_t count=0;
            check(ovphysx_raycast(physics.handle,from,direction,10,false,OVPHYSX_SCENE_QUERY_MODE_CLOSEST,&hits,&count));
            require(count==1,"Generated hidden capsule has no PhysX shape");
            ovphysx_string_t path{};const auto collider=hits[0].collision;
            check(ovphysx_scene_query_get_paths_from_ids(physics.handle,&collider,1,&path,1,&count));
            require(std::string(path.ptr,path.length).find("__FbikBoneColliders")!=std::string::npos,"Ray missed generated collider");
            void *shape=nullptr;
            check(ovphysx_get_physx_ptr(physics.handle,path,OVPHYSX_PHYSX_TYPE_SHAPE,&shape));
            require(shape && static_cast<physx::PxShape*>(shape)->getGeometry().getType()==physx::PxGeometryType::eCAPSULE,
                    "PhysX did not create a capsule body shape");
            auto output=ovphysx::utils::writeWorldTransformsToOvstage(physics.handle,scene.stage,++scene.ordinal);
            require(output.ok(),"Could not publish probe position");st::seal(scene.stage,scene.ordinal);
            auto state=st::read(scene.stage,scene.ordinal,{"omni:fabric:worldMatrix"});
            const auto y=state.at("/World/Probe").at("omni:fabric:worldMatrix").numbers[13];
            std::cout << "Probe contacting generated collider at y=" << y << '\n';
            require(y>.3 && y<2,"Dynamic probe did not collide with kinematic bone capsule");
        }
        {
            Scene scene;scene.load(root/"resources/scenes/basic_walk.usda");
            fbik::Module module;auto stats=module.init(scene.stage,scene.ordinal);
            require(stats.characters==1 && stats.skeletons==1 && stats.capsules==23 && stats.leaves==2 &&
                    stats.collapsedBones==52 && stats.joints==22 && stats.hinges==4,
                    "SOMA collider/joint count mismatch");
            const auto &names=module.characters().front().joints;
            require(std::count(names.begin(),names.end(),"Hips/LeftLeg")==1 &&
                    std::count(names.begin(),names.end(),"Hips/RightLeg")==1,"Missing SOMA thigh bones");
            for (const auto &pair:module.collisionExclusions())
                require(!((pair.bone0=="Hips/LeftLeg" && pair.bone1=="Hips/RightLeg") ||
                          (pair.bone1=="Hips/LeftLeg" && pair.bone0=="Hips/RightLeg")),
                        "SOMA thigh capsules must remain collision-enabled");
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
                    require(shape && static_cast<physx::PxShape*>(shape)->getGeometry().getType()==physx::PxGeometryType::eCAPSULE,
                            "SOMA bone is missing a native capsule shape");
                    ++cooked;
                }
                require(cooked==stats.capsules,"Not all SOMA capsules were created");
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
                    "physics:body0","physics:body1","physics:localPos0","physics:localPos1","physics:localRot0","physics:localRot1",
                    "physics:lowerLimit","physics:upperLimit","limit:rotX:physics:low","limit:rotX:physics:high","fbik:swingLimit"});
                namespace sk=viewer::skeleton;
                double maxError=0;
                size_t checkedHinges=0,checkedBalls=0;
                for(const auto &[path,prim]:state) if(!sk::get(prim,"physics:body0").paths.empty() &&
                                                       !sk::get(prim,"physics:body1").paths.empty()) {
                    const auto point=[&](const char *body,const char *position) {
                        const auto &p=prim.at(position).numbers;
                        return sk::world(state.at(prim.at(body).paths[0])).transform({float(p[0]),float(p[1]),float(p[2])});
                    };
                    maxError=std::max(maxError,double((point("physics:body0","physics:localPos0")-
                                                     point("physics:body1","physics:localPos1")).magnitude()));
                    const auto rotation=[&](const char *body,const char *local) {
                        const auto &v=prim.at(local).numbers; // ovstage quaternion order is x,y,z,w.
                        return MeshTools::Quat(sk::world(state.at(prim.at(body).paths[0])).A)*
                               MeshTools::Quat(float(v[0]),float(v[1]),float(v[2]),float(v[3]));
                    };
                    auto relative=rotation("physics:body0","physics:localRot0").getConjugate()*
                                  rotation("physics:body1","physics:localRot1");
                    relative.normalize();if(relative.w<0) relative=relative*-1;
                    const float twist=2*std::atan2(relative.x,relative.w)*180/Pi;
                    if(sk::get(prim,"usd-prim-type").token=="PhysicsRevoluteJoint") {
                        ++checkedHinges;
                        require(twist>=prim.at("physics:lowerLimit").numbers[0]-2 &&
                                twist<=prim.at("physics:upperLimit").numbers[0]+2,"SOMA hinge exceeded its limits");
                        require(std::sqrt(relative.y*relative.y+relative.z*relative.z)<.03,"SOMA hinge acquired swing");
                    }
                    else {
                        ++checkedBalls;
                        require(twist>=prim.at("limit:rotX:physics:low").numbers[0]-2 &&
                                twist<=prim.at("limit:rotX:physics:high").numbers[0]+2,"SOMA ball joint exceeded twist limit");
                        const float swing=std::acos(std::clamp(MeshTools::Mat33(relative).column0.x,-1.f,1.f))*180/Pi;
                        if(swing>prim.at("fbik:swingLimit").numbers[0]+2)
                            std::cerr << path << " swing=" << swing << " limit=" << prim.at("fbik:swingLimit").numbers[0] << '\n';
                        require(swing<=prim.at("fbik:swingLimit").numbers[0]+2,"SOMA ball joint exceeded swing cone");
                    }
                }
                require(checkedHinges==4 && checkedBalls==18,"SOMA angular-limit checks missed joints");
                std::cout << "SOMA ragdoll maximum attachment error: " << maxError << " m\n";
                require(maxError<.05,"SOMA ragdoll joints separated");
                physics.stop(scene.ordinal);
            }
            std::cout << "SOMA: " << stats.capsules << " capsules, " << stats.joints << " joints, "
                      << stats.collapsedBones << " collapsed bones, " << stats.leaves << " leaves omitted\n";
        }
        std::cout << "FBIK discovery, bindings, fitted and fallback capsules, kinematic collision, animation, skinning and repeated init passed.\n";
    }
    catch(const std::exception &e) { std::cerr << e.what() << '\n';code=1; }
    if(initialized) ovstage_shutdown();
    return code;
}
