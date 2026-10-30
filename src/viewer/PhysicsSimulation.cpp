#include "PhysicsSimulation.h"
#include <ovphysx/experimental/OvStageOutput.hpp>
#include <extensions/PxD6Joint.h>
#include <PxRigidDynamic.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <stdexcept>
#include <sstream>
#include <iomanip>
#include <locale>
#include <set>

namespace viewer
{
namespace
{
void check(ovphysx_result_t result)
{
    if (result.status != OVPHYSX_API_SUCCESS)
    {
        const auto error = ovphysx_get_last_error();
        throw std::runtime_error("PhysX: " + std::string(error.ptr ? error.ptr : "", error.length));
    }
}
using Matrix = std::array<double, 16>;
const Matrix identity{1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1};
Matrix matrix(const stage::Prim &prim, const char *name)
{
    auto it = prim.find(name);
    if (it == prim.end() || it->second.numbers.size() != 16)
        return identity;
    Matrix result;
    std::copy_n(it->second.numbers.begin(), 16, result.begin());
    return result;
}
bool flag(const stage::Prim &prim, const char *name)
{
    auto it = prim.find(name);
    return it != prim.end() && !it->second.numbers.empty() && it->second.numbers[0] != 0;
}
physx::PxTransform rigidPose(const Matrix &m)
{
    for (double v:m) if (!std::isfinite(v)) throw std::runtime_error("Invalid animation target.");
    const physx::PxVec3 x{float(m[0]),float(m[1]),float(m[2])};
    const physx::PxVec3 y{float(m[4]),float(m[5]),float(m[6])};
    const physx::PxVec3 z{float(m[8]),float(m[9]),float(m[10])};
    if (std::min({x.magnitudeSquared(),y.magnitudeSquared(),z.magnitudeSquared()})<1e-12f)
        throw std::runtime_error("Degenerate animation target.");
    return {physx::PxVec3(float(m[12]),float(m[13]),float(m[14])),
            physx::PxQuat(physx::PxMat33(x.getNormalized(),y.getNormalized(),z.getNormalized())).getNormalized()};
}
Matrix multiply(const Matrix &a, const Matrix &b)
{
    Matrix result{};
    for (int r=0; r<4; ++r)
        for (int c=0; c<4; ++c)
            for (int k=0; k<4; ++k)
                result[4*r+c] += a[4*r+k]*b[4*k+c];
    return result;
}
Matrix inverse(Matrix a)
{
    Matrix result = identity;
    for (int c=0; c<4; ++c)
    {
        int pivot = c;
        for (int r=c+1; r<4; ++r)
            if (std::abs(a[4*r+c]) > std::abs(a[4*pivot+c])) pivot = r;
        if (std::abs(a[4*pivot+c]) < 1e-12)
            throw std::runtime_error("Cannot simulate a body under a singular parent transform.");
        for (int k=0; k<4; ++k)
        {
            std::swap(a[4*c+k], a[4*pivot+k]);
            std::swap(result[4*c+k], result[4*pivot+k]);
        }
        const double scale = a[4*c+c];
        for (int k=0; k<4; ++k) { a[4*c+k] /= scale; result[4*c+k] /= scale; }
        for (int r=0; r<4; ++r)
            if (r != c)
            {
                const double factor = a[4*r+c];
                for (int k=0; k<4; ++k)
                {
                    a[4*r+k] -= factor*a[4*c+k];
                    result[4*r+k] -= factor*result[4*c+k];
                }
            }
    }
    return result;
}
void populationWait(ovstage_instance_t *s, ovstage_population_enqueue_result_t op)
{
    stage::check(op.status);
    ovstage_population_op_wait_result_t result{};
    stage::check(ovstage_population_wait_op(s, op.op_index, OVSTAGE_TIMEOUT_INFINITE, &result));
    if (result.error_op_id_count)
    {
        const auto error = ovstage_population_get_last_op_error(result.error_op_ids[0]);
        throw std::runtime_error(std::string(error.ptr, error.length));
    }
}
Vec3 normalized(Vec3 value)
{
    const auto length = value.magnitude();
    if (!std::isfinite(length) || length < 1e-8f)
        throw std::runtime_error("Invalid picking ray.");
    return value / length;
}
void writeAttribute(ovstage_instance_t *s, ovstage_ordinal_t ordinal, const std::string &path, const char *name, void *value, DLDataType dtype, ovstage_attribute_semantic_t semantic)
{
    ovx_string_t pathValue = stage::str(path.c_str());
    ovstage_predicate_t predicate{};
    predicate.attribute.string = stage::str("usd-path");
    predicate.op = OVSTAGE_FILTER_OP_IN;
    predicate.values = &pathValue;
    predicate.value_count = 1;
    ovstage_filter_t filter{&predicate, 1};
    ovstage_query_handle_t query{};
    stage::wait(s, ovstage_query(s, &filter, nullptr, 0, &query));
    try
    {
        int64_t shape = 1;
        DLTensor tensor{};
        tensor.data = value;
        tensor.device = {kDLCPU, 0};
        tensor.ndim = 1;
        tensor.shape = &shape;
        tensor.dtype = dtype;
        ovstage_write_data_t data{};
        data.tensors = &tensor;
        data.tensor_count = 1;
        data.semantic = semantic;
        ovx_string_or_token_t attribute{};
        attribute.string = stage::str(name);
        stage::wait(s, ovstage_write_attribute(s, query, attribute, ordinal, data, OVSTAGE_PRIM_MODE_UPSERT));
    }
    catch (...) { ovstage_release_query(s, query); throw; }
    ovstage_release_query(s, query);
}
} // namespace

PhysicsRuntime::PhysicsRuntime()
{
    check(ovphysx_initialize());
    try
    {
        ovphysx_string_t root{};
        check(ovphysx_get_codeless_schema_root(&root));
        const ovx_string_t path{root.ptr, root.length};
        stage::check(ovstage_population_register_usd_schemas(&path, 1));
    }
    catch (...) { ovphysx_shutdown(); throw; }
}
PhysicsRuntime::~PhysicsRuntime() { ovphysx_shutdown(); }

struct PhysicsSimulation::Impl
{
    ovstage_instance_t *stage;
    ovphysx_handle_t handle = OVPHYSX_INVALID_HANDLE;
    std::unique_ptr<ovphysx::utils::OvStageOutputCache> cache;
    std::map<std::string, Matrix> initial;
    std::map<std::string, Matrix> childLocals;
    std::set<std::string> kinematic;
    std::set<std::string> quasiStatic;
    std::map<std::string,float> swingCones;
    struct FollowDrive {
        std::string joint;
        physx::PxTransform target{physx::PxIdentity}, previous{physx::PxIdentity};
        bool enabled = false;
    };
    std::map<std::string,FollowDrive> followDrives;
    fbik::FollowSettings followSettings;
    stage::Query targetsQuery, localsQuery, transformsQuery;
    std::vector<std::string> targetPaths;
    std::vector<Matrix> targetLocals, targetWorlds, localTransforms;
    std::string jointPath;
    ovstage_population_usd_reference_handle_t jointLayer{};
    bool hasJointLayer = false, activeDrag = false, stepped = false, pendingOutput = false;
    float dragDepth = 0;
    Vec3 target{0, 0, 0};
    void refreshFollow(const viewer::stage::Snapshot &state)
    {
        for (auto &[body,drive]:followDrives) {
            const bool enabled=flag(state.at(drive.joint),"physics:jointEnabled");
            if (enabled && !drive.enabled) {
                drive.target=rigidPose(matrix(state.at(body),"omni:fabric:worldMatrix"));
                drive.previous=drive.target;
            }
            drive.enabled=enabled;
        }
    }
    void prepareFollow(ovstage_ordinal_t ordinal)
    {
        // Even a one-bone character needs native output before a seek can change
        // its animated parent transform without advancing the simulation.
        if (!stepped && std::any_of(followDrives.begin(),followDrives.end(),
                                   [](const auto &entry) { return entry.second.enabled; }))
            realizeJoints(ordinal);
    }
    void driveAnimation(ovstage_ordinal_t ordinal)
    {
        for (auto &[path,drive]:followDrives) {
            if (!drive.enabled) continue;
            if (!stepped) realizeJoints(ordinal);
            // Population edits (including mouse grabbing) can recreate native joints.
            // Reapply settings and targets to the current joint before every tick.
            void *pointer=nullptr;
            check(ovphysx_get_physx_ptr(handle,{drive.joint.c_str(),drive.joint.size()},OVPHYSX_PHYSX_TYPE_JOINT,&pointer));
            auto *joint=pointer ? static_cast<physx::PxJoint*>(pointer)->is<physx::PxD6Joint>() : nullptr;
            if (!joint) throw std::runtime_error("Expected an animation D6 joint: "+drive.joint);
            for (int axis=0;axis<physx::PxD6Axis::eCOUNT;++axis)
                joint->setMotion(physx::PxD6Axis::Enum(axis),physx::PxD6Motion::eFREE);
            joint->setLocalPose(physx::PxJointActorIndex::eACTOR0,physx::PxTransform(physx::PxIdentity));
            joint->setLocalPose(physx::PxJointActorIndex::eACTOR1,physx::PxTransform(physx::PxIdentity));
            const physx::PxD6JointDrive spring(followSettings.stiffness,followSettings.damping,PX_MAX_F32,true);
            for (const auto axis:{physx::PxD6Drive::eX,physx::PxD6Drive::eY,physx::PxD6Drive::eZ})
                joint->setDrive(axis,spring);
            if (joint->getAngularDriveConfig()!=physx::PxD6AngularDriveConfig::eSLERP)
                joint->setAngularDriveConfig(physx::PxD6AngularDriveConfig::eSLERP);
            joint->setDrive(physx::PxD6Drive::eSLERP,spring);
            // Feed forward sampled animation velocity to reduce tracking lag. Held
            // animation naturally has zero target velocity on the next physics tick.
            auto delta=drive.target.q*drive.previous.q.getConjugate();
            if (delta.w<0) delta=-delta;
            float angle=0;physx::PxVec3 axis(0);
            delta.toRadiansAndUnitAxis(angle,axis);
            joint->setDrivePosition(drive.target);
            joint->setDriveVelocity((drive.target.p-drive.previous.p)/PhysicsSimulation::timeStep,
                                    axis*(angle/PhysicsSimulation::timeStep));
            drive.previous=drive.target;
        }
    }
    void realizeJoints(ovstage_ordinal_t ordinal)
    {
        // ovphysx requires a warmup to expose native joints. At 1 ns, existing
        // contact/limit errors can generate enormous correction velocities. Keep
        // the authored starting state, then restore it before the first real tick.
        const auto state=viewer::stage::read(stage,ordinal,{"omni:fabric:worldMatrix",
            "physics:velocity","physics:angularVelocity","physics:startsAsleep"});
        check(ovphysx_step_sync(handle,1e-9f));
        for (const auto &[path,local]:initial) {
            void *pointer=nullptr;
            check(ovphysx_get_physx_ptr(handle,{path.c_str(),path.size()},OVPHYSX_PHYSX_TYPE_ACTOR,&pointer));
            auto *body=pointer ? static_cast<physx::PxRigidActor*>(pointer)->is<physx::PxRigidDynamic>() : nullptr;
            if (!body) continue;
            const auto &prim=state.at(path);
            const auto m=matrix(prim,"omni:fabric:worldMatrix");
            const physx::PxMat33 rotation(
                physx::PxVec3(float(m[0]),float(m[1]),float(m[2])).getNormalized(),
                physx::PxVec3(float(m[4]),float(m[5]),float(m[6])).getNormalized(),
                physx::PxVec3(float(m[8]),float(m[9]),float(m[10])).getNormalized());
            body->setGlobalPose(physx::PxTransform(physx::PxVec3(float(m[12]),float(m[13]),float(m[14])),
                                                 physx::PxQuat(rotation).getNormalized()));
            if (body->getRigidBodyFlags() & physx::PxRigidBodyFlag::eKINEMATIC) continue;
            const auto velocity=[&](const char *name) {
                const auto it=prim.find(name);
                if (it==prim.end() || it->second.numbers.empty()) return physx::PxVec3(0);
                const auto &v=it->second.numbers;
                if (v.size()!=3) throw std::runtime_error("Invalid initial velocity: "+path);
                return physx::PxVec3(float(v[0]),float(v[1]),float(v[2]));
            };
            body->setLinearVelocity(velocity("physics:velocity"));
            body->setAngularVelocity(velocity("physics:angularVelocity")*(physx::PxPi/180));
            if (flag(prim,"physics:startsAsleep")) body->putToSleep();
        }
        stepped=true;
    }
    void configureCones(ovstage_ordinal_t ordinal)
    {
        if (swingCones.empty()) return;
        const auto state=viewer::stage::read(stage,ordinal,{"physics:jointEnabled"});
        for (const auto &[path,angle]:swingCones) {
            // ovphysx does not instantiate disabled joints in Kinematic mode.
            if (!state.count(path) || !flag(state.at(path),"physics:jointEnabled")) continue;
            if (!stepped) realizeJoints(ordinal);
            void *pointer=nullptr;
            check(ovphysx_get_physx_ptr(handle,{path.c_str(),path.size()},OVPHYSX_PHYSX_TYPE_JOINT,&pointer));
            auto *joint=pointer ? static_cast<physx::PxJoint*>(pointer)->is<physx::PxD6Joint>() : nullptr;
            if (!joint) throw std::runtime_error("Expected a D6 joint for swing cone: "+path);
            joint->setSwingLimit(physx::PxJointLimitCone(angle,angle));
        }
    }
    std::vector<physx::PxRigidDynamic*> quasiStaticBodies(ovstage_ordinal_t ordinal)
    {
        // Native pointers are borrowed for this tick only: population/control
        // changes can recreate actors between ticks.
        if (!quasiStatic.empty() && !stepped) realizeJoints(ordinal);
        std::vector<physx::PxRigidDynamic*> bodies;
        bodies.reserve(quasiStatic.size());
        for (const auto &path:quasiStatic) {
            void *pointer=nullptr;
            check(ovphysx_get_physx_ptr(handle,{path.c_str(),path.size()},OVPHYSX_PHYSX_TYPE_ACTOR,&pointer));
            auto *body=pointer ? static_cast<physx::PxRigidActor*>(pointer)->is<physx::PxRigidDynamic>() : nullptr;
            if (!body || (body->getRigidBodyFlags() & physx::PxRigidBodyFlag::eKINEMATIC))
                throw std::runtime_error("Expected a dynamic quasi static body: "+path);
            bodies.push_back(body);
        }
        return bodies;
    }
    void drain(ovstage_ordinal_t ordinal, bool refreshCones=false)
    {
        stage::seal(stage, ordinal);
        ovstage_ordinal_range_t range{};
        range.end_ordinal = ordinal; // Single input batch; never includes physics output.
        check(ovphysx_update_from_ovstage(handle, range));
        // Control/layer edits may recreate joints. Reacquire pointers instead of
        // retaining native objects across population changes.
        if (refreshCones) configureCones(ordinal);
    }
    explicit Impl(ovstage_instance_t *s) : stage(s) {}
    void destroy()
    {
        cache.reset();
        if (handle != OVPHYSX_INVALID_HANDLE)
        {
            ovphysx_detach_ovstage(handle);
            ovphysx_destroy_instance(handle);
            handle = OVPHYSX_INVALID_HANDLE;
        }
    }
    ~Impl() { destroy(); }
};

PhysicsSimulation::PhysicsSimulation(ovstage_instance_t *s, ovstage_ordinal_t ordinal) : impl(std::make_unique<Impl>(s))
{
    const auto snapshot = stage::read(s, ordinal, {"omni:xform", "omni:fabric:worldMatrix", "physics:rigidBodyEnabled",
        "physics:kinematicEnabled", "fbik:swingLimit", "fbik:quasiStatic", "fbik:followBody", "physics:jointEnabled"});
    for (const auto &[path,prim]:snapshot) if (const auto it=prim.find("fbik:followBody");it!=prim.end()) {
        if (it->second.paths.size()!=1) throw std::runtime_error("Invalid follow joint: "+path);
        Impl::FollowDrive drive;drive.joint=path;
        impl->followDrives.emplace(it->second.paths[0],drive);
    }
    impl->refreshFollow(snapshot);
    for (const auto &[path,prim]:snapshot) if (const auto it=prim.find("fbik:swingLimit");it!=prim.end()) {
        const auto &values=it->second.numbers;
        if (values.size()!=1 || !std::isfinite(values[0]) || values[0]<=0 || values[0]>=180)
            throw std::runtime_error("Invalid swing cone: "+path);
        impl->swingCones.emplace(path,float(values[0])*physx::PxPi/180);
    }
    for (const auto &[path, prim] : snapshot)
        if (flag(prim, "physics:rigidBodyEnabled"))
            impl->initial.emplace(path, matrix(prim, "omni:xform"));
    for (const auto &[path, prim] : snapshot)
        if (flag(prim, "physics:kinematicEnabled")) impl->kinematic.insert(path);
    for (const auto &[path, prim] : snapshot)
        if (flag(prim, "fbik:quasiStatic")) impl->quasiStatic.insert(path);
    impl->jointPath = "/__FbikMouseJoint";
    const auto prims = stage::read(s, ordinal, {"usd-prim-type"});
    while (prims.count(impl->jointPath)) impl->jointPath += "_";
    // The renderer may reconcile local child transforms from world-matrix edits.
    // Keep authored rigid attachments under our ownership for the duration of a run.
    for (const auto &[path, prim] : snapshot)
        if (!impl->initial.count(path) && prim.count("omni:xform"))
            for (const auto &[body, transform] : impl->initial)
                if (path.rfind(body + "/", 0) == 0)
                { impl->childLocals.emplace(path, matrix(prim, "omni:xform")); break; }
    if (impl->initial.empty())
        throw std::runtime_error("This USD contains no enabled rigid bodies to simulate.");
    std::vector<std::string> localPaths;
    std::set<std::string> readPaths;
    for (const auto &[path, value] : impl->initial) {
        localPaths.push_back(path);
        for (auto ancestor = path; !ancestor.empty(); ancestor = ancestor.substr(0, ancestor.find_last_of('/')))
            if (snapshot.count(ancestor)) readPaths.insert(ancestor);
    }
    for (const auto &[path, value] : impl->childLocals) localPaths.push_back(path);
    impl->localsQuery.init(s, localPaths);
    impl->transformsQuery.init(s, {readPaths.begin(), readPaths.end()},
                              {"omni:xform", "omni:fabric:worldMatrix", "omni:resetXformStack"});
    impl->localTransforms.reserve(localPaths.size());
    ovphysx_create_args args = OVPHYSX_CREATE_ARGS_DEFAULT;
    check(ovphysx_create_instance(&args, &impl->handle));
    check(ovphysx_attach_ovstage(impl->handle, s, ordinal));
    impl->configureCones(ordinal);
    impl->prepareFollow(ordinal);
    impl->cache = std::make_unique<ovphysx::utils::OvStageOutputCache>();
}
PhysicsSimulation::~PhysicsSimulation() = default;

void PhysicsSimulation::applyControlChanges(ovstage_ordinal_t ordinal)
{
    impl->drain(ordinal,true);
    const auto snapshot=stage::read(impl->stage,ordinal,{"physics:kinematicEnabled","fbik:quasiStatic",
        "physics:jointEnabled","omni:fabric:worldMatrix"});
    impl->refreshFollow(snapshot);
    impl->prepareFollow(ordinal);
    impl->kinematic.clear();
    impl->quasiStatic.clear();
    for (const auto &[path,prim]:snapshot) {
        if (flag(prim,"physics:kinematicEnabled")) impl->kinematic.insert(path);
        if (flag(prim,"fbik:quasiStatic")) impl->quasiStatic.insert(path);
    }
}

void PhysicsSimulation::setKinematicTargets(const std::vector<fbik::BodyTarget> &targets, ovstage_ordinal_t &ordinal)
{
    if (targets.empty()) return;
    for (const auto &t : targets) {
        if (!impl->kinematic.count(t.path)) throw std::runtime_error("Target is not a kinematic body: "+t.path);
        for (double v : t.world) if (!std::isfinite(v)) throw std::runtime_error("Invalid kinematic target.");
        for (double v : t.local) if (!std::isfinite(v)) throw std::runtime_error("Invalid local target.");
    }
    std::vector<std::string> paths;
    paths.reserve(targets.size());
    for (const auto &t : targets) paths.push_back(t.path);
    if (paths != impl->targetPaths) {
        impl->targetsQuery.init(impl->stage, paths);
        impl->targetPaths = std::move(paths);
    }
    impl->targetLocals.clear(); impl->targetWorlds.clear();
    for (const auto &t : targets) {
        impl->targetLocals.push_back(t.local);
        impl->targetWorlds.push_back(t.world);
    }
    ++ordinal; // Only this control ordinal is drained; animation and physics output stay separate.
    impl->targetsQuery.writeTransforms(ordinal, impl->targetLocals, &impl->targetWorlds);
    impl->drain(ordinal); // CPU ovphysx converts transform edits to setKinematicTarget, not teleportation.
}

void PhysicsSimulation::setFollowTargets(const std::vector<fbik::BodyTarget> &targets, bool discontinuity)
{
    std::map<std::string,physx::PxTransform> poses;
    for (const auto &target:targets) {
        const auto it=impl->followDrives.find(target.path);
        if (it==impl->followDrives.end() || !it->second.enabled)
            throw std::runtime_error("Target is not a following body: "+target.path);
        if (!poses.emplace(target.path,rigidPose(target.world)).second)
            throw std::runtime_error("Duplicate follow target: "+target.path);
    }
    for (const auto &[path,pose]:poses) {
        auto &drive=impl->followDrives.at(path);
        drive.target=pose;
        if (discontinuity) drive.previous=pose;
    }
    // Sampling animation can move the stage hierarchy above a dynamic body.
    // Republish native poses before display, including seeks with no physics tick.
    if (!poses.empty()) impl->pendingOutput=true;
}
void PhysicsSimulation::setFollowSettings(const fbik::FollowSettings &settings)
{
    if (!std::isfinite(settings.stiffness) || settings.stiffness<0 ||
        !std::isfinite(settings.damping) || settings.damping<0)
        throw std::runtime_error("Follow stiffness and damping must be finite and nonnegative.");
    impl->followSettings=settings;
}

void PhysicsSimulation::step(ovstage_ordinal_t &ordinal, bool publishOutput)
{
    impl->driveAnimation(ordinal);
    const auto bodies=impl->quasiStaticBodies(ordinal);
    const auto clearVelocities=[&] {
        for (auto *body:bodies) {
            body->setLinearVelocity(physx::PxVec3(0),false);
            body->setAngularVelocity(physx::PxVec3(0),false);
        }
    };
    // Let constraints and contacts correct positions during the tick, but keep
    // neither incoming momentum nor solver-generated velocities between ticks.
    clearVelocities();
    check(ovphysx_step_sync(impl->handle, timeStep));
    clearVelocities();
    impl->stepped = true;
    impl->pendingOutput = true;
    if (publishOutput) publish(ordinal);
}

void PhysicsSimulation::publish(ovstage_ordinal_t &ordinal)
{
    if (!impl->pendingOutput) return;
    const auto output = ovphysx::utils::writeWorldTransformsToOvstage(impl->handle, impl->stage, ++ordinal, impl->cache.get());
    // Even partial output must be sealed before recovery writes.
    stage::seal(impl->stage, ordinal);
    if (!output.ok()) throw std::runtime_error("PhysX output: " + output.message);
    if (output.instancerGroupsSkipped)
        throw std::runtime_error("Simulation of point-instanced bodies is not supported by this viewer yet.");

    // Convert body world poses to local transforms so hierarchy updates propagate
    // them to child meshes and camera movement cannot overwrite simulation output.
    const auto snapshot = impl->transformsQuery.read(ordinal);
    std::map<std::string, Matrix> worlds;
    std::function<Matrix(const std::string &)> world = [&](const std::string &path) -> Matrix {
        if (path.empty() || path == "/") return identity;
        if (auto it = worlds.find(path); it != worlds.end()) return it->second;
        auto it = snapshot.find(path);
        Matrix value = world(path.substr(0, path.find_last_of('/')));
        if (it != snapshot.end())
        {
            if (impl->initial.count(path)) value = matrix(it->second, "omni:fabric:worldMatrix");
            else value = multiply(impl->childLocals.count(path) ? impl->childLocals.at(path) : matrix(it->second, "omni:xform"),
                                  flag(it->second, "omni:resetXformStack") ? identity : value);
        }
        return worlds[path] = value;
    };
    ++ordinal;
    impl->localTransforms.clear();
    for (const auto &[path, initial] : impl->initial)
    {
        const auto &prim = snapshot.at(path);
        const auto parent = flag(prim, "omni:resetXformStack") ? identity : world(path.substr(0, path.find_last_of('/')));
        impl->localTransforms.push_back(multiply(world(path), inverse(parent)));
    }
    for (const auto &[path, local] : impl->childLocals)
        impl->localTransforms.push_back(local);
    impl->localsQuery.writeTransforms(ordinal, impl->localTransforms);
    stage::seal(impl->stage, ordinal);
    stage::hierarchy(impl->stage, ordinal);
    impl->pendingOutput = false;
    // No output ordinal is ever passed to ovphysx_update_from_ovstage.
}

void PhysicsSimulation::stop(ovstage_ordinal_t &ordinal)
{
    impl->destroy();
    impl->activeDrag = false;
    if (impl->hasJointLayer)
    {
        populationWait(impl->stage, ovstage_population_remove_usd_reference(impl->stage, impl->jointLayer));
        impl->hasJointLayer = false;
        populationWait(impl->stage, ovstage_population_apply_usd_changes(impl->stage, ++ordinal));
        stage::seal(impl->stage, ordinal);
    }
    ++ordinal;
    impl->localTransforms.clear();
    for (const auto &[path, initial] : impl->initial)
        impl->localTransforms.push_back(initial);
    for (const auto &[path, local] : impl->childLocals)
        impl->localTransforms.push_back(local);
    impl->localsQuery.writeTransforms(ordinal, impl->localTransforms);
    impl->pendingOutput = false;
    stage::seal(impl->stage, ordinal);
    stage::hierarchy(impl->stage, ordinal);
}

bool PhysicsSimulation::beginDrag(Vec3 origin, Vec3 direction, ovstage_ordinal_t &ordinal)
{
    publish(ordinal);
    endDrag(ordinal);
    direction = normalized(direction);
    if (!std::isfinite(origin.x) || !std::isfinite(origin.y) || !std::isfinite(origin.z))
        throw std::runtime_error("Invalid picking ray origin.");
    if (!impl->stepped)
    {
        // ovphysx creates queryable objects on the first step. Warm up with the
        // same minimal timestep used by its tensor bindings, preserving the visible pose.
        check(ovphysx_step_sync(impl->handle, 1e-9f));
        impl->stepped = true;
    }
    const float from[] = {origin.x, origin.y, origin.z}, ray[] = {direction.x, direction.y, direction.z};
    const ovphysx_scene_query_hit_t *hits = nullptr;
    uint32_t count = 0;
    check(ovphysx_raycast(impl->handle, from, ray, 1e8f, false, OVPHYSX_SCENE_QUERY_MODE_CLOSEST, &hits, &count));
    if (!count) return false;
    const auto hit = hits[0];
    ovphysx_string_t path{};
    check(ovphysx_scene_query_get_paths_from_ids(impl->handle, &hit.rigid_body, 1, &path, 1, &count));
    const std::string body(path.ptr ? path.ptr : "", path.length);
    if (!impl->initial.count(body) || impl->kinematic.count(body) || hit.proto_index != 0xffffffffu) return false;
    const auto snapshot = stage::read(impl->stage, ordinal, {"omni:fabric:worldMatrix"});
    const auto inv = inverse(matrix(snapshot.at(body), "omni:fabric:worldMatrix"));
    float local[3]{};
    for (int c = 0; c < 3; ++c)
        local[c] = float(hit.position[0]*inv[c] + hit.position[1]*inv[4+c] + hit.position[2]*inv[8+c] + inv[12+c]);
    if (impl->hasJointLayer)
    {
        populationWait(impl->stage, ovstage_population_remove_usd_reference(impl->stage, impl->jointLayer));
        impl->hasJointLayer = false;
        populationWait(impl->stage, ovstage_population_apply_usd_changes(impl->stage, ++ordinal));
        impl->drain(ordinal,true);
    }
    std::ostringstream usd;
    usd.imbue(std::locale::classic());
    usd << std::setprecision(9) << "#usda 1.0\ndef PhysicsJoint \"" << impl->jointPath.substr(1)
        << "\" (prepend apiSchemas = [\"PhysicsDriveAPI:transX\", \"PhysicsDriveAPI:transY\", \"PhysicsDriveAPI:transZ\"])\n{\n"
        << "rel physics:body1 = <" << body << ">\n"
        << "point3f physics:localPos0 = (" << hit.position[0] << "," << hit.position[1] << "," << hit.position[2] << ")\n"
        << "point3f physics:localPos1 = (" << local[0] << "," << local[1] << "," << local[2] << ")\n"
        << "bool physics:jointEnabled = true\nbool physics:excludeFromArticulation = true\n";
    for (const char *axis : {"transX", "transY", "transZ"})
        usd << "uniform token drive:" << axis << ":physics:type = \"acceleration\"\n"
            << "float drive:" << axis << ":physics:stiffness = 400\n"
            << "float drive:" << axis << ":physics:damping = 40\n"
            << "float drive:" << axis << ":physics:maxForce = 1000\n"
            << "float drive:" << axis << ":physics:targetPosition = 0\n";
    usd << "}\n";
    const auto content = usd.str();
    populationWait(impl->stage, ovstage_population_add_usd_reference_from_string(impl->stage,
        stage::str(content.c_str()), stage::str("/"), &impl->jointLayer));
    impl->hasJointLayer = true;
    populationWait(impl->stage, ovstage_population_apply_usd_changes(impl->stage, ++ordinal));
    impl->drain(ordinal,true);
    impl->dragDepth = hit.distance;
    impl->target = Vec3(hit.position[0], hit.position[1], hit.position[2]);
    impl->activeDrag = true;
    return true;
}
void PhysicsSimulation::moveDrag(Vec3 origin, Vec3 direction, ovstage_ordinal_t &ordinal)
{
    if (!impl->activeDrag) return;
    const auto target = origin + normalized(direction) * impl->dragDepth;
    float position[] = {target.x, target.y, target.z};
    for (float value : position)
        if (!std::isfinite(value)) throw std::runtime_error("Invalid mouse joint target.");
    if ((target - impl->target).magnitudeSquared() < 1e-12f) return;
    writeAttribute(impl->stage, ++ordinal, impl->jointPath, "physics:localPos0", position,
                   {kDLFloat, 32, 3}, OVSTAGE_SEMANTIC_NONE);
    impl->drain(ordinal);
    impl->target = target;
}
void PhysicsSimulation::endDrag(ovstage_ordinal_t &ordinal)
{
    if (!impl->activeDrag) return;
    uint8_t enabled = 0;
    writeAttribute(impl->stage, ++ordinal, impl->jointPath, "physics:jointEnabled", &enabled,
                   {kDLBool, 8, 1}, OVSTAGE_SEMANTIC_NONE);
    impl->drain(ordinal);
    impl->activeDrag = false;
}
bool PhysicsSimulation::dragging() const { return impl->activeDrag; }
} // namespace viewer
