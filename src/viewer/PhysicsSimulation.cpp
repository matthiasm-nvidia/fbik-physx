#include "PhysicsSimulation.h"
#include <ovphysx/experimental/OvStageOutput.hpp>
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
void writeLocal(ovstage_instance_t *s, ovstage_ordinal_t ordinal, const std::string &path, Matrix value)
{
    writeAttribute(s, ordinal, path, "omni:xform", value.data(), {kDLFloat, 64, 16}, OVSTAGE_SEMANTIC_MATRIX);
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
    std::string jointPath;
    ovstage_population_usd_reference_handle_t jointLayer{};
    bool hasJointLayer = false, activeDrag = false, stepped = false;
    float dragDepth = 0;
    Vec3 target{0, 0, 0};
    void drain(ovstage_ordinal_t ordinal)
    {
        stage::seal(stage, ordinal);
        ovstage_ordinal_range_t range{};
        range.end_ordinal = ordinal; // Single input batch; never includes physics output.
        check(ovphysx_update_from_ovstage(handle, range));
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
    const auto snapshot = stage::read(s, ordinal, {"omni:xform", "physics:rigidBodyEnabled", "physics:kinematicEnabled"});
    for (const auto &[path, prim] : snapshot)
        if (flag(prim, "physics:rigidBodyEnabled"))
            impl->initial.emplace(path, matrix(prim, "omni:xform"));
    for (const auto &[path, prim] : snapshot)
        if (flag(prim, "physics:kinematicEnabled")) impl->kinematic.insert(path);
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
    ovphysx_create_args args = OVPHYSX_CREATE_ARGS_DEFAULT;
    check(ovphysx_create_instance(&args, &impl->handle));
    check(ovphysx_attach_ovstage(impl->handle, s, ordinal));
    impl->cache = std::make_unique<ovphysx::utils::OvStageOutputCache>();
}
PhysicsSimulation::~PhysicsSimulation() = default;

void PhysicsSimulation::setKinematicTargets(const std::vector<fbik::BodyTarget> &targets, ovstage_ordinal_t &ordinal)
{
    if (targets.empty()) return;
    for (const auto &t : targets) {
        if (!impl->kinematic.count(t.path)) throw std::runtime_error("Target is not a kinematic body: "+t.path);
        for (double v : t.world) if (!std::isfinite(v)) throw std::runtime_error("Invalid kinematic target.");
        for (double v : t.local) if (!std::isfinite(v)) throw std::runtime_error("Invalid local target.");
    }
    ++ordinal; // Only this control ordinal is drained; animation and physics output stay separate.
    for (const auto &t : targets) {
        writeLocal(impl->stage,ordinal,t.path,t.local);
        auto world=t.world;
        writeAttribute(impl->stage,ordinal,t.path,"omni:fabric:worldMatrix",world.data(),
                       {kDLFloat,64,16},OVSTAGE_SEMANTIC_MATRIX);
    }
    impl->drain(ordinal); // CPU ovphysx converts transform edits to setKinematicTarget, not teleportation.
}

void PhysicsSimulation::step(ovstage_ordinal_t &ordinal)
{
    check(ovphysx_step_sync(impl->handle, timeStep));
    impl->stepped = true;
    const auto output = ovphysx::utils::writeWorldTransformsToOvstage(impl->handle, impl->stage, ++ordinal, impl->cache.get());
    // Even partial output must be sealed before recovery writes.
    stage::seal(impl->stage, ordinal);
    if (!output.ok()) throw std::runtime_error("PhysX output: " + output.message);
    if (output.instancerGroupsSkipped)
        throw std::runtime_error("Simulation of point-instanced bodies is not supported by this viewer yet.");

    // Convert body world poses to local transforms so hierarchy updates propagate
    // them to child meshes and camera movement cannot overwrite simulation output.
    const auto snapshot = stage::read(impl->stage, ordinal,
        {"omni:xform", "omni:fabric:worldMatrix", "omni:resetXformStack"});
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
    for (const auto &[path, initial] : impl->initial)
    {
        const auto &prim = snapshot.at(path);
        const auto parent = flag(prim, "omni:resetXformStack") ? identity : world(path.substr(0, path.find_last_of('/')));
        writeLocal(impl->stage, ordinal, path, multiply(world(path), inverse(parent)));
    }
    for (const auto &[path, local] : impl->childLocals)
        writeLocal(impl->stage, ordinal, path, local);
    stage::seal(impl->stage, ordinal);
    stage::hierarchy(impl->stage, ordinal);
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
    for (const auto &[path, initial] : impl->initial)
        writeLocal(impl->stage, ordinal, path, initial);
    for (const auto &[path, local] : impl->childLocals)
        writeLocal(impl->stage, ordinal, path, local);
    stage::seal(impl->stage, ordinal);
    stage::hierarchy(impl->stage, ordinal);
}

bool PhysicsSimulation::beginDrag(Vec3 origin, Vec3 direction, ovstage_ordinal_t &ordinal)
{
    endDrag(ordinal);
    if (!impl->stepped) return false; // Scene-query objects exist after the first completed step.
    direction = normalized(direction);
    if (!std::isfinite(origin.x) || !std::isfinite(origin.y) || !std::isfinite(origin.z))
        throw std::runtime_error("Invalid picking ray origin.");
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
        impl->drain(ordinal);
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
    impl->drain(ordinal);
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
