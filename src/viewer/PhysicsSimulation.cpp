#include "PhysicsSimulation.h"
#include <ovphysx/experimental/OvStageOutput.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <stdexcept>

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
void writeLocal(ovstage_instance_t *s, ovstage_ordinal_t ordinal, const std::string &path, Matrix value)
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
        tensor.data = value.data();
        tensor.device = {kDLCPU, 0};
        tensor.ndim = 1;
        tensor.shape = &shape;
        tensor.dtype = {kDLFloat, 64, 16};
        ovstage_write_data_t data{};
        data.tensors = &tensor;
        data.tensor_count = 1;
        data.semantic = OVSTAGE_SEMANTIC_MATRIX;
        ovx_string_or_token_t attribute{};
        attribute.string = stage::str("omni:xform");
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
    const auto snapshot = stage::read(s, ordinal, {"omni:xform", "physics:rigidBodyEnabled"});
    for (const auto &[path, prim] : snapshot)
        if (flag(prim, "physics:rigidBodyEnabled"))
            impl->initial.emplace(path, matrix(prim, "omni:xform"));
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

void PhysicsSimulation::step(ovstage_ordinal_t &ordinal)
{
    check(ovphysx_step_sync(impl->handle, timeStep));
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
    ++ordinal;
    for (const auto &[path, initial] : impl->initial)
        writeLocal(impl->stage, ordinal, path, initial);
    for (const auto &[path, local] : impl->childLocals)
        writeLocal(impl->stage, ordinal, path, local);
    stage::seal(impl->stage, ordinal);
    stage::hierarchy(impl->stage, ordinal);
}
} // namespace viewer
