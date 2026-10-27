#include "RtxView.h"
#include "StageCamera.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <functional>
#include <limits>
#include <ovrtx/ovrtx.h>
#include <ovrtx/ovrtx_config.h>
#include <ovstage/ovstage.h>
#include <ovx/path_dictionary/path_dictionary_utils.h>
#include <sstream>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>

namespace viewer
{
namespace
{
ovx_string_t str(const std::string &s)
{
    return {s.data(), s.size()};
}
ovx_string_t lit(const char *s)
{
    return {s, std::strlen(s)};
}
std::string text(ovx_string_t s)
{
    return s.ptr ? std::string(s.ptr, s.length) : std::string();
}

void stageCheck(ovstage_api_status_t status, const char *operation)
{
    if (status != OVSTAGE_OK)
        throw std::runtime_error(std::string(operation) + ": " + text(ovstage_get_last_error()));
}

void stageWait(ovstage_instance_t *stage, ovstage_enqueue_result_t op)
{
    stageCheck(op.status, "Stage operation");
    ovstage_op_wait_result_t result{};
    const auto status = ovstage_wait_op(stage, op.op_index, OVSTAGE_TIMEOUT_INFINITE, &result);
    std::string error;
    for (size_t i = 0; i < result.error_op_id_count; ++i)
        error += text(ovstage_get_last_op_error(stage, result.error_op_ids[i])) + "\n";
    ovstage_release_op(stage, op.op_index);
    stageCheck(status, "Waiting for stage");
    if (!error.empty())
        throw std::runtime_error(error);
}

void populationWait(ovstage_instance_t *stage, ovstage_population_enqueue_result_t op)
{
    if (op.status != OVSTAGE_OK)
        throw std::runtime_error("USD load (status " + std::to_string(op.status) +
                                 "): " + text(ovstage_population_get_last_error()));
    ovstage_population_op_wait_result_t result{};
    const auto status = ovstage_population_wait_op(stage, op.op_index, OVSTAGE_TIMEOUT_INFINITE, &result);
    std::string error;
    for (size_t i = 0; i < result.error_op_id_count; ++i)
        error += text(ovstage_population_get_last_op_error(result.error_op_ids[i])) + "\n";
    if (status != OVSTAGE_OK)
        throw std::runtime_error("USD load wait (status " + std::to_string(status) + "): " + error +
                                 text(ovstage_population_get_last_error()));
    if (!error.empty())
        throw std::runtime_error(error);
}

template <class Result> void rtxCheck(Result result, const char *operation)
{
    if (result.status != OVRTX_API_SUCCESS)
        throw std::runtime_error(std::string(operation) + ": " + text(ovrtx_get_last_error()));
}

void rtxWait(ovrtx_renderer_t *renderer, ovrtx_enqueue_result_t op)
{
    rtxCheck(op, "RTX operation");
    ovrtx_op_wait_result_t result{};
    rtxCheck(ovrtx_wait_op(renderer, op.op_index, ovrtx_timeout_infinite, &result), "Waiting for RTX");
    std::string error;
    for (size_t i = 0; i < result.num_error_ops; ++i)
        error += text(ovrtx_get_last_op_error(result.error_op_ids[i])) + "\n";
    if (!error.empty())
        throw std::runtime_error(error);
}

struct Scene
{
    ovstage_instance_t *stage = nullptr;
    ovstage_query_handle_t camera = OVSTAGE_INVALID_QUERY_HANDLE;
    ovstage_query_handle_t product = OVSTAGE_INVALID_QUERY_HANDLE;
    std::string viewerRoot = "/__FbikPhysxViewer";
    ovstage_ordinal_t ordinal = 1;
    Vec3 center{0, 0, 0};
    float radius = 5.0f;
    bool zUp = false;
    std::optional<StageCamera> initialCamera;
    std::string path;
    ~Scene()
    {
        if (stage)
        {
            // Destruction completes pending operations and releases stage-owned queries.
            ovstage_destroy_instance(stage);
        }
    }
    void seal()
    {
        ovstage_write_floor_desc_t floor{};
        floor.ordinal = ordinal;
        floor.scope = OVSTAGE_SCOPE_ALL;
        stageWait(stage, ovstage_advance_write_floor(stage, &floor));
    }
    ovstage_query_handle_t query(const char *path)
    {
        ovx_string_t value = lit(path);
        ovstage_predicate_t predicate{};
        predicate.attribute.string = lit("usd-path");
        predicate.op = OVSTAGE_FILTER_OP_IN;
        predicate.values = &value;
        predicate.value_count = 1;
        ovstage_filter_t filter{&predicate, 1};
        ovstage_query_handle_t handle{};
        stageWait(stage, ovstage_query(stage, &filter, nullptr, 0, &handle));
        ovstage_query_result_t result{};
        stageCheck(ovstage_fetch_query_result(stage, handle, OVSTAGE_TIMEOUT_INFINITE, &result), "Query camera");
        const size_t count = result.total_prim_count;
        ovstage_release_query_result(stage, &result);
        if (count != 1)
            throw std::runtime_error(std::string("Expected exactly one prim: ") + path);
        return handle;
    }
    void write(ovstage_query_handle_t query, const char *name, void *data, uint8_t code, uint8_t bits, uint16_t lanes,
               ovstage_attribute_semantic_t semantic = OVSTAGE_SEMANTIC_NONE, int64_t arraySize = -1)
    {
        int64_t shape = arraySize < 0 ? 1 : arraySize;
        DLTensor tensor{};
        tensor.data = data;
        tensor.device = {kDLCPU, 0};
        tensor.ndim = 1;
        tensor.shape = &shape;
        tensor.dtype = {code, bits, lanes};
        ovstage_write_data_t payload{};
        payload.tensors = &tensor;
        payload.tensor_count = 1;
        payload.semantic = semantic;
        payload.is_array = arraySize >= 0;
        ovx_string_or_token_t attribute{};
        attribute.string = lit(name);
        stageWait(stage, ovstage_write_attribute(stage, query, attribute, ordinal, payload, OVSTAGE_PRIM_MODE_UPSERT));
    }
};

// Scalar access honors the tensor byte offset and element stride.
double scalar(const DLTensor &tensor, size_t row, size_t lane)
{
    const auto *bytes = static_cast<const uint8_t *>(tensor.data) + tensor.byte_offset;
    const size_t stride = tensor.strides ? static_cast<size_t>(tensor.strides[0]) : 1;
    const size_t offset = (row * stride * tensor.dtype.lanes + lane) * (tensor.dtype.bits / 8);
    if (tensor.dtype.code == kDLFloat && tensor.dtype.bits == 32)
    {
        float value;
        std::memcpy(&value, bytes + offset, sizeof(value));
        return value;
    }
    if (tensor.dtype.code == kDLFloat && tensor.dtype.bits == 64)
    {
        double value;
        std::memcpy(&value, bytes + offset, sizeof(value));
        return value;
    }
    return std::numeric_limits<double>::quiet_NaN();
}

void frameBounds(Scene &scene)
{
    struct Geometry
    {
        std::vector<double> extent;
        std::vector<double> worldExtent;
        std::array<double, 16> matrix{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
    };
    std::unordered_map<ovx_primpath_t, Geometry> geometries;
    ovstage_query_handle_t query{};
    stageWait(scene.stage, ovstage_query(scene.stage, nullptr, nullptr, 0, &query));
    ovstage_query_result_t info{};
    stageCheck(ovstage_fetch_query_result(scene.stage, query, OVSTAGE_TIMEOUT_INFINITE, &info),
               "Discover stage attributes");
    auto *dictionary = ovstage_get_path_dictionary(scene.stage);
    std::vector<ovx_string_t> names(info.attribute_count);
    path_dictionary_get_strings_from_tokens(dictionary, info.attributes, info.attribute_count, names.data());
    std::unordered_map<ovx_token_t, std::string> selectedNames;
    std::vector<ovx_token_t> attributes;
    for (size_t i = 0; i < names.size(); ++i)
    {
        const auto name = text(names[i]);
        if (name == "extent" || name == "omni:fabric:worldMatrix" || name == "omni:fabric:worldExtent" ||
            name == "usd-metadata:upAxis")
        {
            attributes.push_back(info.attributes[i]);
            selectedNames.emplace(info.attributes[i], name);
        }
    }
    ovstage_release_query_result(scene.stage, &info);
    if (attributes.empty())
    {
        stageWait(scene.stage, ovstage_release_query(scene.stage, query));
        return;
    }
    ovstage_read_handle_t read{};
    ovstage_ordinal_range_t range{};
    range.end_ordinal = scene.ordinal;
    stageWait(scene.stage,
              ovstage_read_attributes(scene.stage, query, attributes.data(), attributes.size(), range, &read));
    try
    {
        while (true)
        {
            ovstage_read_group_t group{};
            const auto status = ovstage_fetch_read_next(scene.stage, read, OVSTAGE_TIMEOUT_INFINITE, &group);
            if (status == OVSTAGE_ERROR_END_OF_ITERATION)
                break;
            stageCheck(status, "Read bounds");
            if (!group.is_delete && group.data.tensor_count > 0)
            {
                size_t count = 0;
                path_dictionary_get_num_paths_from_path_list(dictionary, group.prims.list, &count);
                std::vector<ovx_primpath_t> paths(count);
                path_dictionary_get_paths_from_path_list(dictionary, group.prims.list, 0, count, paths.data(), &count);
                for (uint32_t i = 0; i < group.prims.count; ++i)
                {
                    if (group.data.mask && !(group.data.mask[i / 64] & (uint64_t(1) << (i % 64))))
                        continue;
                    const size_t pathIndex = group.prims.index_map ? group.prims.index_map[i] : group.prims.offset + i;
                    const size_t row = group.data.index_map ? group.data.index_map[i] : i;
                    if (pathIndex >= paths.size())
                        continue;
                    if (group.is_array && row >= group.data.tensor_count)
                        continue;
                    const DLTensor &tensor = group.data.tensors[group.is_array ? row : 0];
                    if (tensor.device.device_type != kDLCPU || !tensor.data || !tensor.shape || tensor.ndim != 1)
                        continue;
                    const auto &name = selectedNames.at(group.attribute);
                    if (name == "usd-metadata:upAxis" && tensor.dtype.code == kDLUInt && tensor.dtype.bits == 64)
                    {
                        ovx_token_t token{};
                        std::memcpy(&token,
                                    static_cast<const uint8_t *>(tensor.data) + tensor.byte_offset +
                                        row * sizeof(token),
                                    sizeof(token));
                        ovx_string_t value{};
                        path_dictionary_get_strings_from_tokens(dictionary, &token, 1, &value);
                        scene.zUp = text(value) == "Z";
                        continue;
                    }
                    if (tensor.dtype.code != kDLFloat)
                        continue;
                    const size_t valueCount =
                        group.is_array ? tensor.shape[0] * tensor.dtype.lanes : tensor.dtype.lanes;
                    std::vector<double> values(valueCount);
                    for (size_t k = 0; k < valueCount; ++k)
                        values[k] =
                            scalar(tensor, group.is_array ? k / tensor.dtype.lanes : row, k % tensor.dtype.lanes);
                    auto &geometry = geometries[paths[pathIndex]];
                    if (name == "extent" && values.size() == 6)
                        geometry.extent = values;
                    if (name == "omni:fabric:worldExtent" && values.size() == 6)
                        geometry.worldExtent = values;
                    if (name == "omni:fabric:worldMatrix" && values.size() == 16)
                        std::copy(values.begin(), values.end(), geometry.matrix.begin());
                }
            }
            ovstage_release_group(scene.stage, &group);
        }
    }
    catch (...)
    {
        stageWait(scene.stage, ovstage_release_read(scene.stage, read));
        stageWait(scene.stage, ovstage_release_query(scene.stage, query));
        throw;
    }
    stageWait(scene.stage, ovstage_release_read(scene.stage, read));
    stageWait(scene.stage, ovstage_release_query(scene.stage, query));
    double minimum[3] = {INFINITY, INFINITY, INFINITY};
    double maximum[3] = {-INFINITY, -INFINITY, -INFINITY};
    for (const auto &entry : geometries)
    {
        const Geometry &geometry = entry.second;
        const bool world = !geometry.worldExtent.empty();
        const auto &extent = world ? geometry.worldExtent : geometry.extent;
        if (extent.empty())
            continue;
        bool valid = true;
        for (int axis = 0; axis < 3; ++axis)
            valid &= std::isfinite(extent[axis]) && std::isfinite(extent[axis + 3]) && extent[axis] <= extent[axis + 3];
        if (!valid)
            continue;
        for (int corner = 0; corner < 8; ++corner)
        {
            double p[3] = {extent[(corner & 1) ? 3 : 0], extent[(corner & 2) ? 4 : 1], extent[(corner & 4) ? 5 : 2]};
            for (int axis = 0; axis < 3; ++axis)
            {
                const auto &m = geometry.matrix;
                const double value =
                    world ? p[axis] : p[0] * m[axis] + p[1] * m[4 + axis] + p[2] * m[8 + axis] + m[12 + axis];
                minimum[axis] = std::min(minimum[axis], value);
                maximum[axis] = std::max(maximum[axis], value);
            }
        }
    }
    if (std::isfinite(minimum[0]) && std::isfinite(maximum[0]))
    {
        double diagonal = 0;
        for (int axis = 0; axis < 3; ++axis)
        {
            scene.center[axis] = static_cast<float>((minimum[axis] + maximum[axis]) * 0.5);
            diagonal += (maximum[axis] - minimum[axis]) * (maximum[axis] - minimum[axis]);
        }
        scene.radius = std::max(0.001f, static_cast<float>(std::sqrt(diagonal) * 0.5));
    }
    std::printf("Scene framing: center=(%.3f, %.3f, %.3f), radius=%.3f\n", scene.center.x, scene.center.y,
                scene.center.z, scene.radius);
}

std::string viewerLayer(const std::string &root, int width, int height)
{
    std::ostringstream usd;
    usd << "#usda 1.0\ndef Scope \"" << root.substr(1) << "\"\n{\n"
        << R"USD(    def Camera "Camera"
    {
        float horizontalAperture = 20.955
        float verticalAperture = 15.2908
        float focalLength = 20
        float2 clippingRange = (0.001, 1000000000)
        matrix4d xformOp:transform = ((1,0,0,0),(0,1,0,0),(0,0,1,0),(6,4,6,1))
        uniform token[] xformOpOrder = ["xformOp:transform"]
    }
    def DomeLight "StudioLight"
    {
        float inputs:intensity = 500
        color3f inputs:color = (0.8, 0.85, 1)
    }
    def RenderProduct "View" (
        prepend apiSchemas = ["OmniRtxSettingsCommonAPI_1"]
    )
    {
)USD"
        << "        rel camera = <" << root << "/Camera>\n"
        << "        rel orderedVars = <" << root << "/Color>\n"
        << "        token omni:rtx:rendermode = \"Real-Time Path-Tracing\"\n"
        << "        int2 resolution = (" << width << ", " << height << ")\n"
        << R"USD(    }
    def RenderVar "Color"
    {
        uniform token dataType = "color4f"
        uniform string sourceName = "LdrColor"
        uniform token sourceType = "raw"
    }
}
)USD";
    return usd.str();
}
} // namespace

struct RtxView::Impl
{
    ovrtx_renderer_t *renderer = nullptr;
    bool initialized = false;
    bool attached = false;
    std::unique_ptr<Scene> scene;
    std::vector<uint8_t> pixels;
    int width = 0, height = 0;
    std::array<double, 16> lastCamera{};
    std::array<float, 7> lastLens{};
    ~Impl()
    {
        if (attached)
            ovrtx_detach_ovstage(renderer);
        scene.reset();
        if (renderer)
            ovrtx_destroy_renderer(renderer);
        if (initialized)
            ovstage_shutdown();
    }
};

RtxView::RtxView() : impl(std::make_unique<Impl>()) {}
RtxView::~RtxView() = default;

void RtxView::initialize(const std::string &executableDirectory)
{
    if (impl->initialized)
        return;
    const auto root = std::filesystem::u8path(executableDirectory) / ".." / "..";
    const auto rtxRoot = std::filesystem::canonical(root / "deps/ovrtx/bin").u8string();
    const auto stageRoot = std::filesystem::canonical(root / "deps/ovstage/bin").u8string();
    ovrtx_config_entry_t entries[] = {ovrtx_config_entry_binary_package_root_path(str(rtxRoot))};
    ovrtx_config_t config{entries, 1};
    if (!impl->renderer)
    {
        rtxCheck(ovrtx_register_schema_paths(&config), "Register renderer schemas");
        rtxCheck(ovrtx_create_renderer(&config, &impl->renderer), "Initialize RTX renderer");
    }
    ovstage_config_entry_t stageEntries[] = {ovstage_config_entry_binary_package_root_path(str(stageRoot))};
    ovstage_config_t stageConfig{stageEntries, 1};
    stageCheck(ovstage_initialize(&stageConfig), "Initialize stage runtime");
    impl->initialized = true;
}

void RtxView::load(const std::string &path, int width, int height)
{
    if (!impl->initialized)
        throw std::runtime_error("Renderer is not initialized.");
    if (!std::filesystem::is_regular_file(std::filesystem::u8path(path)))
        throw std::runtime_error("USD file does not exist: " + path);
    auto candidate = std::make_unique<Scene>();
    candidate->path = path;
    ovstage_instance_desc_t desc{};
    desc.name = "fbik-physx-viewer";
    stageCheck(ovstage_create_instance(&desc, &candidate->stage), "Create stage");
    const ovx_string_t metadata[] = {lit("upAxis"), lit("metersPerUnit")};
    ovstage_population_desc_t population{};
    population.domains = OVSTAGE_POPULATION_DOMAIN_RENDERING;
    std::vector<ovx_string_t> properties;
    for (const auto *name : cameraAttributeNames)
        properties.push_back(lit(name));
    const ovx_string_t primMetadata[] = {lit("instanceable")};
    const ovx_string_t propertyMetadata[] = {lit("interpolation"), lit("bindMaterialAs")};
    ovstage_population_selector_t selector{};
    selector.prim_predicate.kind = OVSTAGE_POPULATION_PRIM_PREDICATE_ALL;
    selector.property_predicate = ovstage_population_property_predicate_has_name(properties.data(), properties.size());
    selector.prim_metadata_paths = primMetadata;
    selector.prim_metadata_path_count = 1;
    selector.property_metadata_paths = propertyMetadata;
    selector.property_metadata_path_count = 2;
    population.selectors = &selector;
    population.selector_count = 1;
    population.stage_metadata_paths = metadata;
    population.stage_metadata_path_count = 2;
    populationWait(candidate->stage, ovstage_population_open_usd_from_file_with_desc(
                                         candidate->stage, str(path), candidate->ordinal, 0, &population, 1));
    candidate->seal();
    stageWait(candidate->stage,
              ovstage_compute_hierarchy(candidate->stage, OVSTAGE_HIERARCHY_COMPUTATION_MODEL_DEFAULT_CPU,
                                        candidate->ordinal, candidate->ordinal + 1));
    ++candidate->ordinal;
    candidate->seal();
    frameBounds(*candidate);
    candidate->initialCamera = readStageCamera(candidate->stage, candidate->ordinal);
    if (candidate->initialCamera)
        std::printf("Initial USD camera: %s\n", candidate->initialCamera->path.c_str());
    const auto prims = stage::read(candidate->stage, candidate->ordinal, {"usd-prim-type"});
    const auto rootExists = [&] {
        for (const auto &[primPath, prim] : prims)
            if (primPath == candidate->viewerRoot || primPath.rfind(candidate->viewerRoot + "/", 0) == 0)
                return true;
        return false;
    };
    while (rootExists())
        candidate->viewerRoot += "_";
    const std::string layer = viewerLayer(candidate->viewerRoot, width, height);
    populationWait(candidate->stage,
                   ovstage_population_add_usd_reference_from_string(candidate->stage, str(layer), lit("/"), nullptr));
    populationWait(candidate->stage, ovstage_population_apply_usd_changes(candidate->stage, ++candidate->ordinal));
    candidate->seal();
    candidate->camera = candidate->query((candidate->viewerRoot + "/Camera").c_str());
    candidate->product = candidate->query((candidate->viewerRoot + "/View").c_str());
    if (impl->attached)
    {
        rtxCheck(ovrtx_detach_ovstage(impl->renderer), "Detach previous stage");
        impl->attached = false;
    }
    try
    {
        rtxCheck(ovrtx_attach_ovstage(impl->renderer, candidate->stage), "Attach USD stage");
        impl->attached = true;
        rtxWait(impl->renderer, ovrtx_update_from_stage(impl->renderer, candidate->ordinal));
    }
    catch (...)
    {
        if (impl->attached)
            ovrtx_detach_ovstage(impl->renderer);
        impl->attached = false;
        if (impl->scene)
        {
            impl->attached = ovrtx_attach_ovstage(impl->renderer, impl->scene->stage).status == OVRTX_API_SUCCESS;
            if (impl->attached)
                rtxWait(impl->renderer, ovrtx_update_from_stage(impl->renderer, impl->scene->ordinal));
        }
        throw;
    }
    impl->scene = std::move(candidate);
    impl->width = impl->height = 0;
    impl->lastLens = {};
    impl->pixels.clear();
}

void RtxView::render(const Camera &camera, int width, int height)
{
    if (!impl->scene || !impl->attached)
        return;
    Scene &scene = *impl->scene;
    const std::array<double, 16> matrix{
        camera.right.x,    camera.right.y,    camera.right.z,    0, camera.up.x,  camera.up.y,  camera.up.z,  0,
        -camera.forward.x, -camera.forward.y, -camera.forward.z, 0, camera.pos.x, camera.pos.y, camera.pos.z, 1};
    const bool resized = width != impl->width || height != impl->height;
    const std::array<float, 7> lens{camera.fov,
                                    float(camera.orthographic),
                                    camera.orthographicHeight,
                                    camera.horizontalOffset,
                                    camera.verticalOffset,
                                    camera.nearClip,
                                    camera.farClip};
    const bool lensChanged = lens != impl->lastLens;
    // USD reference population can refresh camera/product attributes from their
    // source layer. Reapply the live view even when the camera has not moved.
    if (matrix != impl->lastCamera || resized || lensChanged)
    {
        ++scene.ordinal;
        scene.write(scene.camera, "omni:xform", const_cast<double *>(matrix.data()), kDLFloat, 64, 16,
                    OVSTAGE_SEMANTIC_MATRIX);
        if (resized || lensChanged)
        {
            int32_t resolution[2] = {width, height};
            float aperture = 20.955f * static_cast<float>(height) / width;
            if (camera.orthographic)
                aperture = 10 * camera.orthographicHeight;
            float horizontal = aperture * width / height;
            float focal = camera.orthographic ? 50 : aperture / (2 * std::tan(camera.fov * 3.14159265f / 360));
            float horizontalOffset = aperture * camera.horizontalOffset;
            float verticalOffset = aperture * camera.verticalOffset;
            float clip[] = {camera.nearClip, camera.farClip};
            const auto name = lit(camera.orthographic ? "orthographic" : "perspective");
            ovx_token_t projection{};
            auto *dictionary = ovstage_get_path_dictionary(scene.stage);
            const auto result = path_dictionary_create_tokens_from_strings(dictionary, &name, 1, &projection);
            if (result.status != OVX_API_SUCCESS)
            {
                path_dictionary_release_error(dictionary, result.error);
                throw std::runtime_error("Could not create camera projection token.");
            }
            scene.write(scene.product, "resolution", resolution, kDLInt, 32, 2);
            scene.write(scene.camera, "projection", &projection, kDLUInt, 64, 1, OVSTAGE_SEMANTIC_TOKEN_ID);
            scene.write(scene.camera, "horizontalAperture", &horizontal, kDLFloat, 32, 1);
            scene.write(scene.camera, "verticalAperture", &aperture, kDLFloat, 32, 1);
            scene.write(scene.camera, "horizontalApertureOffset", &horizontalOffset, kDLFloat, 32, 1);
            scene.write(scene.camera, "verticalApertureOffset", &verticalOffset, kDLFloat, 32, 1);
            scene.write(scene.camera, "focalLength", &focal, kDLFloat, 32, 1);
            scene.write(scene.camera, "clippingRange", clip, kDLFloat, 32, 2);
        }
        scene.seal();
        stage::hierarchy(scene.stage, scene.ordinal);
        if (resized)
            rtxWait(impl->renderer, ovrtx_update_from_stage(impl->renderer, scene.ordinal));
        impl->lastCamera = matrix;
        impl->lastLens = lens;
    }
    const auto productPath = scene.viewerRoot + "/View";
    ovx_string_t product = str(productPath);
    ovrtx_render_product_set_t products{&product, 1};
    ovrtx_step_result_handle_t resultHandle = OVRTX_INVALID_HANDLE;
    ovrtx_render_var_output_t mapped{};
    bool isMapped = false;
    try
    {
        {
            rtxWait(impl->renderer,
                    ovrtx_step_with_stage(impl->renderer, products, 1.0 / 60.0, scene.ordinal, &resultHandle));
        }
        ovrtx_render_product_set_outputs_t outputs{};
        rtxCheck(ovrtx_fetch_results(impl->renderer, resultHandle, ovrtx_timeout_infinite, &outputs),
                 "Fetch rendered frame");
        ovrtx_render_var_output_handle_t color = OVRTX_INVALID_HANDLE;
        for (size_t p = 0; p < outputs.output_count; ++p)
            for (size_t f = 0; f < outputs.outputs[p].output_frame_count; ++f)
            {
                const auto &frame = outputs.outputs[p].output_frames[f];
                for (size_t v = 0; v < frame.render_var_count; ++v)
                    if (text(frame.output_render_vars[v].render_var_path) == scene.viewerRoot + "/Color")
                        color = frame.output_render_vars[v].output_handle;
            }
        if (color == OVRTX_INVALID_HANDLE)
            throw std::runtime_error("RTX returned no color image.");
        ovrtx_map_output_description_t mapDesc{};
        mapDesc.device_type = OVRTX_MAP_DEVICE_TYPE_CPU;
        {
            rtxCheck(ovrtx_map_render_var_output(impl->renderer, color, &mapDesc, ovrtx_timeout_infinite, &mapped),
                     "Read RTX image");
            isMapped = true;
        }
        if (mapped.num_tensors != 1)
            throw std::runtime_error("Unexpected RTX image tensor count.");
        const DLTensor &image = *mapped.tensors[0].dl;
        if (image.ndim != 3 || image.dtype.code != kDLUInt || image.dtype.bits != 8 || image.dtype.lanes != 1 ||
            image.shape[2] != 4 || image.shape[0] <= 0 || image.shape[1] <= 0)
            throw std::runtime_error("Expected an RGBA8 RTX image.");
        impl->width = static_cast<int>(image.shape[1]);
        impl->height = static_cast<int>(image.shape[0]);
        impl->pixels.resize(static_cast<size_t>(impl->width) * impl->height * 4);
        const auto *source = static_cast<const uint8_t *>(image.data) + image.byte_offset;
        const int64_t rowStride = image.strides ? image.strides[0] : impl->width * 4;
        if (image.strides && (image.strides[1] != 4 || image.strides[2] != 1))
            throw std::runtime_error("Unexpected RTX pixel layout.");
        for (int y = 0; y < impl->height; ++y)
            std::memcpy(impl->pixels.data() + static_cast<size_t>(y) * impl->width * 4, source + y * rowStride,
                        static_cast<size_t>(impl->width) * 4);
        rtxCheck(ovrtx_unmap_render_var_output(impl->renderer, mapped.map_handle, {}), "Unmap RTX image");
        isMapped = false;
        rtxCheck(ovrtx_destroy_results(impl->renderer, resultHandle), "Release RTX frame");
    }
    catch (...)
    {
        if (isMapped)
            ovrtx_unmap_render_var_output(impl->renderer, mapped.map_handle, {});
        if (resultHandle != OVRTX_INVALID_HANDLE)
            ovrtx_destroy_results(impl->renderer, resultHandle);
        throw;
    }
}

bool RtxView::loaded() const
{
    return impl->scene && impl->attached;
}
const std::vector<uint8_t> &RtxView::pixels() const
{
    return impl->pixels;
}
int RtxView::width() const
{
    return impl->width;
}
int RtxView::height() const
{
    return impl->height;
}
Vec3 RtxView::center() const
{
    return impl->scene ? impl->scene->center : Vec3(0, 0, 0);
}
float RtxView::radius() const
{
    return impl->scene ? impl->scene->radius : 5;
}
bool RtxView::zUp() const
{
    return impl->scene && impl->scene->zUp;
}
const Camera *RtxView::initialCamera() const
{
    return impl->scene && impl->scene->initialCamera ? &impl->scene->initialCamera->camera : nullptr;
}
const std::string &RtxView::path() const
{
    static const std::string empty;
    return impl->scene ? impl->scene->path : empty;
}
} // namespace viewer
