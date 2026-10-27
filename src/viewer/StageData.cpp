#include "StageData.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <ovx/path_dictionary/path_dictionary_utils.h>
#include <stdexcept>
#include <unordered_map>

namespace viewer::stage
{
namespace
{
std::string text(ovx_string_t s)
{
    return s.ptr ? std::string(s.ptr, s.length) : std::string();
}
void dictionaryCheck(path_dictionary_instance_t *dict, ovx_api_result_t result)
{
    if (result.status != OVX_API_SUCCESS)
    {
        const auto error = text(result.error);
        path_dictionary_release_error(dict, result.error);
        throw std::runtime_error("Stage path dictionary: " + error);
    }
}
std::string pathText(path_dictionary_instance_t *dict, ovx_primpath_t path)
{
    std::vector<ovx_token_t> buffer(4096);
    ovx_token_t *tokens = nullptr;
    size_t count = 0, processed = 0;
    dictionaryCheck(dict, path_dictionary_get_tokens_from_paths(dict, &path, 1, buffer.data(), buffer.size(), &tokens,
                                                                &count, &processed));
    if (processed != 1)
        throw std::runtime_error("USD prim path is too deep.");
    std::vector<ovx_string_t> names(count);
    if (count)
        dictionaryCheck(dict, path_dictionary_get_strings_from_tokens(dict, tokens, count, names.data()));
    std::string result;
    for (auto name : names)
    {
        auto part = text(name);
        if (!part.empty() && part != "/")
            result += "/" + part;
    }
    return result.empty() ? "/" : result;
}
template <class T> T valueAt(const DLTensor &t, size_t row, size_t lane)
{
    T value;
    const auto stride = t.strides ? t.strides[0] : 1;
    const auto offset = (row * stride * t.dtype.lanes + lane) * sizeof(T);
    std::memcpy(&value, static_cast<const uint8_t *>(t.data) + t.byte_offset + offset, sizeof(T));
    return value;
}
double number(const DLTensor &t, size_t row, size_t lane)
{
    if (t.dtype.code == kDLFloat && t.dtype.bits == 16)
    {
        const auto bits = valueAt<uint16_t>(t, row, lane);
        const int exponent = (bits >> 10) & 31, fraction = bits & 1023;
        const double magnitude = exponent == 0    ? std::ldexp(double(fraction), -24)
                                 : exponent == 31 ? (fraction ? NAN : INFINITY)
                                                  : std::ldexp(double(1024 + fraction), exponent - 25);
        return (bits & 0x8000) ? -magnitude : magnitude;
    }
    if (t.dtype.code == kDLFloat && t.dtype.bits == 32)
        return valueAt<float>(t, row, lane);
    if (t.dtype.code == kDLFloat && t.dtype.bits == 64)
        return valueAt<double>(t, row, lane);
    if (t.dtype.code == kDLInt && t.dtype.bits == 32)
        return valueAt<int32_t>(t, row, lane);
    if (t.dtype.code == kDLUInt && t.dtype.bits == 32)
        return valueAt<uint32_t>(t, row, lane);
    if ((t.dtype.code == kDLBool || t.dtype.code == kDLUInt) && t.dtype.bits == 8)
        return valueAt<uint8_t>(t, row, lane);
    throw std::runtime_error("Unsupported USD numeric attribute type.");
}
} // namespace
ovx_string_t str(const char *s)
{
    return {s, std::strlen(s)};
}
void check(ovstage_api_status_t status)
{
    if (status != OVSTAGE_OK)
        throw std::runtime_error("Stage (status " + std::to_string(status) + "): " + text(ovstage_get_last_error()));
}
void wait(ovstage_instance_t *stage, ovstage_enqueue_result_t op)
{
    check(op.status);
    ovstage_op_wait_result_t result{};
    const auto status = ovstage_wait_op(stage, op.op_index, OVSTAGE_TIMEOUT_INFINITE, &result);
    std::string error;
    for (size_t i = 0; i < result.error_op_id_count; ++i)
        error += text(ovstage_get_last_op_error(stage, result.error_op_ids[i])) + "\n";
    ovstage_release_op(stage, op.op_index);
    check(status);
    if (!error.empty())
        throw std::runtime_error(error);
}
void seal(ovstage_instance_t *stage, ovstage_ordinal_t ordinal)
{
    ovstage_write_floor_desc_t floor{};
    floor.ordinal = ordinal;
    floor.scope = OVSTAGE_SCOPE_ALL;
    wait(stage, ovstage_advance_write_floor(stage, &floor));
}
void hierarchy(ovstage_instance_t *stage, ovstage_ordinal_t ordinal)
{
    wait(stage, ovstage_compute_hierarchy(stage, OVSTAGE_HIERARCHY_COMPUTATION_MODEL_DEFAULT_CPU, ordinal, ordinal));
}
Snapshot read(ovstage_instance_t *stage, ovstage_ordinal_t ordinal, const std::vector<std::string> &requested)
{
    Snapshot output;
    ovstage_query_handle_t query{};
    ovstage_read_handle_t read{};
    wait(stage, ovstage_query(stage, nullptr, nullptr, 0, &query));
    try
    {
        auto *dict = ovstage_get_path_dictionary(stage);
        std::vector<ovx_string_t> names;
        for (const auto &name : requested)
            names.push_back(str(name.c_str()));
        std::vector<ovx_token_t> tokens(names.size());
        dictionaryCheck(dict,
                        path_dictionary_create_tokens_from_strings(dict, names.data(), names.size(), tokens.data()));
        std::unordered_map<ovx_token_t, std::string> lookup;
        for (size_t i = 0; i < tokens.size(); ++i)
            lookup[tokens[i]] = requested[i];

        ovstage_ordinal_range_t range{};
        range.end_ordinal = ordinal;
        wait(stage, ovstage_read_attributes(stage, query, tokens.data(), tokens.size(), range, &read));
        for (;;)
        {
            ovstage_read_group_t group{};
            const auto status = ovstage_fetch_read_next(stage, read, OVSTAGE_TIMEOUT_INFINITE, &group);
            if (status == OVSTAGE_ERROR_END_OF_ITERATION)
                break;
            check(status);
            try
            {
                if (!group.is_delete)
                {
                    size_t count = 0;
                    dictionaryCheck(dict, path_dictionary_get_num_paths_from_path_list(dict, group.prims.list, &count));
                    std::vector<ovx_primpath_t> paths(count);
                    dictionaryCheck(dict, path_dictionary_get_paths_from_path_list(dict, group.prims.list, 0, count,
                                                                                   paths.data(), &count));
                    for (size_t i = 0; i < group.prims.count; ++i)
                    {
                        if (group.data.mask && !(group.data.mask[i / 64] & (uint64_t(1) << (i % 64))))
                            continue;
                        const auto index = group.prims.index_map ? group.prims.index_map[i] : group.prims.offset + i;
                        const auto row = group.data.index_map ? group.data.index_map[i] : i;
                        if (index >= paths.size() || !group.data.tensor_count ||
                            (group.is_array && row >= group.data.tensor_count))
                            throw std::runtime_error("Invalid stage read layout.");
                        const auto &t = group.data.tensors[group.is_array ? row : 0];
                        if (t.ndim != 1 || !t.shape || t.device.device_type != kDLCPU || (t.shape[0] && !t.data))
                            throw std::runtime_error("USD extraction requires CPU attribute data.");
                        Value value;
                        if (group.semantic == OVSTAGE_SEMANTIC_RELATIONSHIP_PATH_ID)
                        {
                            const auto n = group.is_array ? t.shape[0] : 1;
                            for (int64_t k = 0; k < n; ++k)
                                value.paths.push_back(
                                    pathText(dict, valueAt<ovx_primpath_t>(t, group.is_array ? k : row, 0)));
                        }
                        else if (t.dtype.code == kDLUInt && t.dtype.bits == 64)
                        {
                            const auto n = group.is_array ? t.shape[0] : 1;
                            for (int64_t k = 0; k < n; ++k)
                            {
                                const auto token = valueAt<ovx_token_t>(t, group.is_array ? k : row, 0);
                                ovx_string_t s{};
                                if (token)
                                    dictionaryCheck(dict, path_dictionary_get_strings_from_tokens(dict, &token, 1, &s));
                                value.tokens.push_back(text(s));
                            }
                            if (!value.tokens.empty())
                                value.token = value.tokens.front();
                        }
                        else
                        {
                            const auto n = group.is_array ? t.shape[0] * t.dtype.lanes : t.dtype.lanes;
                            for (size_t k = 0; k < static_cast<size_t>(n); ++k)
                                value.numbers.push_back(
                                    number(t, group.is_array ? k / t.dtype.lanes : row, k % t.dtype.lanes));
                        }
                        output[pathText(dict, paths[index])][lookup.at(group.attribute)] = std::move(value);
                    }
                }
            }
            catch (...)
            {
                ovstage_release_group(stage, &group);
                throw;
            }
            check(ovstage_release_group(stage, &group));
        }
    }
    catch (...)
    {
        if (read)
        {
            auto op = ovstage_release_read(stage, read);
            if (op.status == OVSTAGE_OK)
                wait(stage, op);
        }
        wait(stage, ovstage_release_query(stage, query));
        throw;
    }
    wait(stage, ovstage_release_read(stage, read));
    wait(stage, ovstage_release_query(stage, query));
    return output;
}
} // namespace viewer::stage
