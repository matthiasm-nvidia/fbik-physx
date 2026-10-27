#pragma once
#include <map>
#include <ovstage/ovstage.h>
#include <string>
#include <vector>

namespace viewer::stage
{
ovx_string_t str(const char *text);
void check(ovstage_api_status_t status);
void wait(ovstage_instance_t *stage, ovstage_enqueue_result_t op);
void seal(ovstage_instance_t *stage, ovstage_ordinal_t ordinal);
void hierarchy(ovstage_instance_t *stage, ovstage_ordinal_t ordinal);
struct Value
{
    std::vector<double> numbers;
    std::string token;
    std::vector<std::string> tokens;
    std::vector<std::string> paths;
};
using Prim = std::map<std::string, Value>;
using Snapshot = std::map<std::string, Prim>;
Snapshot read(ovstage_instance_t *stage, ovstage_ordinal_t ordinal, const std::vector<std::string> &names);
} // namespace viewer::stage
