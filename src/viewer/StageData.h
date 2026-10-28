#pragma once
#include <map>
#include <memory>
#include <array>
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
// Explicit path order is also the tensor row order. The stage must outlive this object.
class Query
{
  public:
    Query();
    ~Query();
    Query(const Query &) = delete;
    Query &operator=(const Query &) = delete;
    void init(ovstage_instance_t *stage, const std::vector<std::string> &paths,
              const std::vector<std::string> &attributes = {});
    Snapshot read(ovstage_ordinal_t ordinal) const;
    void writeTransforms(ovstage_ordinal_t ordinal, const std::vector<std::array<double, 16>> &local,
                         const std::vector<std::array<double, 16>> *world = nullptr) const;
    void clear();
  private:
    struct Impl;
    std::unique_ptr<Impl> impl;
};
} // namespace viewer::stage
