#pragma once
#include <filesystem>
#include <string>
#include <vector>

namespace viewer
{
class RecentFiles
{
  public:
    explicit RecentFiles(std::filesystem::path storage);
    // Call only after successfully loading the USD stage. Updates most-recent-first order.
    void add(const std::string &path);
    void save() const;
    const std::vector<std::string> &paths() const { return paths_; }

  private:
    std::filesystem::path storage_;
    std::vector<std::string> paths_;
};
} // namespace viewer
