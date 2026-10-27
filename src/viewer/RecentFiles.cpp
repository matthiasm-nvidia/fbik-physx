#define NOMINMAX
#include <windows.h>
#include "RecentFiles.h"
#include <algorithm>
#include <fstream>
#include <stdexcept>

namespace viewer
{
namespace
{
constexpr size_t maxRecentFiles = 10;
bool samePath(const std::string &a, const std::string &b)
{
    const auto first = std::filesystem::u8path(a).wstring();
    const auto second = std::filesystem::u8path(b).wstring();
    return CompareStringOrdinal(first.c_str(), -1, second.c_str(), -1, TRUE) == CSTR_EQUAL;
}
} // namespace

RecentFiles::RecentFiles(std::filesystem::path storage) : storage_(std::move(storage))
{
    std::ifstream input(storage_);
    std::string line;
    while (paths_.size() < maxRecentFiles && std::getline(input, line))
    {
        if (!line.empty() && line.back() == '\r')
            line.pop_back();
        try
        {
            if (!line.empty() && std::filesystem::u8path(line).is_absolute() &&
                std::none_of(paths_.begin(), paths_.end(), [&](const auto &p) { return samePath(p, line); }))
                paths_.push_back(line);
        }
        catch (const std::exception &)
        {
            // Ignore a malformed history entry; it must not prevent opening the viewer.
        }
    }
}

void RecentFiles::add(const std::string &path)
{
    const auto absolute = std::filesystem::weakly_canonical(std::filesystem::absolute(std::filesystem::u8path(path))).u8string();
    paths_.erase(std::remove_if(paths_.begin(), paths_.end(), [&](const auto &p) { return samePath(p, absolute); }), paths_.end());
    paths_.insert(paths_.begin(), absolute);
    if (paths_.size() > maxRecentFiles)
        paths_.resize(maxRecentFiles);
}

void RecentFiles::save() const
{
    std::filesystem::create_directories(storage_.parent_path());
    auto temporary = storage_;
    temporary += ".tmp";
    std::ofstream output(temporary, std::ios::binary | std::ios::trunc);
    for (const auto &path : paths_)
        output << path << '\n';
    output.close();
    if (!output || !MoveFileExW(temporary.c_str(), storage_.c_str(), MOVEFILE_REPLACE_EXISTING | MOVEFILE_WRITE_THROUGH))
        throw std::runtime_error("Could not save recent USD files.");
}
} // namespace viewer
