#include "RecentFiles.h"
#include <fstream>
#include <iostream>
#include <stdexcept>

void require(bool value, const char *message)
{
    if (!value)
        throw std::runtime_error(message);
}
int main(int argc, char **argv)
{
    try
    {
        require(argc == 2, "Expected test output directory");
        const auto directory = std::filesystem::absolute(argv[1]);
        std::filesystem::create_directories(directory);
        const auto storage = directory / "recent.txt";
        { std::ofstream reset(storage, std::ios::trunc); }
        viewer::RecentFiles history(storage);
        require(history.paths().empty(), "Empty history should be usable");
        const auto unicode = (directory / std::filesystem::path(L"character \u00e4 \u65e5.usd")).u8string();
        history.add(unicode);
        history.save();
        viewer::RecentFiles restored(storage);
        require(restored.paths().size() == 1 && restored.paths().front() == unicode, "Unicode path did not survive restart");
        for (int i = 0; i < 12; ++i)
            restored.add((directory / ("file" + std::to_string(i) + ".usd")).u8string());
        require(restored.paths().size() == 10, "Recent files are not bounded");
        const auto repeated = (directory / "FILE5.USD").u8string();
        restored.add(repeated);
        require(restored.paths().front() == repeated && restored.paths().size() == 10, "Duplicate should move to front");
        require(std::filesystem::u8path(restored.paths()[1]).filename() == "file11.usd", "Recent order changed");
        restored.save();
        require(viewer::RecentFiles(storage).paths() == restored.paths(), "Saved order differs");
        {
            std::ofstream append(storage, std::ios::trunc);
            append << "relative.usd\n\n" << unicode << '\n' << unicode << '\n';
        }
        viewer::RecentFiles malformed(storage);
        require(malformed.paths().size() == 1 && malformed.paths().front() == unicode, "Invalid or duplicate entries retained");
        // Missing files remain selectable so the viewer can report a load error.
        require(!std::filesystem::exists(std::filesystem::u8path(unicode)), "Fixture unexpectedly exists");
        malformed.save();
        std::cout << "Recent-file persistence, Unicode, ordering, deduplication and limit passed.\n";
        return 0;
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
        return 1;
    }
}
