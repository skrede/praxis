#ifndef HPP_GUARD_PRAXIS_TESTS_PRESETS_SCRATCH_DIRECTORY_H
#define HPP_GUARD_PRAXIS_TESTS_PRESETS_SCRATCH_DIRECTORY_H

#include <atomic>
#include <random>
#include <string>
#include <filesystem>
#include <system_error>

namespace praxis::fixture {

// A directory under the temporary root that no other process and no two calls here name. It is
// created on the way out, and whoever asked for it owns it and removes it as a tree.
inline std::filesystem::path scratch_directory()
{
    static const unsigned long long process = std::random_device{}();
    static std::atomic<unsigned long long> instance{0u};

    for(unsigned attempt = 0u; attempt < 64u; ++attempt)
    {
        const std::filesystem::path directory = std::filesystem::temp_directory_path() / ("praxis-" + std::to_string(process) + "-" + std::to_string(instance++));
        if(std::filesystem::create_directory(directory))
            return directory;
    }

    throw std::filesystem::filesystem_error("no unclaimed scratch directory under the temporary root", std::make_error_code(std::errc::file_exists));
}

// The one scratch directory a whole run shares, for documents that are written by one function and
// resolved by another. It is removed as a tree when the run ends.
inline const std::filesystem::path &shared_scratch_directory()
{
    struct owned_tree
    {
        owned_tree()
                : where(scratch_directory())
        {
        }

        ~owned_tree()
        {
            std::error_code ignored;
            std::filesystem::remove_all(where, ignored);
        }

        std::filesystem::path where;
    };

    static const owned_tree kept;

    return kept.where;
}

}

#endif
