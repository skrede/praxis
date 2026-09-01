#include "praxis/config/store.h"

#include <string>
#include <cstdlib>
#include <filesystem>
#include <system_error>

namespace praxis::config {
namespace {

// The process environment is the whole of what is read here.
std::filesystem::path home()
{
#ifdef _WIN32
    if(const char *profile = std::getenv("USERPROFILE"); profile != nullptr && *profile != '\0')
        return std::filesystem::path(profile);
    const char *drive = std::getenv("HOMEDRIVE");
    const char *below = std::getenv("HOMEPATH");
    if(drive != nullptr && below != nullptr && *drive != '\0' && *below != '\0')
        return std::filesystem::path(std::string(drive) + below);
#else
    if(const char *set = std::getenv("HOME"); set != nullptr && *set != '\0')
        return std::filesystem::path(set);
#endif
    return std::filesystem::path();
}

// A path a person wrote may begin with `~`, which names the home directory; where no home directory
// can be read the tilde is left as it was written, so the path reported is the path looked at.
std::filesystem::path expanded(const std::filesystem::path &given)
{
    if(given.empty() || *given.begin() != "~")
        return given;
    const std::filesystem::path where = home();
    return where.empty() ? given : where / given.lexically_relative("~");
}

}

location resolve(const std::filesystem::path &given, const std::filesystem::path &base)
{
    const std::filesystem::path wanted = expanded(given);
    const std::filesystem::path joined = wanted.is_absolute() ? wanted : base / wanted;
    std::error_code failed;
    const std::filesystem::path resolved = std::filesystem::weakly_canonical(joined, failed);
    return location{given, failed ? joined.lexically_normal() : resolved};
}

}
