#include "demo_documents.h"

#include <spdlog/spdlog.h>

#include <format>
#include <utility>
#include <filesystem>
#include <system_error>

namespace praxis::demo {

namespace {

// A copy that did not land is named here, once. The target is answered either way, because the
// writer's own message is what reports a save that could not be made.
void reproduce(const std::filesystem::path &seed, const std::filesystem::path &target)
{
    std::error_code failed;
    std::filesystem::copy_file(seed, target, std::filesystem::copy_options::skip_existing, failed);
    if(failed)
        spdlog::error(std::format("The document {} was not copied to {}: {}", seed.string(), target.string(), failed.message()));
}

}

documents::documents(std::filesystem::path seeds, std::filesystem::path state)
        : m_seeds(std::move(seeds))
        , m_state(std::move(state))
{
}

const std::filesystem::path &documents::seeds() const
{
    return m_seeds;
}

config::location documents::reading(const std::filesystem::path &named) const
{
    const config::location copy = config::resolve(named, m_state);

    return std::filesystem::exists(copy.resolved) ? copy : config::resolve(named, m_seeds);
}

config::location documents::writing(const std::filesystem::path &named) const
{
    const config::location target         = config::resolve(named, m_state);
    const std::filesystem::path directory = target.resolved.parent_path();

    std::error_code failed;
    std::filesystem::create_directories(directory, failed);
    if(failed)
        spdlog::error(std::format("The directory {} could not be created: {}", directory.string(), failed.message()));

    const config::location seed = config::resolve(named, m_seeds);
    if(!std::filesystem::exists(target.resolved) && std::filesystem::exists(seed.resolved))
        reproduce(seed.resolved, target.resolved);

    return target;
}

}
