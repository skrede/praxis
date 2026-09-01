#include "demo_offered.h"
#include "demo_configuration.h"

#include "praxis/presets/arrangements.h"

#include "praxis/config/store.h"

#include <string>
#include <memory>
#include <vector>
#include <filesystem>

namespace praxis::demo {

std::vector<std::string> register_offered(const std::shared_ptr<scene::preset_registry> &registry, const documents &mine, const config::document &values, const std::filesystem::path &,
                                          const std::shared_ptr<write_back> &writing)
{
    const std::vector<config::location> reading = preset_locations(values, mine);

    return presets::register_arrangements(
            registry, reading, [mine](const std::filesystem::path &named) { return mine.reading(named); },
            [writing](const config::binding &at, const config::document &carried) { writing->composing(at, carried); });
}

}
