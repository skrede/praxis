#include "demo_offered.h"

#include "demo_machines.h"

#include <string>
#include <memory>
#include <vector>
#include <filesystem>

namespace praxis::demo {

std::vector<std::string> register_offered(const std::shared_ptr<scene::preset_registry> &registry, const documents &mine, const config::document &values,
                                          const std::filesystem::path &packages, const std::shared_ptr<write_back> &writing)
{
    report_composed_capabilities();

    return register_arm_presets(registry, values, mine, packages, writing);
}

}
