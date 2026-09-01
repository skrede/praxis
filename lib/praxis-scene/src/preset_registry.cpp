#include "praxis/scene/preset_registry.h"

#include <ranges>
#include <utility>

namespace praxis::scene {

std::vector<std::string> preset_registry::preset_names() const
{
    std::vector<std::string> names;
    for(const auto &key : m_presets | std::views::keys)
        names.push_back(key);
    return names;
}

preset_registry::factory preset_registry::load_preset(const std::string &name)
{
    if(const auto it = m_presets.find(name); it != m_presets.end())
        return it->second;
    return nullptr;
}

void preset_registry::register_preset(const std::string &name, factory preset_factory)
{
    m_presets[name] = std::move(preset_factory);
}

}
