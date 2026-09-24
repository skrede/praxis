#include "praxis/scene/preset_registry.h"

#include <utility>
#include <algorithm>

namespace praxis::scene {

std::vector<std::string> preset_registry::preset_names() const
{
    std::vector<std::string> names;
    names.reserve(m_presets.size());
    for(const auto &entry : m_presets)
        names.push_back(entry.first);
    return names;
}

preset_registry::factory preset_registry::load_preset(const std::string &name)
{
    const auto entry = std::ranges::find_if(m_presets, [&name](const auto &held) { return held.first == name; });
    if(entry != m_presets.end())
        return entry->second;
    return nullptr;
}

void preset_registry::register_preset(const std::string &name, factory preset_factory)
{
    const auto entry = std::ranges::find_if(m_presets, [&name](const auto &held) { return held.first == name; });
    if(entry != m_presets.end())
    {
        entry->second = std::move(preset_factory);
        return;
    }

    m_presets.emplace_back(name, std::move(preset_factory));
}

}
