#ifndef HPP_GUARD_PRAXIS_SCENE_PRESET_REGISTRY_H
#define HPP_GUARD_PRAXIS_SCENE_PRESET_REGISTRY_H

#include "praxis/scene/preset.h"

#include <memory>
#include <string>
#include <vector>
#include <functional>
#include <unordered_map>

namespace praxis::scene {

class preset_registry
{
public:
    using factory = std::function<std::shared_ptr<preset>(const preset_site &)>;

    std::vector<std::string> preset_names() const;

    factory load_preset(const std::string &name);
    void register_preset(const std::string &name, factory preset_factory);

private:
    std::unordered_map<std::string, factory> m_presets;
};

}

#endif
