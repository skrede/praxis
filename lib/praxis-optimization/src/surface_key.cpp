#include "surface_keyed.h"

#include "praxis/optimization/surface_key.h"

#include "praxis/scene/key_window.h"

#include <span>
#include <memory>
#include <string>
#include <vector>
#include <utility>

namespace praxis::optimization {

bool shown_through(const threepp::Object3D &figure)
{
    for(const threepp::Object3D *at = &figure; at != nullptr; at = at->parent)
        if(!at->visible)
            return false;

    return true;
}

std::vector<scene::key_entry> drawn_entries(std::span<const keyed_figure> figures)
{
    std::vector<scene::key_entry> entries;
    for(const keyed_figure &figure : figures)
        if(figure.drawn)
            entries.push_back(scene::key_entry{figure.label, figure.tone});

    return entries;
}

std::shared_ptr<scene::key_window> surface_key(const scenario_window_inputs &inputs, std::string name)
{
    return std::make_shared<scene::key_window>(std::move(name), [keyed = inputs.keyed] { return keyed != nullptr ? *keyed : std::vector<scene::key_entry>{}; });
}

}
