#ifndef HPP_GUARD_PRAXIS_SCENE_KEY_WINDOW_H
#define HPP_GUARD_PRAXIS_SCENE_KEY_WINDOW_H

#include "praxis/scene/imgui_window.h"

#include <string>
#include <vector>
#include <cstdint>
#include <functional>

namespace praxis::scene {

// One row of a key: what it names, and the tone its swatch is drawn in, as 0xRRGGBB.
struct key_entry
{
    std::string label;
    std::uint32_t tone;
};

using key_source = std::function<std::vector<key_entry>()>;

// One row per entry the source answers, in its order: a filled square a text line high in the
// entry's tone, then its label.
class key_window : public imgui_window
{
public:
    key_window(std::string name, key_source source);

    void render() override;

private:
    key_source m_source;
};

}

#endif
