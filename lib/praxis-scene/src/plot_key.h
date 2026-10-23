#ifndef HPP_GUARD_PRAXIS_SCENE_PLOT_KEY_H
#define HPP_GUARD_PRAXIS_SCENE_PLOT_KEY_H

#include "praxis/scene/plot_window.h"

#include <imgui.h>

#include <string>
#include <vector>
#include <cstddef>

namespace praxis::scene {

// One entry per distinct series name the plotting library would show in its own legend, each a
// swatch a text line high and the name, laid out in rows that wrap at the width given.
class plot_key
{
public:
    plot_key(const std::vector<plot_series> &series, float width);

    // What the key takes under the plot, the item spacing above each row included.
    float height() const;

    void take_color(const std::string &name, ImVec4 color);
    void draw() const;

private:
    float m_width;
    std::vector<std::string> m_names;
    std::vector<ImU32> m_colors;

    std::size_t rows() const;
};

}

#endif
