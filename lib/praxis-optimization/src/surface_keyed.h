#ifndef HPP_GUARD_PRAXIS_OPTIMIZATION_SURFACE_KEYED_H
#define HPP_GUARD_PRAXIS_OPTIMIZATION_SURFACE_KEYED_H

#include "praxis/scene/key_window.h"

#include <threepp/core/Object3D.hpp>

#include <span>
#include <vector>
#include <cstdint>

namespace praxis::optimization {

// Shown only where the figure and every node above it are.
bool shown_through(const threepp::Object3D &figure);

// A figure a surface may draw, whether this frame drew it, and its key label and tone.
struct keyed_figure
{
    bool drawn;
    const char *label;
    std::uint32_t tone;
};

// An entry for each drawn figure, in their order.
std::vector<scene::key_entry> drawn_entries(std::span<const keyed_figure> figures);

}

#endif
