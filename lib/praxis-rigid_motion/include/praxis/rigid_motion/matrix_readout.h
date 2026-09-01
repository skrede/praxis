#ifndef HPP_GUARD_PRAXIS_RIGID_MOTION_MATRIX_READOUT_H
#define HPP_GUARD_PRAXIS_RIGID_MOTION_MATRIX_READOUT_H

#include "praxis/rigid_motion/frame_stencil.h"

#include "praxis/scene/labeled_value_window.h"

#include <span>
#include <cstddef>
#include <cstdint>
#include <functional>

namespace praxis::rigid_motion {

enum class matrix_form : std::uint8_t
{
    rotation,
    transformation
};

// parent_relative reads the placement as the stencil holds it, expressed in the frame that
// placement is relative to; world reads that chain composed and expressed in the world frame.
// Contiguous from zero, so a combo box indexes the label table with the enumerator's own value.
enum class matrix_frame : std::uint8_t
{
    parent_relative,
    world
};

std::span<const char *const> matrix_frame_labels();

class matrix_readout
{
public:
    // The route is asked once per reading rather than once at composition, so a scenario changing
    // which object is selected drives this by answering differently.
    matrix_readout(const frame_stencil &read, matrix_form drawn, std::function<std::size_t()> selected);

    matrix_frame frame() const;

    void set_frame(matrix_frame chosen);

    void render_controls();

    scene::readout reading() const;

private:
    matrix_form m_form;
    matrix_frame m_frame;
    const frame_stencil &m_stencil;
    std::function<std::size_t()> m_selected;
};

}

#endif
