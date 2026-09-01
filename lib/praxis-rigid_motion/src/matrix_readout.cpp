#include "praxis/rigid_motion/matrix_readout.h"

#include "praxis/scene/widgets.h"

#include <Eigen/Core>

#include <array>
#include <string>
#include <vector>
#include <cstddef>
#include <utility>

namespace praxis::rigid_motion {

namespace {

constexpr std::array<const char *, 2> frame_labels{"Parent frame", "World frame"};

std::vector<std::vector<scene::labeled_value>> rows_of(const transform &tf, Eigen::Index extent)
{
    std::vector<std::vector<scene::labeled_value>> rows;
    rows.reserve(static_cast<std::size_t>(extent));
    for(Eigen::Index row = 0; row < extent; ++row)
    {
        std::vector<scene::labeled_value> cells;
        cells.reserve(static_cast<std::size_t>(extent));
        for(Eigen::Index column = 0; column < extent; ++column)
            cells.push_back(scene::labeled_value{static_cast<float>(tf(row, column)), std::string()});

        rows.push_back(std::move(cells));
    }

    return rows;
}

}

std::span<const char *const> matrix_frame_labels()
{
    return std::span<const char *const>(frame_labels);
}

matrix_readout::matrix_readout(const frame_stencil &read, matrix_form drawn, std::function<std::size_t()> selected)
        : m_form(drawn)
        , m_frame(matrix_frame::parent_relative)
        , m_stencil(read)
        , m_selected(std::move(selected))
{
}

matrix_frame matrix_readout::frame() const
{
    return m_frame;
}

void matrix_readout::set_frame(matrix_frame chosen)
{
    m_frame = chosen;
}

void matrix_readout::render_controls()
{
    scene::render_enum_selection("POV", m_frame, matrix_frame_labels());
}

// The stencil's accessors are total and read the identity past the end, so every reading carries
// rows and none carries a message.
scene::readout matrix_readout::reading() const
{
    const std::size_t index = m_selected ? m_selected() : 0;
    const transform read    = m_frame == matrix_frame::world ? m_stencil.world_pose(index) : m_stencil.pose(index);

    return scene::readout{std::string(), rows_of(read, m_form == matrix_form::rotation ? 3 : 4)};
}

}
