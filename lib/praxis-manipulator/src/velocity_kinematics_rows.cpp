#include "velocity_kinematics_rows.h"

#include "robot/ellipsoid_placement.h"

#include <Eigen/Core>

#include <cmath>
#include <string>
#include <vector>
#include <cstddef>
#include <utility>

namespace praxis::manipulator {

namespace {

using value_row = std::vector<scene::labeled_value>;

constexpr const char *unpublished      = "The arm has published nothing yet.";
constexpr const char *stated_absent    = "absent";
constexpr const char *stated_refused   = "refused";
constexpr const char *stated_unbounded = "unbounded";

const char *frame_word(jacobian_frame frame)
{
    return frame == jacobian_frame::space ? "space" : "body";
}

const char *block_word(jacobian_block which)
{
    return which == jacobian_block::angular ? "Angular" : "Linear";
}

const expected<jacobian, refusal> &matrix_of(const arm_snapshot &seen, jacobian_frame frame)
{
    return frame == jacobian_frame::space ? seen.space_jacobian : seen.body_jacobian;
}

const jacobian_manipulability &decomposition_of(const arm_snapshot &seen, jacobian_frame frame)
{
    return frame == jacobian_frame::space ? seen.space_manipulability : seen.body_manipulability;
}

const expected<manipulability_ellipsoid, refusal> &block_of(const jacobian_manipulability &both, jacobian_block which)
{
    return which == jacobian_block::angular ? both.angular : both.linear;
}

// A value that is not finite reaching a text field is the one thing no cell may carry, so it is
// stated instead of printed.
scene::labeled_value cell(std::string label, double value)
{
    if(!std::isfinite(value))
        return scene::labeled_value{0.0f, std::move(label), stated_absent};

    return scene::labeled_value{static_cast<float>(value), std::move(label), std::string()};
}

std::vector<value_row> rows_of(const jacobian &taken)
{
    std::vector<value_row> rows;
    rows.reserve(static_cast<std::size_t>(taken.rows()));
    for(Eigen::Index row = 0; row < taken.rows(); ++row)
    {
        value_row cells;
        cells.reserve(static_cast<std::size_t>(taken.cols()));
        for(Eigen::Index column = 0; column < taken.cols(); ++column)
            cells.push_back(cell(std::string(), taken(row, column)));

        rows.push_back(std::move(cells));
    }

    return rows;
}

// The block's name stands on the first cell alone, so its three values read as one group under one
// name rather than as three names repeating it.
value_row values_row(const manipulability_ellipsoid &block, jacobian_block which)
{
    value_row cells;
    cells.reserve(3u);
    cells.push_back(cell(block_word(which), block.singular_values[0]));
    for(Eigen::Index axis = 1; axis < 3; ++axis)
        cells.push_back(cell(std::string(), block.singular_values[axis]));

    return cells;
}

// Neither cell carries the block's name: the values row above says which block this is, and this row
// stands directly beneath it.
value_row scalars_row(const manipulability_ellipsoid &block)
{
    value_row cells;
    cells.reserve(2u);
    cells.push_back(cell("Measure", block.measure));
    cells.push_back(block.condition ? cell("Condition", *block.condition) : scene::labeled_value{0.0f, "Condition", stated_absent});

    return cells;
}

// The blocks are appended in the enumeration's own order, so the angular readings always stand above
// the linear ones and which row belongs to which body is read off the labels rather than guessed.
void append_blocks(scene::readout &into, const jacobian_manipulability &both, ellipsoid_view read, double angular_scale, double linear_scale)
{
    const double scale[jacobian_block_count]{angular_scale, linear_scale};
    for(std::size_t index = 0; index < jacobian_block_count; ++index)
    {
        const jacobian_block which                               = static_cast<jacobian_block>(index);
        const expected<manipulability_ellipsoid, refusal> &block = block_of(both, which);
        const std::string named(block_word(which));
        if(!block)
        {
            into.rows.push_back(value_row{scene::labeled_value{0.0f, named, stated_refused}});
            continue;
        }

        value_row beneath = scalars_row(*block);
        if(ellipsoid_unbounded(block, read, scale[index]))
            beneath.push_back(scene::labeled_value{0.0f, named + " ellipsoid", stated_unbounded});

        into.rows.push_back(values_row(*block, which));
        into.rows.push_back(std::move(beneath));
    }
}

}

bool ellipsoid_unbounded(const expected<manipulability_ellipsoid, refusal> &block, ellipsoid_view read, double scale)
{
    return block.has_value() && !drawn_semi_axes(*block, read, scale).allFinite();
}

bool either_ellipsoid_unbounded(const jacobian_manipulability &both, ellipsoid_view read, double angular_scale, double linear_scale)
{
    return ellipsoid_unbounded(both.angular, read, angular_scale) || ellipsoid_unbounded(both.linear, read, linear_scale);
}

scene::readout velocity_kinematics_reading(const arm_snapshot *seen, jacobian_frame frame, ellipsoid_view read, double angular_scale, double linear_scale)
{
    if(seen == nullptr)
        return scene::readout{unpublished, {}};

    const expected<jacobian, refusal> &taken = matrix_of(*seen, frame);
    if(!taken)
        return scene::readout{std::string("The ") + frame_word(frame) + " Jacobian was refused.", {}};

    const jacobian_manipulability &both = decomposition_of(*seen, frame);
    if(!both.angular && !both.linear)
        return scene::readout{std::string("Neither block of the ") + frame_word(frame) + " Jacobian was decomposed.", {}};

    scene::readout answered{std::string(), rows_of(*taken)};
    append_blocks(answered, both, read, angular_scale, linear_scale);

    return answered;
}

}
