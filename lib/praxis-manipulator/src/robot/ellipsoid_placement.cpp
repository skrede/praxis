#include "robot/ellipsoid_figure.h"
#include "robot/joint_decoration.h"
#include "robot/ellipsoid_placement.h"

#include <Eigen/Core>

#include <span>
#include <cmath>
#include <memory>
#include <cstddef>
#include <optional>
#include <algorithm>

namespace praxis::manipulator {

namespace {

// How far a continuation line reaches past the face its axis was cut at, in drawn metres.
constexpr double continuation_length = 0.05;

void wear_step(threepp::Object3D &drawn, std::span<const std::shared_ptr<threepp::Material>> ramp, std::size_t step)
{
    if(!ramp.empty())
        wear(drawn, ramp[std::min(step, ramp.size() - 1u)]);
}

void place_continuations(const Eigen::Vector3d &semi_axes, const Eigen::Matrix3d &axes, const Eigen::Vector3d &at, const std::optional<double> &cap,
                         std::span<const std::shared_ptr<threepp::Object3D>> lines, std::span<const std::shared_ptr<threepp::Material>> tone, std::size_t step)
{
    for(std::size_t axis = 0; axis < 3u; ++axis)
    {
        const Eigen::Index along = static_cast<Eigen::Index>(axis);
        const bool cut           = cap.has_value() && semi_axes[along] > *cap;
        for(std::size_t way = 0; way < 2u; ++way)
        {
            const std::size_t which = 2u * axis + way;
            if(which >= lines.size() || lines[which] == nullptr)
                continue;

            threepp::Object3D &line = *lines[which];
            if(!cut)
            {
                line.visible = false;
                continue;
            }

            const Eigen::Vector3d reach = way == 0u ? Eigen::Vector3d(axes.col(along)) : Eigen::Vector3d(-axes.col(along));
            wear_step(line, tone, step);
            place_continuation(line, Eigen::Vector3d(at + *cap * reach), reach, continuation_length);
        }
    }
}

}

Eigen::Vector3d drawn_semi_axes(const manipulability_ellipsoid &block, ellipsoid_view read, double scale)
{
    if(read == ellipsoid_view::velocity)
        return scale * block.singular_values;

    return Eigen::Vector3d(scale / block.singular_values.array());
}

void hide_ellipsoid_block(threepp::Object3D &body, std::span<const std::shared_ptr<threepp::Object3D>> lines)
{
    body.visible = false;
    for(const std::shared_ptr<threepp::Object3D> &line : lines)
        if(line != nullptr)
            line->visible = false;
}

// The finiteness check runs whether or not a cap was named: lifting the cap removes a bound, not a
// check.
void place_ellipsoid_block(const expected<manipulability_ellipsoid, refusal> &block, const Eigen::Vector3d &at, ellipsoid_view read, double scale, const std::optional<double> &cap,
                           threepp::Object3D &body, std::span<const std::shared_ptr<threepp::Object3D>> lines, const ellipsoid_tones &tone)
{
    if(!block)
    {
        hide_ellipsoid_block(body, lines);
        return;
    }

    const Eigen::Vector3d semi_axes = drawn_semi_axes(*block, read, scale);
    if(!semi_axes.allFinite())
    {
        hide_ellipsoid_block(body, lines);
        return;
    }

    const std::size_t step = ellipsoid_ramp_step(block->condition);
    shape_ellipsoid(body, semi_axes, cap);
    place_ellipsoid(body, block->principal_axes, at);
    wear_step(body, tone.body, step);
    body.visible = true;

    place_continuations(semi_axes, block->principal_axes, at, cap, lines, tone.line, step);
}

}
