#include "chain_translation.h"

#include <span>
#include <limits>
#include <vector>
#include <cstddef>
#include <utility>
#include <optional>

namespace praxis::manipulator {

namespace {

std::vector<cartan::joint_limits<double>> to_cartan_bounds(const joint_limits &bounds, std::size_t count)
{
    constexpr double unbounded = std::numeric_limits<double>::infinity();
    const auto free_joint      = cartan::joint_limits<double>::make(-unbounded, unbounded).value();

    std::vector<cartan::joint_limits<double>> converted;
    converted.reserve(count);
    for(std::size_t i = 0; i < count; ++i)
    {
        // A bound pair the validated factory refuses -- reversed, or nonfinite in a direction that
        // describes no interval -- is treated exactly as a missing one: the joint is left free.
        const auto index  = static_cast<Eigen::Index>(i);
        const double low  = index < bounds.lower_position.size() ? bounds.lower_position[index] : -unbounded;
        const double high = index < bounds.upper_position.size() ? bounds.upper_position[index] : unbounded;
        const auto pair   = cartan::joint_limits<double>::make(low, high);
        converted.push_back(pair.has_value() ? pair.value() : free_joint);
    }

    return converted;
}

}

std::optional<chain_type> to_cartan_chain(const screw_chain &chain)
{
    const auto home = cartan::se3<double>::from_matrix(chain.home);
    if(!home.has_value() || chain.space_screws.empty())
        return std::nullopt;

    std::vector<cartan::screw_axis<double>> axes;
    axes.reserve(chain.space_screws.size());
    for(const screw_axis &s : chain.space_screws)
    {
        const auto axis = cartan::screw_axis<double>::from_vector(s);
        if(!axis.has_value())
            return std::nullopt;
        axes.push_back(axis.value());
    }

    auto bounds = to_cartan_bounds(chain.limits, axes.size());

    return chain_type(home.value(), std::move(axes), std::move(bounds));
}

expected<std::vector<screw_axis>, refusal> to_body_screws(const rigid_motion::screw_ops &screw, const transform &m, std::span<const screw_axis> space_screws)
{
    const rotation transposed                     = m.block<3, 3>(0, 0).transpose();
    const expected<adjoint, refusal> inverse_home = screw.adjoint_matrix_from_rotation_position(transposed, -(transposed * m.block<3, 1>(0, 3)));
    if(!inverse_home)
        return unexpected(inverse_home.error());

    std::vector<screw_axis> body;
    body.reserve(space_screws.size());
    for(const screw_axis &s : space_screws)
        body.push_back(*inverse_home * s);

    return body;
}

refusal refusal_from(cartan::chain_failure failure)
{
    switch(failure)
    {
        case cartan::chain_failure::dimension_mismatch:
            return refusal::unsupported_input;
        case cartan::chain_failure::tag_axis_contradiction:
        case cartan::chain_failure::reversed_position_bounds:
        case cartan::chain_failure::negative_velocity_limit:
        case cartan::chain_failure::negative_effort_limit:
        case cartan::chain_failure::negative_acceleration_limit:
        case cartan::chain_failure::non_finite_input:
            return refusal::degenerate;
    }

    return refusal::degenerate;
}

}
