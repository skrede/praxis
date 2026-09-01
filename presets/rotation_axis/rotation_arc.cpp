#include "rotation_arc.h"

#include <threepp/objects/Line.hpp>

#include <threepp/core/BufferGeometry.hpp>

#include <threepp/materials/LineBasicMaterial.hpp>

#include <Eigen/Core>

#include <cmath>
#include <memory>
#include <vector>
#include <cstddef>
#include <numbers>
#include <algorithm>

namespace praxis::presets {

namespace {

// Samples per pi radians of turn, so the chord error is bounded however far the turn is driven.
constexpr std::size_t samples_per_turn = 64;

threepp::Vector3 drawn_at(const Eigen::Vector3d &point)
{
    return threepp::Vector3{static_cast<float>(point.x()), static_cast<float>(point.y()), static_cast<float>(point.z())};
}

std::size_t steps_over(double angle_radians)
{
    const double turns = std::abs(angle_radians) / std::numbers::pi;

    return std::max<std::size_t>(2u, static_cast<std::size_t>(std::llround(turns * static_cast<double>(samples_per_turn))));
}

}

std::shared_ptr<threepp::Object3D> traversed_arc(const rigid_motion::screw_ops &screw, const Eigen::Vector3d &unit_axis, double angle_radians, const transform &start,
                                                 const Eigen::Vector3d &carried, const threepp::Color &tone)
{
    if(angle_radians == 0.0)
        return nullptr;

    const std::size_t steps      = steps_over(angle_radians);
    const Eigen::Vector3d seated = start.topLeftCorner<3, 3>() * carried + start.block<3, 1>(0, 3);
    const auto over              = static_cast<double>(steps);

    std::vector<threepp::Vector3> points;
    points.reserve(steps + 1u);
    for(std::size_t step = 0; step <= steps; ++step)
        points.push_back(drawn_at(screw.matrix_exponential_so3(unit_axis, static_cast<double>(step) / over * angle_radians) * seated));

    auto drawn = threepp::BufferGeometry::create();
    drawn->setFromPoints(points);

    return threepp::Line::create(drawn, threepp::LineBasicMaterial::create({{"color", tone}}));
}

}
