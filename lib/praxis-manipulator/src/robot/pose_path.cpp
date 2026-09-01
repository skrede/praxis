#include "robot/pose_path.h"

#include <threepp/objects/Line.hpp>

#include <threepp/math/Color.hpp>
#include <threepp/math/Vector3.hpp>

#include <threepp/core/BufferGeometry.hpp>

#include <threepp/materials/LineBasicMaterial.hpp>

#include <Eigen/Core>

#include <span>
#include <memory>
#include <string>
#include <vector>
#include <utility>

namespace praxis::manipulator {

namespace {

constexpr threepp::Color::ColorName path_tone = threepp::Color::deepskyblue;

}

threepp::Color opening_path_tone()
{
    return threepp::Color(path_tone);
}

// A line's width is not settable through any OpenGL core profile, so the drawing is one pixel wide
// whatever is asked of the material, and the tone is the only thing carrying it.
std::shared_ptr<threepp::Object3D> pose_path_object(std::string name, std::span<const Eigen::Vector3d> through, threepp::Color tone)
{
    if(through.size() < 2u)
        return nullptr;

    std::vector<threepp::Vector3> points;
    points.reserve(through.size());
    for(const Eigen::Vector3d &at : through)
        points.emplace_back(static_cast<float>(at.x()), static_cast<float>(at.y()), static_cast<float>(at.z()));

    auto geometry = threepp::BufferGeometry::create();
    geometry->setFromPoints(points);

    auto drawn  = threepp::Line::create(geometry, threepp::LineBasicMaterial::create({{"color", tone}}));
    drawn->name = std::move(name);

    return drawn;
}

}
