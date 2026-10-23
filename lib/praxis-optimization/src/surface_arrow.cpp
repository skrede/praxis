#include "surface_arrow.h"
#include "scenario_defaults.h"

#include <threepp/math/Color.hpp>
#include <threepp/math/Vector3.hpp>

#include <threepp/geometries/ConeGeometry.hpp>
#include <threepp/geometries/CylinderGeometry.hpp>

#include <threepp/materials/MeshPhongMaterial.hpp>

#include <memory>
#include <string>
#include <cstdint>
#include <utility>
#include <algorithm>

namespace praxis::optimization {

namespace {

constexpr unsigned int arrow_segments = 24;

}

surface_arrow arrow_object(std::string name, std::uint32_t tone)
{
    auto material   = threepp::MeshPhongMaterial::create();
    material->color = threepp::Color(tone);

    const auto shaft_radius = static_cast<float>(gradient_shaft_radius);
    auto shaft              = threepp::Mesh::create(threepp::CylinderGeometry::create(shaft_radius, shaft_radius, 1.f, arrow_segments), material);
    auto head   = threepp::Mesh::create(threepp::ConeGeometry::create(static_cast<float>(gradient_head_radius), static_cast<float>(gradient_head_length), arrow_segments), material);
    shaft->name = "shaft";
    head->name  = "head";

    auto carrier  = threepp::Group::create();
    carrier->name = std::move(name);
    carrier->add(shaft);
    carrier->add(head);

    return surface_arrow{carrier, shaft, head};
}

void place_arrow(const surface_arrow &drawn, const Eigen::Vector3d &from, const Eigen::Vector3d &along, double length)
{
    const Eigen::Vector3d unit = along.normalized();
    drawn.object->quaternion.setFromUnitVectors(threepp::Vector3(0.f, 1.f, 0.f),
                                                threepp::Vector3(static_cast<float>(unit.x()), static_cast<float>(unit.y()), static_cast<float>(unit.z())));
    drawn.object->position.set(static_cast<float>(from.x()), static_cast<float>(from.y()), static_cast<float>(from.z()));

    const double shaft      = std::max(length - gradient_head_length, 0.0);
    drawn.shaft->scale.y    = static_cast<float>(shaft);
    drawn.shaft->position.y = static_cast<float>(shaft / 2.0);
    drawn.shaft->visible    = shaft > 0.0;

    const auto shrunk = static_cast<float>(std::min(length / gradient_head_length, 1.0));
    drawn.head->scale.set(shrunk, shrunk, shrunk);
    drawn.head->position.y = static_cast<float>(shaft + shrunk * gradient_head_length / 2.0);
}

std::shared_ptr<threepp::Mesh> rod_object(std::string name, std::uint32_t tone, double radius)
{
    auto material   = threepp::MeshPhongMaterial::create();
    material->color = threepp::Color(tone);

    const auto across = static_cast<float>(radius);
    auto rod          = threepp::Mesh::create(threepp::CylinderGeometry::create(across, across, 1.f, arrow_segments), material);
    rod->name         = std::move(name);

    return rod;
}

void place_rod(threepp::Mesh &rod, const Eigen::Vector3d &from, const Eigen::Vector3d &to)
{
    const Eigen::Vector3d along = to - from;
    const double length         = along.norm();
    rod.visible                 = length > 0.0;
    if(!rod.visible)
        return;

    const Eigen::Vector3d unit   = along / length;
    const Eigen::Vector3d middle = (from + to) / 2.0;
    rod.quaternion.setFromUnitVectors(threepp::Vector3(0.f, 1.f, 0.f), threepp::Vector3(static_cast<float>(unit.x()), static_cast<float>(unit.y()), static_cast<float>(unit.z())));
    rod.position.set(static_cast<float>(middle.x()), static_cast<float>(middle.y()), static_cast<float>(middle.z()));
    rod.scale.y = static_cast<float>(length);
}

}
