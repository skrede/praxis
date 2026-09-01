#include "rotation_arrow.h"

#include <threepp/objects/Mesh.hpp>
#include <threepp/objects/Group.hpp>

#include <threepp/geometries/ConeGeometry.hpp>
#include <threepp/geometries/CylinderGeometry.hpp>

#include <threepp/materials/Material.hpp>
#include <threepp/materials/MeshPhongMaterial.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <memory>
#include <string>
#include <utility>

namespace praxis::presets {

namespace {

constexpr double head_length           = 0.05; // in metres, whatever the arrow's own length
constexpr double head_radius_multiple  = 1.5;  // of the stem's diameter
constexpr unsigned int radial_segments = 24;

void stretch_stem(threepp::Object3D &stem, double length)
{
    const double height = length - head_length;

    stem.scale.y    = static_cast<float>(height);
    stem.position.y = static_cast<float>(height / 2.0);
    stem.visible    = height > 0.0;
}

void stand_head(threepp::Object3D &head, double length)
{
    if(length <= head_length)
    {
        const auto shrunk = static_cast<float>(length / head_length);

        head.scale.set(shrunk, shrunk, shrunk);
        head.position.y = static_cast<float>(length / 2.0);
        return;
    }

    head.scale.set(1.f, 1.f, 1.f);
    head.position.y = static_cast<float>(length - head_length / 2.0);
}

}

drawn_arrow arrow_object(std::string name, const threepp::Color &tone, double girth)
{
    const auto stem_radius = static_cast<float>(girth / 2.0);
    const auto head_radius = static_cast<float>(girth * head_radius_multiple);

    const std::shared_ptr<threepp::Material> worn = threepp::MeshPhongMaterial::create({{"flatShading", true}, {"color", tone}});

    auto stem  = threepp::Mesh::create(threepp::CylinderGeometry::create(stem_radius, stem_radius, 1.f, radial_segments), worn);
    stem->name = "stem";

    auto head  = threepp::Mesh::create(threepp::ConeGeometry::create(head_radius, static_cast<float>(head_length), radial_segments), worn);
    head->name = "head";

    auto carrier  = threepp::Group::create();
    carrier->name = std::move(name);
    carrier->add(stem);
    carrier->add(head);

    return drawn_arrow{carrier, stem, head};
}

void place_arrow(const drawn_arrow &drawn, const Eigen::Vector3d &along)
{
    const double length = along.norm();
    if(length <= 0.0)
    {
        drawn.object->visible = false;
        return;
    }

    const Eigen::Quaterniond turned = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitY(), along / length);

    drawn.object->quaternion.set(static_cast<float>(turned.x()), static_cast<float>(turned.y()), static_cast<float>(turned.z()), static_cast<float>(turned.w()));
    drawn.object->visible = true;

    stretch_stem(*drawn.stem, length);
    stand_head(*drawn.head, length);
}

}
