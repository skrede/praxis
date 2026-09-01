#include "robot/column_arrow.h"
#include "robot/joint_decoration.h"

#include "praxis/rigid_motion/types.h"

#include <threepp/objects/Mesh.hpp>
#include <threepp/objects/Group.hpp>

#include <threepp/geometries/ConeGeometry.hpp>
#include <threepp/geometries/CylinderGeometry.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <string>
#include <memory>
#include <utility>

namespace praxis::manipulator {

namespace {

constexpr double shaft_girth           = 0.008; // the diameter of a shaft, in metres
constexpr double tip_length            = 0.02;  // in metres, whatever the arrow's own length
constexpr double tip_radius_multiple   = 1.25;  // of the shaft's diameter
constexpr unsigned int radial_segments = 24;

void stretch_shaft(threepp::Object3D &shaft, double length)
{
    const double height = length - tip_length;

    shaft.scale.y    = static_cast<float>(height);
    shaft.position.y = static_cast<float>(height / 2.0);
    shaft.visible    = height > 0.0;
}

void stand_tip(threepp::Object3D &tip, double length)
{
    if(length <= tip_length)
    {
        const auto shrunk = static_cast<float>(length / tip_length);

        tip.scale.set(shrunk, shrunk, shrunk);
        tip.position.y = static_cast<float>(length / 2.0);
        return;
    }

    tip.scale.set(1.f, 1.f, 1.f);
    tip.position.y = static_cast<float>(length - tip_length / 2.0);
}

}

drawn_arrow arrow_object(std::string name, std::shared_ptr<threepp::Material> tone)
{
    const auto shaft_radius = static_cast<float>(shaft_girth / 2.0);
    const auto tip_radius   = static_cast<float>(shaft_girth * tip_radius_multiple);

    auto shaft  = threepp::Mesh::create(threepp::CylinderGeometry::create(shaft_radius, shaft_radius, 1.f, radial_segments), tone);
    shaft->name = "shaft";

    auto tip  = threepp::Mesh::create(threepp::ConeGeometry::create(tip_radius, static_cast<float>(tip_length), radial_segments), std::move(tone));
    tip->name = "tip";

    auto carrier  = threepp::Group::create();
    carrier->name = std::move(name);
    carrier->add(shaft);
    carrier->add(tip);

    return drawn_arrow{carrier, shaft, tip};
}

void place_arrow(const drawn_arrow &drawn, const Eigen::Vector3d &from, const Eigen::Vector3d &along, double length)
{
    const double reach = along.norm();
    if(reach <= 0.0 || length <= 0.0)
    {
        drawn.object->visible = false;
        return;
    }

    transform placed             = transform::Identity();
    placed.topLeftCorner<3, 3>() = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitY(), along / reach).toRotationMatrix();
    placed.block<3, 1>(0, 3)     = from;

    write_placement(*drawn.object, placed);
    drawn.object->visible = true;

    stretch_shaft(*drawn.shaft, length);
    stand_tip(*drawn.tip, length);
}

}
