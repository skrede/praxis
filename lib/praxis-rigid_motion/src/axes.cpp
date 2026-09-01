#include "praxis/rigid_motion/axes.h"

#include <threepp/objects/Mesh.hpp>
#include <threepp/objects/Group.hpp>

#include <threepp/math/Color.hpp>
#include <threepp/math/MathUtils.hpp>

#include <threepp/geometries/BoxGeometry.hpp>
#include <threepp/geometries/ConeGeometry.hpp>
#include <threepp/geometries/SphereGeometry.hpp>
#include <threepp/geometries/CylinderGeometry.hpp>

#include <threepp/materials/MeshPhongMaterial.hpp>

#include <memory>

namespace praxis::rigid_motion {

namespace {

constexpr double tip_length_fraction   = 0.24;
constexpr double tip_radius_multiple   = 1.5;
constexpr double origin_radius_ratio   = 1.0;
constexpr unsigned int radial_segments = 24;

std::shared_ptr<threepp::Material> lit(const threepp::Color &tone)
{
    return threepp::MeshPhongMaterial::create({{"flatShading", true}, {"color", tone}});
}

// A cylinder and a cone are both built along the renderer's +Y, so an axis is drawn there and the
// group carrying it is turned onto the axis it stands for.
std::shared_ptr<threepp::Object3D> make_axis(const axes_settings &chosen, const threepp::Color &tone)
{
    const auto shaft_radius = static_cast<float>(chosen.axis_thickness / 2.0);
    const auto tip_radius   = static_cast<float>(chosen.axis_thickness * tip_radius_multiple);
    const auto tip_height   = static_cast<float>(chosen.axis_length * tip_length_fraction);
    const auto shaft_height = static_cast<float>(chosen.axis_length) - tip_height;

    const std::shared_ptr<threepp::Material> drawn = lit(tone);

    auto shaft        = threepp::Mesh::create(threepp::CylinderGeometry::create(shaft_radius, shaft_radius, shaft_height, radial_segments), drawn);
    shaft->name       = "shaft";
    shaft->position.y = shaft_height / 2.f;

    auto tip        = threepp::Mesh::create(threepp::ConeGeometry::create(tip_radius, tip_height, radial_segments), drawn);
    tip->name       = "tip";
    tip->position.y = shaft_height + tip_height / 2.f;

    auto axis = threepp::Group::create();
    axis->add(shaft);
    axis->add(tip);

    return axis;
}

std::shared_ptr<threepp::Object3D> turned_axis(const axes_settings &chosen, const threepp::Color &tone, const char *name, float about_x, float about_z)
{
    auto drawn  = make_axis(chosen, tone);
    drawn->name = name;
    drawn->rotation.set(about_x, 0.f, about_z);
    return drawn;
}

std::shared_ptr<threepp::Object3D> origin_marker(const axes_settings &chosen)
{
    auto marker  = threepp::Mesh::create(threepp::SphereGeometry::create(static_cast<float>(chosen.axis_thickness * origin_radius_ratio)), lit(threepp::Color::white));
    marker->name = "origin";
    return marker;
}

std::shared_ptr<threepp::Object3D> cube_body(const object_body &chosen)
{
    const auto edge = static_cast<float>(chosen.cube_edge);

    auto drawn  = threepp::Mesh::create(threepp::BoxGeometry::create(edge, edge, edge), lit(threepp::Color::steelblue));
    drawn->name = "body";

    return drawn;
}

std::shared_ptr<threepp::Object3D> mesh_body(const object_body &chosen)
{
    if(!chosen.mesh)
        return nullptr;

    auto drawn  = threepp::Group::create();
    drawn->name = "body";
    drawn->add(chosen.mesh);

    return drawn;
}

}

std::shared_ptr<threepp::Object3D> make_axes(const axes_settings &chosen, bool with_origin_marker)
{
    auto drawn  = threepp::Group::create();
    drawn->name = "axes";

    drawn->add(turned_axis(chosen, threepp::Color::red, "x", 0.f, -threepp::math::PI / 2.f));
    drawn->add(turned_axis(chosen, threepp::Color::green, "y", 0.f, 0.f));
    drawn->add(turned_axis(chosen, threepp::Color::blue, "z", threepp::math::PI / 2.f, 0.f));

    if(with_origin_marker)
        drawn->add(origin_marker(chosen));

    return drawn;
}

std::shared_ptr<threepp::Object3D> make_body(const object_body &chosen)
{
    switch(chosen.shape)
    {
        case body_shape::cube:
            return cube_body(chosen);
        case body_shape::mesh:
            return mesh_body(chosen);
        case body_shape::none:
            break;
    }

    return nullptr;
}

}
