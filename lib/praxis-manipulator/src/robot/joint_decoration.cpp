#include "robot/joint_decoration.h"

#include "inert_screw_report.h"

#include <threepp/objects/Line.hpp>
#include <threepp/objects/ObjectWithMaterials.hpp>

#include <threepp/math/Box3.hpp>
#include <threepp/math/Color.hpp>
#include <threepp/math/Matrix4.hpp>
#include <threepp/math/Vector3.hpp>

#include <threepp/core/BufferGeometry.hpp>

#include <threepp/materials/LineBasicMaterial.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <array>
#include <vector>
#include <memory>
#include <cstddef>
#include <utility>
#include <optional>

namespace praxis::manipulator {

namespace {

constexpr double axis_reach_fraction          = 1.0;
constexpr double bare_arm_extent              = 1.0; // metres
constexpr threepp::Color::ColorName axis_tone = threepp::Color::orange;

// A drawn axis in the configuration the arm stands at: the direction it runs along and one point it
// passes through.
struct drawn_axis
{
    Eigen::Vector3d along;
    Eigen::Vector3d through;
};

// The renderer stores a transform column by column, and in single precision.
threepp::Matrix4 to_renderer_transform(const transform &tf)
{
    std::array<float, 16> rendered{};
    for(Eigen::Index column = 0; column < 4; ++column)
        for(Eigen::Index row = 0; row < 4; ++row)
            rendered[static_cast<std::size_t>(4 * column + row)] = static_cast<float>(tf(row, column));

    return threepp::Matrix4(rendered);
}

// A screw with an angular part passes through the point of its axis nearest the frame's origin,
// which is the angular direction crossed into the linear part. A screw with no angular part has no
// point on its axis at all, so the indicator is placed at the translation of the pose carrying it.
std::optional<drawn_axis> axis_of(const twist &carried_screw, const transform &carried)
{
    const Eigen::Vector3d angular = carried_screw.head<3>();
    const Eigen::Vector3d linear  = carried_screw.tail<3>();
    if(angular.norm() > angular_epsilon)
        return drawn_axis{angular.normalized(), angular.normalized().cross(linear / angular.norm())};

    if(linear.norm() > angular_epsilon)
        return drawn_axis{linear.normalized(), carried.block<3, 1>(0, 3)};

    return std::nullopt;
}

// A screw axis fixes a line and a direction and no roll about itself, so any rotation taking the
// drawn line's own z onto that direction places it, and the line is symmetric about the point it is
// anchored at.
std::optional<transform> axis_placement(const screw_axis &axis, const transform &carried, const rigid_motion::screw_ops &screw)
{
    const expected<twist, refusal> moved = screw.adjoint_map(axis, carried);
    if(!moved)
        return std::nullopt;

    const std::optional<drawn_axis> named = axis_of(*moved, carried);
    if(!named)
        return std::nullopt;

    transform placed             = transform::Identity();
    placed.topLeftCorner<3, 3>() = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(), named->along).toRotationMatrix();
    placed.block<3, 1>(0, 3)     = named->through;

    return placed;
}

}

double opening_axis_reach(threepp::Object3D *arm)
{
    threepp::Box3 around;
    if(arm != nullptr)
        around.setFromObject(*arm);

    const double extent = around.isEmpty() ? bare_arm_extent : static_cast<double>(around.getSize().length());

    return axis_reach_fraction * extent;
}

// Drawn along the z-axis through the origin, which is the line object's own frame: the object's
// placement is what carries it onto the joint's axis. It runs the same distance either way of that
// placement, so naming a point further along the same axis leaves the drawn line where it was.
std::shared_ptr<threepp::Material> axis_material(bool told)
{
    return threepp::LineBasicMaterial::create({{"color", told ? threepp::Color(selected_joint_tone) : threepp::Color(axis_tone)}});
}

std::shared_ptr<threepp::Object3D> joint_axis_object(std::string name, double reach, std::shared_ptr<threepp::Material> tone)
{
    const std::vector<threepp::Vector3> ends{threepp::Vector3{0.f, 0.f, -static_cast<float>(reach)}, threepp::Vector3{0.f, 0.f, static_cast<float>(reach)}};

    auto geometry = threepp::BufferGeometry::create();
    geometry->setFromPoints(ends);

    auto drawn  = threepp::Line::create(geometry, std::move(tone));
    drawn->name = std::move(name);

    return drawn;
}

void wear(threepp::Object3D &drawn, const std::shared_ptr<threepp::Material> &tone)
{
    if(auto *shaded = dynamic_cast<threepp::ObjectWithMaterials *>(&drawn))
        shaded->setMaterial(tone);
}

void place_joint_axes(std::span<const std::shared_ptr<threepp::Object3D>> drawn, std::span<const screw_axis> space_screws, const joint_vector &theta,
                      const rigid_motion::screw_ops &screw)
{
    transform carried = transform::Identity();
    for(std::size_t joint = 0; joint < drawn.size() && joint < space_screws.size(); ++joint)
    {
        const std::optional<transform> placed = axis_placement(space_screws[joint], carried, screw);
        if(placed)
            write_placement(*drawn[joint], *placed);
        drawn[joint]->visible = placed.has_value();

        const auto at     = static_cast<Eigen::Index>(joint);
        const double turn = at < theta.size() ? theta[at] : 0.0;
        carried           = transform(carried * screw.matrix_exponential_screw(space_screws[joint], turn));
    }
}

bool decline_unbound_fold(std::span<const std::shared_ptr<threepp::Object3D>> drawn, threepp::Object3D *chain, const rigid_motion::screw_ops &screw,
                          const rigid_motion::screw_slot_set &inert, bool &reported)
{
    if(chain == nullptr)
    {
        reported = false;

        return false;
    }

    if(!inert_and_reported(screw, inert, rigid_motion::screw_slot::matrix_exponential_screw, "the joint chain and the screw axes are not drawn", reported))
        return false;

    for(const std::shared_ptr<threepp::Object3D> &line : drawn)
        line->visible = false;
    chain->visible = false;

    return true;
}

void write_placement(threepp::Object3D &node, const transform &placed)
{
    const threepp::Matrix4 put = to_renderer_transform(placed);

    node.position.setFromMatrixPosition(put);
    node.quaternion.setFromRotationMatrix(put);
}

}
