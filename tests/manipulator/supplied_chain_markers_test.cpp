#include "fixtures.h"
#include "drawn_chain.h"
#include "window_stage.h"

#include "praxis/manipulator/arm_snapshot.h"
#include "praxis/manipulator/supplied_chain.h"
#include "praxis/manipulator/loadable_robot_stencil.h"

#include "praxis/scheduler/scheduler.h"

#include "praxis/rigid_motion/capabilities.h"

#include <catch2/catch_test_macros.hpp>

#include <threepp/scenes/Scene.hpp>

#include <threepp/core/Object3D.hpp>

#include <threepp/math/Matrix4.hpp>
#include <threepp/math/Vector3.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <array>
#include <cmath>
#include <memory>
#include <string>
#include <vector>
#include <cstddef>
#include <numbers>
#include <algorithm>

using namespace praxis::fixture;
using namespace praxis::scheduler;
using namespace praxis::manipulator;

namespace {

constexpr double single_precision_tolerance = 1.0e-5;

// Only the configuration and the tool offset are read by what is under test.
arm_snapshot carrying(const joint_vector &joints, const praxis::transform &offset)
{
    arm_snapshot seen = at_rest(joints, Eigen::Vector3d(Eigen::Vector3d::Zero()), praxis::rotation(praxis::rotation::Identity()));
    seen.tool_offset  = offset;

    return seen;
}

praxis::screw_axis revolute_screw(double through_x)
{
    return praxis::rigid_motion::baseline().screw.screw_axis_from_point_direction_pitch(Eigen::Vector3d(through_x, 0.0, 0.0), Eigen::Vector3d::UnitZ(), 0.0).value();
}

// The fixture arm's own geometry: z through the origin, and z through the elbow a link length along x.
std::vector<praxis::screw_axis> two_axes()
{
    return {revolute_screw(0.0), revolute_screw(static_cast<double>(link_length))};
}

praxis::transform translated(double x, double y, double z)
{
    praxis::transform placed = praxis::transform::Identity();
    placed.block<3, 1>(0, 3) = Eigen::Vector3d(x, y, z);

    return placed;
}

// Turned a quarter turn about z and displaced, so it stands nowhere the rendered flange does.
praxis::transform displaced_home()
{
    praxis::transform home     = translated(0.1, 0.2, 0.3);
    home.topLeftCorner<3, 3>() = Eigen::AngleAxisd(std::numbers::pi / 2.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();

    return home;
}

// No coordinate translation and no coordinate rotation, so a marker left at the flange differs from
// one carried to the tool frame in position and in every axis direction at once.
praxis::transform turned_tool_offset()
{
    praxis::transform offset     = translated(0.12, -0.07, 0.19);
    offset.topLeftCorner<3, 3>() = Eigen::AngleAxisd(0.7, Eigen::Vector3d(1.0, 1.0, 1.0).normalized()).toRotationMatrix();

    return offset;
}

// Every screw slot bound but `left`, which is taken from a default-constructed aggregate carrying the
// inert implementations.
praxis::rigid_motion::screw_ops bound_without(praxis::rigid_motion::screw_slot left)
{
    const praxis::rigid_motion::screw_ops inert;
    praxis::rigid_motion::screw_ops composed = praxis::rigid_motion::baseline().screw;
    if(left == praxis::rigid_motion::screw_slot::matrix_exponential_screw)
        composed.matrix_exponential_screw = inert.matrix_exponential_screw;
    if(left == praxis::rigid_motion::screw_slot::adjoint_map)
        composed.adjoint_map = inert.adjoint_map;

    return composed;
}

// FK(theta) = e^[S1]theta1 ... e^[Sn]thetan M. Lynch & Park, Modern Robotics, section 4.1.1.
praxis::transform fk(const praxis::transform &home, const std::vector<praxis::screw_axis> &screws, const joint_vector &theta)
{
    praxis::transform carried = praxis::transform::Identity();
    for(std::size_t joint = 0; joint < screws.size(); ++joint)
        carried = carried * praxis::rigid_motion::baseline().screw.matrix_exponential_screw(screws[joint], theta[static_cast<Eigen::Index>(joint)]);

    return carried * home;
}

// The same pose as the renderer holds one: column by column, and in single precision.
threepp::Matrix4 as_the_renderer_holds(const praxis::transform &placed)
{
    std::array<float, 16> held{};
    for(Eigen::Index column = 0; column < 4; ++column)
        for(Eigen::Index row = 0; row < 4; ++row)
            held[static_cast<std::size_t>(4 * column + row)] = static_cast<float>(placed(row, column));

    return threepp::Matrix4(held);
}

threepp::Matrix4 carried_by(const threepp::Matrix4 &base, const threepp::Matrix4 &offset)
{
    threepp::Matrix4 at(base);
    at.multiply(offset);

    return at;
}

threepp::Matrix4 placement_of(threepp::Object3D &drawn)
{
    threepp::Matrix4 placed;
    placed.compose(drawn.position, drawn.quaternion, drawn.scale);

    return placed;
}

// The worst component by which any axis direction of the pose written onto a node departs from the
// rule's.
double axes_departure(threepp::Object3D &drawn, const threepp::Matrix4 &rule)
{
    const threepp::Matrix4 placed = placement_of(drawn);

    double worst = 0.0;
    for(unsigned int axis = 0; axis < 3; ++axis)
    {
        threepp::Vector3 held;
        held.setFromMatrixColumn(rule, axis).normalize();
        threepp::Vector3 shown;
        shown.setFromMatrixColumn(placed, axis).normalize().sub(held);
        worst = std::max({worst, std::abs(static_cast<double>(shown.x)), std::abs(static_cast<double>(shown.y)), std::abs(static_cast<double>(shown.z))});
    }

    return worst;
}

// In metres of position and in the worst component of any axis direction.
double placement_departure(threepp::Object3D &drawn, const threepp::Matrix4 &rule)
{
    threepp::Vector3 place;
    place.setFromMatrixPosition(rule);

    return std::max(static_cast<double>(drawn.position.distanceTo(place)), axes_departure(drawn, rule));
}

Eigen::Vector3d turned_back(const threepp::Matrix4 &placed)
{
    threepp::Vector3 at;
    at.setFromMatrixPosition(placed);

    return {at.x, -at.z, at.y};
}

// A scene needs no graphics context and a renderer robot needs no display, so the whole stage is
// built headlessly. Both frame markers and a bare node under the tool key hang at the flange.
struct stage
{
    explicit stage(const joint_vector &joints)
            : stage(joints, praxis::rigid_motion::baseline().screw, praxis::rigid_motion::screw_slot_set{})
    {
    }

    stage(const joint_vector &joints, const praxis::rigid_motion::screw_ops &turning, praxis::rigid_motion::screw_slot_set inert)
            : at(joints)
            , loop(inline_workers)
            , scene(threepp::Scene::create())
            , published(std::make_shared<arm_publisher>())
            , shown(two_joint_handle(), attachments{}, *scene, loop.main_strand(), published->reader(), turning, inert)
    {
        publish(praxis::transform::Identity());
        REQUIRE(shown.initialize().has_value());
        shown.set_flange_attachment(flange_attachment::frame_marker, make_flange_marker(shown.robot()));
        shown.set_flange_attachment(flange_attachment::tool_frame_marker, make_flange_marker(shown.robot()));
        shown.set_flange_attachment(flange_attachment::tool, threepp::Object3D::create());
    }

    void publish(const praxis::transform &offset)
    {
        published->publish(std::make_shared<const arm_snapshot>(carrying(at, offset)));
    }

    void draw()
    {
        REQUIRE(loop.main_strand().post([this] { shown.render(); }).has_value());
        REQUIRE(loop.drain().has_value());
        scene->updateMatrixWorld(true);
    }

    // The rendered flange is refreshed against its parents' world matrices from the scene update
    // before, so a flange placement agrees with the published configuration from the second draw on.
    void settle()
    {
        draw();
        draw();
    }

    threepp::Object3D &attached(flange_attachment which)
    {
        return *shown.attached_at(which);
    }

    threepp::Matrix4 flange()
    {
        return shown.robot().getEndEffectorTransform();
    }

    // The root-link frame in the scene, composed from the rendered robot node's own placement.
    threepp::Matrix4 root_rule()
    {
        threepp::Matrix4 root;
        root.compose(shown.robot().position, shown.robot().quaternion, shown.robot().scale);

        return root;
    }

    threepp::Matrix4 in_root(const praxis::transform &placed)
    {
        return carried_by(root_rule(), as_the_renderer_holds(placed));
    }

    // How far the frame marker stands from the rendered flange and the tool marker from the rendered
    // flange carried by `offset`, whichever is further.
    double off_the_flange(const praxis::transform &offset)
    {
        return std::max(placement_departure(attached(flange_attachment::frame_marker), flange()),
                        placement_departure(attached(flange_attachment::tool_frame_marker), carried_by(flange(), as_the_renderer_holds(offset))));
    }

    // How far either frame marker stands from the root-link frame's own origin and axes.
    double off_the_root()
    {
        return std::max(placement_departure(attached(flange_attachment::frame_marker), root_rule()), placement_departure(attached(flange_attachment::tool_frame_marker), root_rule()));
    }

    withheld_chain withheld()
    {
        const praxis::expected<chain_end, withheld_chain> end = shown.supplied_chain_end(*published->reader().read());
        REQUIRE_FALSE(end.has_value());

        return end.error();
    }

    joint_vector at;
    scheduler loop;
    std::shared_ptr<threepp::Scene> scene;
    std::shared_ptr<arm_publisher> published;
    loadable_robot_stencil shown;
};

const joint_vector &bent()
{
    static const joint_vector held = configuration(0.4, -0.7);

    return held;
}

}

TEST_CASE("a supplied chain standing where the rendered arm stands puts the flange frame marker on the rendered flange", "[manipulator][supplied]")
{
    stage drawn(bent());
    CHECK_FALSE(drawn.shown.holds_supplied_chain());

    REQUIRE(drawn.shown.supply_joint_screws(translated(static_cast<double>(link_length), 0.0, 0.0), two_axes()).has_value());
    CHECK(drawn.shown.holds_supplied_chain());
    drawn.settle();

    CHECK(placement_departure(drawn.attached(flange_attachment::frame_marker), carried_by(drawn.flange(), threepp::Matrix4())) < single_precision_tolerance);
}

TEST_CASE("a supplied chain places the flange frame marker at the product of its exponentials and its home pose", "[manipulator][supplied]")
{
    stage drawn(bent());
    REQUIRE(drawn.shown.supply_joint_screws(displaced_home(), two_axes()).has_value());
    drawn.settle();

    threepp::Object3D &marker = drawn.attached(flange_attachment::frame_marker);
    CHECK(placement_departure(marker, drawn.in_root(fk(displaced_home(), two_axes(), bent()))) < single_precision_tolerance);
    CHECK(placement_departure(marker, drawn.flange()) > 0.1);
}

TEST_CASE("a supplied chain places the tool frame marker at its flange carried by the published tool offset", "[manipulator][supplied]")
{
    stage drawn(bent());
    drawn.publish(turned_tool_offset());
    REQUIRE(drawn.shown.supply_joint_screws(displaced_home(), two_axes()).has_value());
    drawn.settle();

    const praxis::transform rule = fk(displaced_home(), two_axes(), bent()) * turned_tool_offset();
    CHECK(placement_departure(drawn.attached(flange_attachment::tool_frame_marker), drawn.in_root(rule)) < single_precision_tolerance);
}

TEST_CASE("the tool mesh and the tool stick stay on the rendered flange while a supplied chain moves the markers", "[manipulator][supplied]")
{
    stage drawn(bent());
    drawn.publish(turned_tool_offset());
    REQUIRE(drawn.shown.supply_joint_screws(displaced_home(), two_axes()).has_value());
    drawn.settle();

    CHECK(placement_departure(drawn.attached(flange_attachment::tool), carried_by(drawn.flange(), threepp::Matrix4())) < single_precision_tolerance);

    auto *stick = drawn.scene->getObjectByName<threepp::Object3D>(loadable_robot_stencil::tool_stick_name());
    REQUIRE(stick != nullptr);
    const std::vector<Eigen::Vector3d> ends = segment_ends(*stick);
    CHECK((ends.front() - turned_back(drawn.flange())).norm() < single_precision_tolerance);
    CHECK((ends.back() - turned_back(carried_by(drawn.flange(), as_the_renderer_holds(turned_tool_offset())))).norm() < single_precision_tolerance);
}

TEST_CASE("a supplied chain edited between two frames moves both markers at the next frame", "[manipulator][supplied]")
{
    const praxis::transform edited = translated(0.3, -0.2, 0.1);

    stage drawn(bent());
    drawn.publish(turned_tool_offset());
    REQUIRE(drawn.shown.supply_joint_screws(displaced_home(), two_axes()).has_value());
    drawn.settle();
    REQUIRE(drawn.shown.supply_joint_screws(edited, two_axes()).has_value());
    drawn.draw();

    const praxis::transform flange_rule = fk(edited, two_axes(), bent());
    CHECK(placement_departure(drawn.attached(flange_attachment::frame_marker), drawn.in_root(flange_rule)) < single_precision_tolerance);
    CHECK(placement_departure(drawn.attached(flange_attachment::tool_frame_marker), drawn.in_root(flange_rule * turned_tool_offset())) < single_precision_tolerance);
}

TEST_CASE("a chain told rather than supplied leaves both markers on the rendered flange whatever its home pose", "[manipulator][supplied]")
{
    stage drawn(bent());
    drawn.publish(turned_tool_offset());
    REQUIRE(drawn.shown.set_joint_screws(displaced_home(), two_axes()).has_value());
    drawn.settle();

    CHECK_FALSE(drawn.shown.holds_supplied_chain());
    CHECK(drawn.off_the_flange(turned_tool_offset()) < single_precision_tolerance);
}

TEST_CASE("a chain told or cleared after a supplied one returns both markers to the rendered flange", "[manipulator][supplied]")
{
    stage drawn(bent());
    drawn.publish(turned_tool_offset());
    REQUIRE(drawn.shown.supply_joint_screws(displaced_home(), two_axes()).has_value());
    drawn.settle();
    REQUIRE(drawn.off_the_flange(turned_tool_offset()) > 0.1);

    REQUIRE(drawn.shown.set_joint_screws(displaced_home(), two_axes()).has_value());
    drawn.settle();
    CHECK_FALSE(drawn.shown.holds_supplied_chain());
    CHECK(drawn.off_the_flange(turned_tool_offset()) < single_precision_tolerance);

    REQUIRE(drawn.shown.supply_joint_screws(displaced_home(), two_axes()).has_value());
    drawn.settle();
    drawn.shown.clear_joint_screws();
    drawn.settle();
    CHECK_FALSE(drawn.shown.holds_supplied_chain());
    CHECK(drawn.off_the_flange(turned_tool_offset()) < single_precision_tolerance);
}

// (M R)(R^T T) = M T: a home pose turned by R and a tool offset turned back by R^T name the same tool
// frame through a turned flange frame.
TEST_CASE("turning the home pose and turning the tool offset back leaves the tool frame marker where it stood", "[manipulator][supplied]")
{
    praxis::transform turn     = praxis::transform::Identity();
    turn.topLeftCorner<3, 3>() = Eigen::AngleAxisd(std::numbers::pi / 2.0, Eigen::Vector3d::UnitY()).toRotationMatrix();

    stage drawn(bent());
    drawn.publish(turned_tool_offset());
    REQUIRE(drawn.shown.supply_joint_screws(displaced_home(), two_axes()).has_value());
    drawn.settle();
    const threepp::Matrix4 tool_stood   = placement_of(drawn.attached(flange_attachment::tool_frame_marker));
    const threepp::Matrix4 flange_stood = placement_of(drawn.attached(flange_attachment::frame_marker));

    drawn.publish(praxis::transform(turn.transpose() * turned_tool_offset()));
    REQUIRE(drawn.shown.supply_joint_screws(praxis::transform(displaced_home() * turn), two_axes()).has_value());
    drawn.settle();

    CHECK(placement_departure(drawn.attached(flange_attachment::tool_frame_marker), tool_stood) < single_precision_tolerance);
    CHECK(axes_departure(drawn.attached(flange_attachment::frame_marker), flange_stood) > 0.5);
}

TEST_CASE("a supplied chain naming more screws than the arm has joints parks both markers at the root and says both counts", "[manipulator][supplied]")
{
    std::vector<praxis::screw_axis> three = two_axes();
    three.push_back(revolute_screw(0.2));

    stage drawn(bent());
    drawn.publish(turned_tool_offset());
    const praxis::expected<void, praxis::refusal> told = drawn.shown.supply_joint_screws(displaced_home(), three);
    REQUIRE_FALSE(told.has_value());
    CHECK(told.error() == praxis::refusal::unsupported_input);
    CHECK(drawn.shown.holds_supplied_chain());
    drawn.settle();

    CHECK(drawn.off_the_root() < single_precision_tolerance);
    const withheld_chain why = drawn.withheld();
    CHECK(why.cause == withheld_cause::joint_count);
    CHECK(why.reason == "The supplied chain is not folded: it holds 3 screws and the arm has 2 joints.");
}

TEST_CASE("a supplied chain folded through an unbound screw exponential parks both markers and names the slot", "[manipulator][supplied]")
{
    praxis::rigid_motion::screw_slot_set exponential;
    exponential.set(praxis::rigid_motion::screw_slot::matrix_exponential_screw);

    stage drawn(bent(), bound_without(praxis::rigid_motion::screw_slot::matrix_exponential_screw), exponential);
    drawn.publish(turned_tool_offset());
    REQUIRE(drawn.shown.supply_joint_screws(displaced_home(), two_axes()).has_value());
    drawn.settle();

    CHECK(drawn.off_the_root() < single_precision_tolerance);
    const withheld_chain why = drawn.withheld();
    CHECK(why.cause == withheld_cause::unbound_slot);
    CHECK(why.reason.starts_with("The supplied chain is not folded: '"));
    CHECK(why.reason.find("screw.matrix_exponential_screw") != std::string::npos);
}

TEST_CASE("a supplied chain whose fold refuses a joint parks both markers and names the joint", "[manipulator][supplied]")
{
    stage drawn(bent(), bound_without(praxis::rigid_motion::screw_slot::adjoint_map), praxis::rigid_motion::screw_slot_set{});
    drawn.publish(turned_tool_offset());
    REQUIRE(drawn.shown.supply_joint_screws(displaced_home(), two_axes()).has_value());
    drawn.settle();

    CHECK(drawn.off_the_root() < single_precision_tolerance);
    const withheld_chain why = drawn.withheld();
    CHECK(why.cause == withheld_cause::refused);
    CHECK(why.reason == "The supplied chain is not folded: joint 1's screw was refused.");
}

TEST_CASE("a chain supplied as unbuilt parks both markers and names the slot until a chain is supplied again", "[manipulator][supplied]")
{
    stage drawn(bent());
    drawn.publish(turned_tool_offset());
    drawn.shown.supply_unbuilt_chain(praxis::rigid_motion::screw_slot::screw_axis_from_angular_linear);
    drawn.settle();

    CHECK(drawn.shown.holds_supplied_chain());
    CHECK(drawn.off_the_root() < single_precision_tolerance);
    CHECK(drawn.withheld().reason == "The supplied chain is not folded: 'screw.screw_axis_from_angular_linear' holds its default.");

    REQUIRE(drawn.shown.supply_joint_screws(displaced_home(), two_axes()).has_value());
    drawn.draw();
    const praxis::transform flange_rule = fk(displaced_home(), two_axes(), bent());
    CHECK(placement_departure(drawn.attached(flange_attachment::frame_marker), drawn.in_root(flange_rule)) < single_precision_tolerance);
    CHECK(placement_departure(drawn.attached(flange_attachment::tool_frame_marker), drawn.in_root(flange_rule * turned_tool_offset())) < single_precision_tolerance);
}
