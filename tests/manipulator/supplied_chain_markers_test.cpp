#include "fixtures.h"
#include "drawn_chain.h"
#include "window_stage.h"
#include "literal_exponential.h"

#include "praxis/manipulator/arm_snapshot.h"
#include "praxis/manipulator/supplied_chain.h"
#include "praxis/manipulator/loadable_robot_stencil.h"

#include "praxis/scheduler/scheduler.h"

#include "praxis/rigid_motion/capabilities.h"

#include <catch2/catch_test_macros.hpp>

#include <threepp/scenes/Scene.hpp>

#include <threepp/core/Object3D.hpp>

#include <threepp/math/Box3.hpp>
#include <threepp/math/Color.hpp>
#include <threepp/math/Matrix4.hpp>
#include <threepp/math/Vector3.hpp>

#include <threepp/materials/interfaces.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>
#include <cstddef>
#include <numbers>
#include <utility>
#include <algorithm>

using namespace praxis::fixture;
using namespace praxis::scheduler;
using namespace praxis::manipulator;

namespace {

constexpr double single_precision_tolerance = 1.0e-5;

// The size, as a fraction of the arm's extent, and the opacity the marker at the description's flange
// is built at.
constexpr double described_extent_fraction = 0.10;
constexpr float described_opacity          = 0.5f;

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

// The fixture arm's two axes with the angular part of `joint`'s screw scaled by two.
std::vector<praxis::screw_axis> doubled_at(std::size_t joint)
{
    std::vector<praxis::screw_axis> axes = two_axes();
    axes[joint]                          = 2.0 * axes[joint];

    return axes;
}

// Each linear part is -w x q, Modern Robotics, section 3.3.2, with w left at its written length.
std::vector<praxis::screw_axis> four_decimal_axes()
{
    const double reach = static_cast<double>(link_length);
    praxis::screw_axis first;
    first << 0.7071, 0.0, 0.7071, 0.0, 0.0, 0.0;
    praxis::screw_axis second;
    second << 0.0, 0.7071, 0.7071, 0.0, -0.7071 * reach, 0.7071 * reach;

    return {first, second};
}

// An eighth turn about z written to one decimal, so its rotation block is not orthonormal.
praxis::transform one_decimal_home()
{
    praxis::transform rough = translated(0.1, 0.2, 0.3);
    rough.topLeftCorner<2, 2>() << 0.7, -0.7, 0.7, 0.7;

    return rough;
}

praxis::transform home_holding(Eigen::Index row, Eigen::Index column, double entry)
{
    praxis::transform home = displaced_home();
    home(row, column)      = entry;

    return home;
}

// The baseline exponential for a screw whose linear part is zero, and one entry that is not a number
// otherwise.
praxis::transform not_a_number_off_the_origin(const praxis::screw_axis &axis, double theta)
{
    praxis::transform turned = praxis::rigid_motion::baseline().screw.matrix_exponential_screw(axis, theta);
    if(axis.tail<3>().norm() > 0.0)
        turned(0, 3) = std::numeric_limits<double>::quiet_NaN();

    return turned;
}

praxis::rigid_motion::screw_ops exponentiating_to_not_a_number()
{
    praxis::rigid_motion::screw_ops composed = praxis::rigid_motion::baseline().screw;
    composed.matrix_exponential_screw        = &not_a_number_off_the_origin;

    return composed;
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

// The same product with each exponential taken from Modern Robotics, eq. (3.88), as written.
praxis::transform literal_fk(const praxis::transform &home, const std::vector<praxis::screw_axis> &screws, const joint_vector &theta)
{
    praxis::transform carried = praxis::transform::Identity();
    for(std::size_t joint = 0; joint < screws.size(); ++joint)
        carried = carried * book_literal_exponential(screws[joint], theta[static_cast<Eigen::Index>(joint)]);

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
// built headlessly. The three frame markers and a bare node under the tool key hang at the flange.
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
        shown.set_flange_attachment(flange_attachment::described_frame_marker, make_flange_marker(shown.robot()));
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

    threepp::Object3D &chain_figure()
    {
        threepp::Object3D *found = praxis::fixture::chain_node(*scene, loadable_robot_stencil::chain_name());
        REQUIRE(found != nullptr);

        return *found;
    }

    threepp::Object3D &screw_axis_line(std::size_t joint)
    {
        threepp::Object3D *found = scene->getObjectByName(loadable_robot_stencil::joint_axis_name(joint));
        REQUIRE(found != nullptr);

        return *found;
    }

    joint_vector at;
    scheduler loop;
    std::shared_ptr<threepp::Scene> scene;
    std::shared_ptr<arm_publisher> published;
    loadable_robot_stencil shown;
};

double extent_of(threepp::Object3D &measured)
{
    threepp::Box3 box;
    box.setFromObject(measured);

    return static_cast<double>(box.getSize().length());
}

// The material an axis of a marker is drawn in, which its shaft and its tip share.
threepp::MaterialWithColor &worn_by(threepp::Object3D &marker, const std::string &axis)
{
    threepp::Object3D *drawn = marker.getObjectByName(axis);
    REQUIRE(drawn != nullptr);
    auto *worn = drawn->getObjectByName("shaft")->materialAs<threepp::MaterialWithColor>();
    REQUIRE(worn != nullptr);

    return *worn;
}

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

TEST_CASE("a supplied chain whose last exponential is not a rigid transform parks both markers and names that joint", "[manipulator][supplied]")
{
    stage drawn(bent(), exponentiating_literally(), praxis::rigid_motion::screw_slot_set{});
    drawn.publish(turned_tool_offset());
    REQUIRE(drawn.shown.supply_joint_screws(displaced_home(), doubled_at(1)).has_value());
    drawn.settle();

    CHECK(drawn.off_the_root() < single_precision_tolerance);
    const withheld_chain why = drawn.withheld();
    CHECK(why.cause == withheld_cause::refused);
    CHECK(why.reason == "The supplied chain is not folded: the exponential of joint 2's screw is not a rigid transform.");
}

TEST_CASE("a supplied chain whose first exponential is not a rigid transform is withheld naming that joint rather than the next", "[manipulator][supplied]")
{
    stage drawn(bent(), exponentiating_literally(), praxis::rigid_motion::screw_slot_set{});
    drawn.publish(turned_tool_offset());
    REQUIRE(drawn.shown.supply_joint_screws(displaced_home(), doubled_at(0)).has_value());
    drawn.settle();

    CHECK(drawn.off_the_root() < single_precision_tolerance);
    const withheld_chain why = drawn.withheld();
    CHECK(why.cause == withheld_cause::refused);
    CHECK(why.reason == "The supplied chain is not folded: the exponential of joint 1's screw is not a rigid transform.");
}

TEST_CASE("a supplied chain folded with the adjoint map left at its default stands both markers where it ends", "[manipulator][supplied]")
{
    stage drawn(bent(), bound_without(praxis::rigid_motion::screw_slot::adjoint_map), praxis::rigid_motion::screw_slot_set{});
    drawn.publish(turned_tool_offset());
    REQUIRE(drawn.shown.supply_joint_screws(displaced_home(), two_axes()).has_value());
    drawn.settle();

    REQUIRE(drawn.shown.supplied_chain_end(*drawn.published->reader().read()).has_value());
    const praxis::transform flange_rule = fk(displaced_home(), two_axes(), bent());
    CHECK(placement_departure(drawn.attached(flange_attachment::frame_marker), drawn.in_root(flange_rule)) < single_precision_tolerance);
    CHECK(placement_departure(drawn.attached(flange_attachment::tool_frame_marker), drawn.in_root(flange_rule * turned_tool_offset())) < single_precision_tolerance);
}

TEST_CASE("a supplied chain exponentiated as Modern Robotics writes it over unit axes stands both markers where it ends", "[manipulator][supplied]")
{
    stage drawn(bent(), exponentiating_literally(), praxis::rigid_motion::screw_slot_set{});
    drawn.publish(turned_tool_offset());
    REQUIRE(drawn.shown.supply_joint_screws(displaced_home(), two_axes()).has_value());
    drawn.settle();

    const praxis::transform flange_rule = fk(displaced_home(), two_axes(), bent());
    CHECK(placement_departure(drawn.attached(flange_attachment::frame_marker), drawn.in_root(flange_rule)) < single_precision_tolerance);
    CHECK(placement_departure(drawn.attached(flange_attachment::tool_frame_marker), drawn.in_root(flange_rule * turned_tool_offset())) < single_precision_tolerance);
}

TEST_CASE("a supplied chain whose axes are written to four decimals and left unnormalized stands both markers where it ends", "[manipulator][supplied]")
{
    const std::vector<praxis::screw_axis> written = four_decimal_axes();

    stage drawn(bent(), exponentiating_literally(), praxis::rigid_motion::screw_slot_set{});
    drawn.publish(turned_tool_offset());
    REQUIRE(drawn.shown.supply_joint_screws(displaced_home(), written).has_value());
    drawn.settle();

    REQUIRE(drawn.shown.supplied_chain_end(*drawn.published->reader().read()).has_value());
    const praxis::transform flange_rule = literal_fk(displaced_home(), written, bent());
    CHECK(placement_departure(drawn.attached(flange_attachment::frame_marker), drawn.in_root(flange_rule)) < single_precision_tolerance);
    CHECK(placement_departure(drawn.attached(flange_attachment::tool_frame_marker), drawn.in_root(flange_rule * turned_tool_offset())) < single_precision_tolerance);
}

// Position only: a rotation block that is not orthonormal has no exact quaternion to be drawn at.
TEST_CASE("a supplied chain whose home pose is written to one decimal stands both markers where it ends", "[manipulator][supplied]")
{
    const praxis::transform rough = one_decimal_home();

    stage drawn(bent());
    REQUIRE(drawn.shown.supply_joint_screws(rough, two_axes()).has_value());
    drawn.settle();

    REQUIRE(drawn.shown.supplied_chain_end(*drawn.published->reader().read()).has_value());
    threepp::Vector3 place;
    place.setFromMatrixPosition(drawn.in_root(fk(rough, two_axes(), bent())));
    CHECK(static_cast<double>(drawn.attached(flange_attachment::frame_marker).position.distanceTo(place)) < single_precision_tolerance);
}

TEST_CASE("a supplied chain whose home pose is not finite parks both markers and says so", "[manipulator][supplied]")
{
    const std::array<praxis::transform, 2> homes{home_holding(1, 3, std::numeric_limits<double>::quiet_NaN()), home_holding(0, 0, std::numeric_limits<double>::infinity())};
    for(const praxis::transform &home : homes)
    {
        stage drawn(bent());
        drawn.publish(turned_tool_offset());
        REQUIRE(drawn.shown.supply_joint_screws(home, two_axes()).has_value());
        drawn.settle();

        CHECK(drawn.off_the_root() < single_precision_tolerance);
        const withheld_chain why = drawn.withheld();
        CHECK(why.cause == withheld_cause::refused);
        CHECK(why.reason == "The supplied chain is not folded: its home pose is not finite.");
    }
}

TEST_CASE("a supplied chain whose exponential answers an entry that is not a number is withheld naming that joint", "[manipulator][supplied]")
{
    stage drawn(bent(), exponentiating_to_not_a_number(), praxis::rigid_motion::screw_slot_set{});
    drawn.publish(turned_tool_offset());
    REQUIRE(drawn.shown.supply_joint_screws(displaced_home(), two_axes()).has_value());
    drawn.settle();

    CHECK(drawn.off_the_root() < single_precision_tolerance);
    const withheld_chain why = drawn.withheld();
    CHECK(why.cause == withheld_cause::refused);
    CHECK(why.reason == "The supplied chain is not folded: the exponential of joint 2's screw is not a rigid transform.");
}

TEST_CASE("the chain figure and every screw axis are drawn exactly where a supplied chain folds", "[manipulator][supplied]")
{
    struct fold_row
    {
        std::string label;
        praxis::transform home;
        std::vector<praxis::screw_axis> screws;
        praxis::rigid_motion::screw_ops turning;
        bool folds;
    };

    const praxis::rigid_motion::screw_ops literal = exponentiating_literally();
    const praxis::rigid_motion::screw_ops plain   = praxis::rigid_motion::baseline().screw;
    const std::vector<fold_row> rows{
            {"four decimals unnormalized", displaced_home(), four_decimal_axes(), literal, true},
            {"last axis doubled", displaced_home(), doubled_at(1), literal, false},
            {"first axis doubled", displaced_home(), doubled_at(0), literal, false},
            {"home not finite", home_holding(1, 3, std::numeric_limits<double>::quiet_NaN()), two_axes(), plain, false},
            {"exponential not a number", displaced_home(), two_axes(), exponentiating_to_not_a_number(), false},
            {"home to one decimal", one_decimal_home(), two_axes(), plain, true},
            {"unit axes", displaced_home(), two_axes(), literal, true},
    };

    for(const fold_row &row : rows)
    {
        INFO(row.label);
        stage drawn(bent(), row.turning, praxis::rigid_motion::screw_slot_set{});
        REQUIRE(drawn.shown.supply_joint_screws(row.home, row.screws).has_value());
        drawn.settle();

        CHECK(drawn.shown.supplied_chain_end(*drawn.published->reader().read()).has_value() == row.folds);
        CHECK(drawn.chain_figure().visible == row.folds);
        for(std::size_t joint = 0; joint < row.screws.size(); ++joint)
            CHECK(drawn.screw_axis_line(joint).visible == row.folds);
    }
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

TEST_CASE("the marker at the description flange stays hidden until a chain is supplied and its switch is on", "[manipulator][supplied]")
{
    stage drawn(bent());
    threepp::Object3D &reference = drawn.attached(flange_attachment::described_frame_marker);
    drawn.settle();
    CHECK_FALSE(reference.visible);

    drawn.shown.set_described_marker_shown(true);
    drawn.draw();
    CHECK_FALSE(reference.visible);

    drawn.shown.set_described_marker_shown(false);
    REQUIRE(drawn.shown.supply_joint_screws(displaced_home(), two_axes()).has_value());
    drawn.draw();
    CHECK_FALSE(reference.visible);

    drawn.shown.set_described_marker_shown(true);
    drawn.draw();
    CHECK(reference.visible);
}

TEST_CASE("the marker at the description flange stands on the rendered flange while a supplied chain moves the others", "[manipulator][supplied]")
{
    stage drawn(bent());
    drawn.publish(turned_tool_offset());
    drawn.shown.set_described_marker_shown(true);
    REQUIRE(drawn.shown.supply_joint_screws(displaced_home(), two_axes()).has_value());
    drawn.settle();

    threepp::Object3D &reference = drawn.attached(flange_attachment::described_frame_marker);
    CHECK(placement_departure(reference, carried_by(drawn.flange(), threepp::Matrix4())) < single_precision_tolerance);
    CHECK(placement_departure(drawn.attached(flange_attachment::frame_marker), drawn.flange()) > 0.1);

    REQUIRE(drawn.shown.set_marker_scale(2.0).has_value());
    drawn.draw();
    CHECK(reference.scale.x == 2.f);
    CHECK(reference.scale.y == 2.f);
    CHECK(reference.scale.z == 2.f);
}

TEST_CASE("a chain told after a supplied one hides the marker at the description flange", "[manipulator][supplied]")
{
    stage drawn(bent());
    drawn.shown.set_described_marker_shown(true);
    REQUIRE(drawn.shown.supply_joint_screws(displaced_home(), two_axes()).has_value());
    drawn.settle();
    REQUIRE(drawn.attached(flange_attachment::described_frame_marker).visible);

    REQUIRE(drawn.shown.set_joint_screws(displaced_home(), two_axes()).has_value());
    drawn.draw();
    CHECK_FALSE(drawn.attached(flange_attachment::described_frame_marker).visible);
}

TEST_CASE("the marker at the description flange is built at its own size and in its own tone", "[manipulator][supplied]")
{
    stage drawn(bent());
    const std::shared_ptr<threepp::Object3D> reference = make_described_flange_marker(drawn.shown.robot());
    const std::shared_ptr<threepp::Object3D> typed     = make_flange_marker(drawn.shown.robot());
    CHECK(std::abs(extent_of(*reference) / extent_of(*typed) - described_extent_fraction / opening_marker_extent_fraction) < 1.0e-4);

    const std::array<std::pair<std::string, threepp::Color>, 3> hues{
            {{"x", threepp::Color(threepp::Color::red)}, {"y", threepp::Color(threepp::Color::green)}, {"z", threepp::Color(threepp::Color::blue)}}};
    for(const auto &[axis, hue] : hues)
    {
        const threepp::MaterialWithColor &faint = worn_by(*reference, axis);
        CHECK(faint.color == hue);
        CHECK(faint.transparent);
        CHECK(faint.opacity == described_opacity);

        const threepp::MaterialWithColor &plain = worn_by(*typed, axis);
        CHECK(plain.color == hue);
        CHECK_FALSE(plain.transparent);
        CHECK(plain.opacity == 1.f);
    }
}
