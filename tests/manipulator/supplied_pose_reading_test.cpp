#include "fixtures.h"
#include "drawn_chain.h"
#include "imgui_frame.h"
#include "window_stage.h"
#include "literal_exponential.h"

#include "praxis/manipulator/arm_snapshot.h"
#include "praxis/manipulator/pose_readout.h"
#include "praxis/manipulator/supplied_chain.h"
#include "praxis/manipulator/loadable_robot_stencil.h"

#include "praxis/scene/labeled_value_window.h"

#include "praxis/scheduler/scheduler.h"

#include "praxis/rigid_motion/angles.h"
#include "praxis/rigid_motion/capabilities.h"

#include <catch2/catch_test_macros.hpp>

#include <imgui.h>

#include <threepp/scenes/Scene.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cmath>
#include <limits>
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

constexpr double position_tolerance = 1.0e-5;
constexpr double degrees_tolerance  = 1.0e-3;

const Eigen::Vector3d published_at(0.7, -0.8, 0.9);

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

praxis::transform displaced_home()
{
    praxis::transform home     = translated(0.1, 0.2, 0.3);
    home.topLeftCorner<3, 3>() = Eigen::AngleAxisd(std::numbers::pi / 2.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();

    return home;
}

praxis::transform turned_tool_offset()
{
    praxis::transform offset     = translated(0.12, -0.07, 0.19);
    offset.topLeftCorner<3, 3>() = Eigen::AngleAxisd(0.7, Eigen::Vector3d(1.0, 1.0, 1.0).normalized()).toRotationMatrix();

    return offset;
}

// FK(theta) = e^[S1]theta1 ... e^[Sn]thetan M. Lynch & Park, Modern Robotics, section 4.1.1.
praxis::transform fk(const praxis::transform &home, const std::vector<praxis::screw_axis> &screws, const joint_vector &theta)
{
    praxis::transform carried = praxis::transform::Identity();
    for(std::size_t joint = 0; joint < screws.size(); ++joint)
        carried = carried * praxis::rigid_motion::baseline().screw.matrix_exponential_screw(screws[joint], theta[static_cast<Eigen::Index>(joint)]);

    return carried * home;
}

const joint_vector &bent()
{
    static const joint_vector held = configuration(0.4, -0.7);

    return held;
}

// Whether the readout's three rows are the pose's position and its Euler angles under the readout's
// opening order, in degrees.
bool reads_at(const praxis::scene::readout &shown, const praxis::transform &pose)
{
    if(!shown.message.empty() || shown.rows.size() != 3u)
        return false;

    const praxis::rotation turned = pose.topLeftCorner<3, 3>();
    const Eigen::Vector3d angles  = praxis::rigid_motion::baseline().frame.euler_from_rotation_matrix(turned, praxis::axis_order::zyx);

    for(Eigen::Index axis = 0; axis < 3; ++axis)
    {
        const std::vector<praxis::scene::labeled_value> &row = shown.rows[static_cast<std::size_t>(axis)];
        if(std::abs(static_cast<double>(row[0].value) - pose(axis, 3)) >= position_tolerance)
            return false;
        if(std::abs(static_cast<double>(row[1].value) - praxis::to_degrees(angles[axis])) >= degrees_tolerance)
            return false;
    }

    return true;
}

Eigen::Vector3d position_read(const praxis::scene::readout &shown)
{
    REQUIRE(shown.rows.size() >= 3u);

    return {shown.rows[0][0].value, shown.rows[1][0].value, shown.rows[2][0].value};
}

// The Frame cycle is the readout's first control, so the cursor stands on it at the top of the panel.
void view_the_flange(pose_readout &readout)
{
    praxis::tests::imgui_frame frames;
    const drawing draw = [&readout]
    {
        ImGui::Begin("Pose");
        readout.render_controls();
        ImGui::End();
    };

    stand_below_top(frames, draw, 0);
    take_next_entry(frames, draw);
}

// One drawing over the fixture arm carrying a frame marker at its flange, and a readout handed it.
struct stage
{
    explicit stage(robot_slot_set inert = robot_slot_set())
            : stage(praxis::rigid_motion::baseline().screw, inert)
    {
    }

    explicit stage(pose_readout::offered_frames offered)
            : stage(praxis::rigid_motion::baseline().screw, robot_slot_set(), offered)
    {
    }

    explicit stage(const praxis::rigid_motion::screw_ops &turning, robot_slot_set inert = robot_slot_set(), pose_readout::offered_frames offered = pose_readout::offered_frames::both)
            : loop(inline_workers)
            , scene(threepp::Scene::create())
            , published(std::make_shared<arm_publisher>())
            , shown(two_joint_handle(), attachments{}, *scene, loop.main_strand(), published->reader(), turning, praxis::rigid_motion::screw_slot_set{})
            , readout(published->reader(), praxis::rigid_motion::baseline().frame, inert, shown, offered)
    {
        publish(turned_tool_offset());
        REQUIRE(shown.initialize().has_value());
        shown.set_flange_attachment(flange_attachment::frame_marker, make_flange_marker(shown.robot()));
    }

    void publish(const praxis::transform &offset)
    {
        arm_snapshot seen = at_rest(bent(), published_at, praxis::rotation(praxis::rotation::Identity()));
        seen.tool_offset  = offset;
        published->publish(std::make_shared<const arm_snapshot>(seen));
    }

    void draw()
    {
        REQUIRE(loop.main_strand().post([this] { shown.render(); }).has_value());
        REQUIRE(loop.drain().has_value());
        scene->updateMatrixWorld(true);
    }

    scheduler loop;
    std::shared_ptr<threepp::Scene> scene;
    std::shared_ptr<arm_publisher> published;
    loadable_robot_stencil shown;
    pose_readout readout;
};

}

TEST_CASE("a pose readout handed a drawing holding a supplied chain reads the tool where that chain ends carried by the published tool offset", "[manipulator][supplied]")
{
    stage drawn;
    REQUIRE(drawn.shown.supply_joint_screws(displaced_home(), two_axes()).has_value());

    CHECK(reads_at(drawn.readout.reading(), fk(displaced_home(), two_axes(), bent()) * turned_tool_offset()));
    CHECK_FALSE(reads_at(drawn.readout.reading(), translated(published_at.x(), published_at.y(), published_at.z())));
}

TEST_CASE("a pose readout switched to the flange reads the flange a supplied chain ends at", "[manipulator][supplied]")
{
    stage drawn;
    REQUIRE(drawn.shown.supply_joint_screws(displaced_home(), two_axes()).has_value());
    view_the_flange(drawn.readout);

    const praxis::transform flange = fk(displaced_home(), two_axes(), bent());
    CHECK(reads_at(drawn.readout.reading(), flange));
    CHECK_FALSE(reads_at(drawn.readout.reading(), flange * turned_tool_offset()));
}

TEST_CASE("a pose readout handed a drawing holding no supplied chain reads the published pose", "[manipulator][supplied]")
{
    const praxis::transform published_pose = translated(published_at.x(), published_at.y(), published_at.z());

    stage drawn;
    CHECK(reads_at(drawn.readout.reading(), published_pose));

    REQUIRE(drawn.shown.set_joint_screws(displaced_home(), two_axes()).has_value());
    CHECK_FALSE(drawn.shown.holds_supplied_chain());
    CHECK(reads_at(drawn.readout.reading(), published_pose));
}

TEST_CASE("a pose readout over a supplied chain that cannot be folded reads zeros and a line saying why", "[manipulator][supplied]")
{
    std::vector<praxis::screw_axis> three = two_axes();
    three.push_back(revolute_screw(0.2));

    stage drawn;
    REQUIRE_FALSE(drawn.shown.supply_joint_screws(displaced_home(), three).has_value());
    const praxis::expected<chain_end, withheld_chain> end = drawn.shown.supplied_chain_end(*drawn.published->reader().read());
    REQUIRE_FALSE(end.has_value());

    const praxis::scene::readout shown = drawn.readout.reading();
    CHECK(shown.message.empty());
    REQUIRE(shown.rows.size() == 4u);
    for(std::size_t row = 0; row < 3u; ++row)
        CHECK(std::all_of(shown.rows[row].begin(), shown.rows[row].end(), [](const praxis::scene::labeled_value &cell) { return cell.value == 0.f; }));
    REQUIRE(shown.rows[3].size() == 1u);
    CHECK(shown.rows[3][0].label.empty());
    CHECK_FALSE(end.error().reason.empty());
    CHECK(shown.rows[3][0].stated == end.error().reason);
}

TEST_CASE("a supplied chain whose last exponential is not a rigid transform reads zeros naming that joint and withholds the flange frame marker", "[manipulator][supplied]")
{
    const std::vector<praxis::screw_axis> doubled{revolute_screw(0.0), praxis::screw_axis(2.0 * revolute_screw(static_cast<double>(link_length)))};

    stage drawn(exponentiating_literally());
    REQUIRE(drawn.shown.supply_joint_screws(displaced_home(), doubled).has_value());
    drawn.draw();

    const praxis::scene::readout shown = drawn.readout.reading();
    CHECK(shown.message.empty());
    REQUIRE(shown.rows.size() == 4u);
    for(std::size_t row = 0; row < 3u; ++row)
        CHECK(std::all_of(shown.rows[row].begin(), shown.rows[row].end(), [](const praxis::scene::labeled_value &cell) { return cell.value == 0.f; }));
    REQUIRE(shown.rows[3].size() == 1u);
    CHECK(shown.rows[3][0].label.empty());
    CHECK(shown.rows[3][0].stated == "The supplied chain is not folded: the exponential of joint 2's screw is not a rigid transform.");

    const std::shared_ptr<threepp::Object3D> marker = drawn.shown.attached_at(flange_attachment::frame_marker);
    REQUIRE(marker != nullptr);
    CHECK_FALSE(marker->visible);
}

TEST_CASE("a supplied chain whose home pose is not finite reads zeros saying so and withholds the flange frame marker", "[manipulator][supplied]")
{
    praxis::transform home = displaced_home();
    home(1, 3)             = std::numeric_limits<double>::quiet_NaN();

    stage drawn;
    REQUIRE(drawn.shown.supply_joint_screws(home, two_axes()).has_value());
    drawn.draw();

    const praxis::scene::readout shown = drawn.readout.reading();
    CHECK(shown.message.empty());
    REQUIRE(shown.rows.size() == 4u);
    for(std::size_t row = 0; row < 3u; ++row)
        CHECK(std::all_of(shown.rows[row].begin(), shown.rows[row].end(), [](const praxis::scene::labeled_value &cell) { return cell.value == 0.f; }));
    REQUIRE(shown.rows[3].size() == 1u);
    CHECK(shown.rows[3][0].label.empty());
    CHECK(shown.rows[3][0].stated == "The supplied chain is not folded: its home pose is not finite.");

    const std::shared_ptr<threepp::Object3D> marker = drawn.shown.attached_at(flange_attachment::frame_marker);
    REQUIRE(marker != nullptr);
    CHECK_FALSE(marker->visible);
}

TEST_CASE("a pose readout over a supplied chain folded with the adjoint map left at its default reads where the chain ends while the drawing withholds the flange frame marker",
          "[manipulator][supplied]")
{
    praxis::rigid_motion::screw_ops turning = praxis::rigid_motion::baseline().screw;
    turning.adjoint_map                     = praxis::rigid_motion::screw_ops{}.adjoint_map;

    stage drawn(turning);
    REQUIRE(drawn.shown.supply_joint_screws(displaced_home(), two_axes()).has_value());
    drawn.draw();

    CHECK(reads_at(drawn.readout.reading(), fk(displaced_home(), two_axes(), bent()) * turned_tool_offset()));
    const std::shared_ptr<threepp::Object3D> marker = drawn.shown.attached_at(flange_attachment::frame_marker);
    REQUIRE(marker != nullptr);
    CHECK_FALSE(marker->visible);
}

TEST_CASE("a pose readout reads a supplied chain whatever robot slots the composition left unbound", "[manipulator][supplied]")
{
    robot_slot_set unbound;
    unbound.set(robot_slot::position_from_pose);
    unbound.set(robot_slot::orientation_from_pose);

    stage drawn(unbound);
    REQUIRE(drawn.shown.supply_joint_screws(displaced_home(), two_axes()).has_value());

    CHECK(reads_at(drawn.readout.reading(), fk(displaced_home(), two_axes(), bent()) * turned_tool_offset()));
}

// (M R)(R^T T) = M T: a home pose turned by R and a tool offset turned back by R^T name the same tool
// frame through a turned flange frame.
TEST_CASE("turning the home pose and turning the tool offset back leaves the tool reading where it stood", "[manipulator][supplied]")
{
    praxis::transform turn       = praxis::transform::Identity();
    turn.topLeftCorner<3, 3>()   = Eigen::AngleAxisd(std::numbers::pi / 2.0, Eigen::Vector3d::UnitY()).toRotationMatrix();
    const praxis::transform tool = fk(displaced_home(), two_axes(), bent()) * turned_tool_offset();

    stage drawn;
    REQUIRE(drawn.shown.supply_joint_screws(displaced_home(), two_axes()).has_value());
    REQUIRE(reads_at(drawn.readout.reading(), tool));

    drawn.publish(praxis::transform(turn.transpose() * turned_tool_offset()));
    REQUIRE(drawn.shown.supply_joint_screws(praxis::transform(displaced_home() * turn), two_axes()).has_value());
    CHECK(reads_at(drawn.readout.reading(), tool));

    view_the_flange(drawn.readout);
    CHECK(reads_at(drawn.readout.reading(), fk(praxis::transform(displaced_home() * turn), two_axes(), bent())));
    CHECK_FALSE(reads_at(drawn.readout.reading(), fk(displaced_home(), two_axes(), bent())));
}

TEST_CASE("a pose readout reads a supplied chain edited between two readings at the second", "[manipulator][supplied]")
{
    const praxis::transform edited = translated(0.3, -0.2, 0.1);

    stage drawn;
    REQUIRE(drawn.shown.supply_joint_screws(displaced_home(), two_axes()).has_value());
    REQUIRE(reads_at(drawn.readout.reading(), fk(displaced_home(), two_axes(), bent()) * turned_tool_offset()));

    REQUIRE(drawn.shown.supply_joint_screws(edited, two_axes()).has_value());
    CHECK(reads_at(drawn.readout.reading(), fk(edited, two_axes(), bent()) * turned_tool_offset()));
}

TEST_CASE("the pose readout and the flange frame marker stand at the same place after a frame", "[manipulator][supplied]")
{
    stage drawn;
    REQUIRE(drawn.shown.supply_joint_screws(displaced_home(), two_axes()).has_value());
    drawn.draw();
    view_the_flange(drawn.readout);

    const std::shared_ptr<threepp::Object3D> marker = drawn.shown.attached_at(flange_attachment::frame_marker);
    REQUIRE(marker != nullptr);
    CHECK((position_read(drawn.readout.reading()) - mark_in_world(*marker)).norm() < position_tolerance);
    CHECK((mark_in_world(*marker) - published_at).norm() > 0.1);
}

TEST_CASE("a readout composed without a drawing reads the published pose while the drawing holds a supplied chain", "[manipulator][supplied]")
{
    stage drawn;
    REQUIRE(drawn.shown.supply_joint_screws(displaced_home(), two_axes()).has_value());
    const pose_readout told(drawn.published->reader(), praxis::rigid_motion::baseline().frame, robot_slot_set());

    CHECK(reads_at(told.reading(), translated(published_at.x(), published_at.y(), published_at.z())));
    CHECK(reads_at(drawn.readout.reading(), fk(displaced_home(), two_axes(), bent()) * turned_tool_offset()));
}

TEST_CASE("a pose readout handed a drawing reads nothing while the arm has published nothing", "[manipulator][supplied]")
{
    stage drawn;
    REQUIRE(drawn.shown.supply_joint_screws(displaced_home(), two_axes()).has_value());

    const arm_publisher nothing;
    const pose_readout readout(nothing.reader(), praxis::rigid_motion::baseline().frame, robot_slot_set(), drawn.shown);
    const praxis::scene::readout shown = readout.reading();

    CHECK(shown.rows.empty());
    CHECK(shown.message == "The arm has published nothing yet.");
}

TEST_CASE("a pose readout offering one frame reads that frame's end of a supplied chain", "[manipulator][supplied]")
{
    const praxis::transform flange = fk(displaced_home(), two_axes(), bent());

    stage flange_only(pose_readout::offered_frames::flange);
    REQUIRE(flange_only.shown.supply_joint_screws(displaced_home(), two_axes()).has_value());
    CHECK(reads_at(flange_only.readout.reading(), flange));
    CHECK_FALSE(reads_at(flange_only.readout.reading(), flange * turned_tool_offset()));

    stage tool_only(pose_readout::offered_frames::tool);
    REQUIRE(tool_only.shown.supply_joint_screws(displaced_home(), two_axes()).has_value());
    CHECK(reads_at(tool_only.readout.reading(), flange * turned_tool_offset()));
}
