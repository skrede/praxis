#include "window_stage.h"

#include "praxis/manipulator/option_widgets.h"
#include "praxis/manipulator/screw_jog_window.h"
#include "praxis/manipulator/task_space_window.h"

#include "praxis/scene/widgets.h"

#include "praxis/rigid_motion/angles.h"
#include "praxis/rigid_motion/axis_order.h"
#include "praxis/rigid_motion/capabilities.h"

#include <catch2/catch_approx.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_exception.hpp>

#include <Eigen/Core>

#include <imgui.h>

#include <memory>
#include <vector>
#include <cstddef>
#include <stdexcept>
#include <functional>

using namespace praxis;
using namespace praxis::tests;
using namespace praxis::fixture;
using namespace praxis::scheduler;
using namespace praxis::manipulator;
using Catch::Matchers::Message;

namespace {

using mode = control_mode;

const rigid_motion::frame_ops reference     = rigid_motion::baseline().frame;
const rigid_motion::screw_ops reference_arc = rigid_motion::baseline().screw;

// No two of them equal and none a right angle or a straight one, so a pose carried the wrong way
// through the seam lands where none of them is.
const Eigen::Vector3d chosen_euler_degrees{37.0, 52.0, -19.0};
const Eigen::Vector3d chosen_position{0.25, -0.4, 0.7};
const rotation chosen_orientation = reference.rotation_matrix_from_euler(chosen_euler_degrees * radians_per_degree, axis_order::zyx);

// Neither a right angle nor a straight one, so a conversion applied twice and a conversion applied
// in the opposite direction each land somewhere the correct value is not.
constexpr const char *typed_degrees  = "37";
constexpr double screw_angle_degrees = 37.0;

// The control holds the angle in single precision, so the value reaching the seam is the entered
// degrees to within one float step of them.
constexpr double conversion_tolerance = 1.0e-7;
constexpr double float_step           = 1.0e-4;

// One character each, entered at one control only, and no two of them equal or equal to a component
// of the published pose, so a value reaching the seam names the control it was entered at.
constexpr const char *typed_point     = "4";
constexpr const char *typed_direction = "2";
constexpr const char *typed_pitch     = "3";
constexpr double axis_point_x         = 4.0;
constexpr double direction_x          = 2.0;
constexpr double screw_pitch          = 3.0;

// The entry the order selector's list opens on, which is not the order an untouched shared pose
// composes its angles in.
constexpr axis_order first_offered = axis_order::xyz;

struct turn
{
    transform start;
    Eigen::Vector3d direction;
    Eigen::Vector3d at;
    double angle;
    double pitch;
};

std::vector<turn> previewed;
std::vector<double> executed;
// The first thing the executed path hands the direction to. A window that refused a zero direction
// never reaches it, which a recorder downstream of it cannot tell from the seam's own refusal.
std::vector<Eigen::Vector3d> commanded;

expected<joint_vector, refusal> recorded_preview(const rigid_motion::screw_ops &, const kinematics &, const transform &start, const Eigen::Vector3d &w, const Eigen::Vector3d &q,
                                                 double theta_radians, double pitch, const joint_vector &j0)
{
    previewed.push_back(turn{start, w, q, theta_radians, pitch});

    return j0;
}

expected<screw_axis, refusal> recorded_axis(const Eigen::Vector3d &q, const Eigen::Vector3d &s, double h)
{
    commanded.push_back(s);

    return reference_arc.screw_axis_from_point_direction_pitch(q, s, h);
}

transform recorded_exponential(const screw_axis &axis, double theta_radians)
{
    executed.push_back(theta_radians);

    return reference_arc.matrix_exponential_screw(axis, theta_radians);
}

rigid_motion::screw_ops turning()
{
    rigid_motion::screw_ops ops               = reference_arc;
    ops.matrix_exponential_screw              = &recorded_exponential;
    ops.screw_axis_from_point_direction_pitch = &recorded_axis;

    return ops;
}

composed_arm screwing(praxis::scheduler::scheduler &loop)
{
    return compose(loop, motion_ops{.task_space_pose = &position_as_configuration, .task_space_screw = &recorded_preview}, turning());
}

arm_snapshot chosen_snapshot()
{
    return at_rest(configuration(0.0, 0.0), chosen_position, chosen_orientation);
}

arm_snapshot poseless_snapshot()
{
    return at_rest(configuration(0.0, 0.0), praxis::unexpected(refusal::no_solution), praxis::unexpected(refusal::no_solution));
}

std::shared_ptr<edited_pose> holding(const Eigen::Vector3d &position, const Eigen::Vector3d &euler_degrees)
{
    auto held           = std::make_shared<edited_pose>();
    held->position      = position.cast<float>();
    held->euler_degrees = euler_degrees.cast<float>();

    return held;
}

drawing over_both(scene::imgui_window &first, scene::imgui_window &second, const char *focused)
{
    return [&first, &second, focused]
    {
        first.render();
        second.render();
        ImGui::SetWindowFocus(focused);
    };
}

void draw_reference_buttons(mode selected)
{
    static_cast<void>(ImGui::Button("Reset"));
    if(selected == mode::preview)
        return;

    ImGui::SameLine();
    static_cast<void>(ImGui::Button("Move"));
}

// The panel a screw jog window draws, written out here rather than read off the window: which widget
// stands where, what it is labeled and which of them the mode adds are the case's own claim.
void draw_reference(const edited_pose &edited, mode selected)
{
    option_cycle<mode, 2> chosen(selected, {mode::preview, mode::simulation}, {"Preview", "Simulation"});
    edited_pose shown = edited;
    Eigen::Vector3f at{0.f, 0.f, 0.f};
    Eigen::Vector3f along{0.f, 0.f, 1.f};
    float pitch = 0.f;
    float angle = 0.f;

    ImGui::Begin("Screw jog");
    render_option_cycle("Control mode", chosen);
    ImGui::InputFloat3("XYZ", shown.position.data());
    ImGui::InputFloat3("ABC", shown.euler_degrees.data());
    scene::render_enum_selection("Euler order", shown.order, axis_order_labels());
    static_cast<void>(ImGui::Button("Reset start"));
    ImGui::InputFloat3("q", at.data());
    ImGui::InputFloat3("w", along.data());
    ImGui::SameLine();
    static_cast<void>(ImGui::Button("Normalize"));
    ImGui::InputFloat("h", &pitch);
    static_cast<void>(ImGui::SliderFloat("theta", &angle, -180.f, 180.f));
    draw_reference_buttons(selected);
    ImGui::End();
}

// The angle control stands one row above the pane's last, which carries the reset control and, in
// simulation, the move control beside it.
void enter_screw_angle(imgui_frame &frames, const drawing &draw)
{
    reach(frames, draw, ImGuiKey_End);
    tap(frames, draw, ImGuiKey_UpArrow);
    type_at_cursor(frames, draw, typed_degrees);
}

void press_move(imgui_frame &frames, const drawing &draw)
{
    reach(frames, draw, ImGuiKey_End);
    tap(frames, draw, ImGuiKey_RightArrow);
    tap(frames, draw, ImGuiKey_RightArrow);
    tap(frames, draw, ImGuiKey_Space);
}

void press_reset(imgui_frame &frames, const drawing &draw)
{
    reach(frames, draw, ImGuiKey_End);
    tap(frames, draw, ImGuiKey_Space);
}

// The seeding control is the pane's fifth row, under the mode selector, the two pose fields and the
// order selector.
void press_reset_start(imgui_frame &frames, const drawing &draw)
{
    reach(frames, draw, ImGuiKey_Home);
    for(int row = 0; row < 4; ++row)
        tap(frames, draw, ImGuiKey_DownArrow);
    tap(frames, draw, ImGuiKey_Space);
}

// The direction is three rows above the angle control, and a row is entered at its leftmost.
void zero_direction(imgui_frame &frames, const drawing &draw)
{
    reach(frames, draw, ImGuiKey_End);
    for(int row = 0; row < 3; ++row)
        tap(frames, draw, ImGuiKey_UpArrow);
    tap(frames, draw, ImGuiKey_RightArrow);
    tap(frames, draw, ImGuiKey_RightArrow);
    type_at_cursor(frames, draw, "0");
}

// A focus request naming the panel closes any popup standing over it, so the frames that drive the
// order selector's list draw the panel and ask for nothing.
drawing over_alone(scene::imgui_window &panel)
{
    return [&panel] { panel.render(); };
}

void type_rows_above(imgui_frame &frames, const drawing &draw, int rows, const char *text)
{
    reach(frames, draw, ImGuiKey_End);
    for(int row = 0; row < rows; ++row)
        tap(frames, draw, ImGuiKey_UpArrow);
    type_at_cursor(frames, draw, text);
}

// The axis group stands between the seeding control and the angle: the point four rows above the
// pane's last, the direction three and the pitch two. A row of components is entered at its leftmost.
void enter_axis_point(imgui_frame &frames, const drawing &draw)
{
    type_rows_above(frames, draw, 4, typed_point);
}

void enter_axis_direction(imgui_frame &frames, const drawing &draw)
{
    type_rows_above(frames, draw, 3, typed_direction);
}

void enter_screw_pitch(imgui_frame &frames, const drawing &draw)
{
    type_rows_above(frames, draw, 2, typed_pitch);
}

// The normalizing control stands beside the direction, past its three components.
void press_normalize(imgui_frame &frames, const drawing &draw)
{
    reach(frames, draw, ImGuiKey_End);
    for(int row = 0; row < 3; ++row)
        tap(frames, draw, ImGuiKey_UpArrow);
    for(int step = 0; step < 3; ++step)
        tap(frames, draw, ImGuiKey_RightArrow);
    tap(frames, draw, ImGuiKey_Space);
}

// The order selector stands six rows above the pane's last and offers its entries in a popup of its
// own, which opens on its first entry.
void take_first_euler_order(imgui_frame &frames, const drawing &draw)
{
    reach(frames, draw, ImGuiKey_End);
    for(int row = 0; row < 6; ++row)
        tap(frames, draw, ImGuiKey_UpArrow);
    tap(frames, draw, ImGuiKey_Space);
    tap(frames, draw, ImGuiKey_Space);
}

using sequence = std::function<void(imgui_frame &, const drawing &)>;
using staging  = drawing (*)(scene::imgui_window &);

// A screw panel in preview over a live arm, driven by one sequence, answering every screw the seam
// was handed while that sequence ran.
std::vector<turn> previewed_over(const std::shared_ptr<edited_pose> &held, const sequence &drive, staging stage = &over)
{
    previewed.clear();
    executed.clear();
    commanded.clear();

    praxis::scheduler::scheduler loop(inline_workers, clock_source{&reading});
    const composed_arm placed = screwing(loop);
    placed.publishing->publish(std::make_shared<const arm_snapshot>(chosen_snapshot()));
    screw_jog_window panel("Screw jog", placed.seen, placed.owned, reference, held, {mode::preview});

    imgui_frame frames;
    const drawing draw = stage(panel);
    start_navigating(frames, draw);
    drive(frames, draw);
    static_cast<void>(loop.drain());

    return previewed;
}

// Compared component by component rather than by direction, so a vector expected at the origin is
// read the same way as any other.
void along(const Eigen::Vector3d &recorded, const Eigen::Vector3d &expected)
{
    for(Eigen::Index axis = 0; axis < 3; ++axis)
        CHECK(recorded[axis] == Catch::Approx(expected[axis]).margin(float_step));
}

void starts_at(const transform &recorded, const Eigen::Vector3d &position, const rotation &orientation)
{
    for(Eigen::Index axis = 0; axis < 3; ++axis)
        CHECK(recorded(axis, 3) == Catch::Approx(position[axis]).margin(float_step));
    CHECK(recorded.topLeftCorner<3, 3>().isApprox(orientation, float_step));
}

// A panel whose seeding landed is indistinguishable from one where the angle was never entered, and
// the nav cursor stands on the same control at the end of both sequences, so the two frames differ
// only in what the controls carry.
std::size_t seeded_over(const arm_snapshot &seen, bool entering)
{
    const std::shared_ptr<arm_publisher> published = publishing(seen);
    screw_jog_window panel("Screw jog", published->reader(), std::weak_ptr<owned_arm>(), reference, std::make_shared<edited_pose>(), {mode::preview});

    imgui_frame frames;
    const drawing draw = over(panel);
    start_navigating(frames, draw);
    if(entering)
        enter_screw_angle(frames, draw);
    press_reset_start(frames, draw);

    return frames.signature();
}

}

TEST_CASE("a screw jog window built over an arm that has published nothing, or over no pose, refuses and names both ends", "[manipulator]")
{
    arm_publisher unheld;
    const std::shared_ptr<arm_publisher> published = publishing(chosen_snapshot());

    REQUIRE_THROWS_MATCHES(screw_jog_window("Screw jog", unheld.reader(), std::weak_ptr<owned_arm>(), reference, std::make_shared<edited_pose>()), std::invalid_argument,
                           Message("praxis: the screw jog window was given no published arm state to hold"));
    REQUIRE_THROWS_MATCHES(screw_jog_window("Screw jog", published->reader(), std::weak_ptr<owned_arm>(), reference, nullptr), std::invalid_argument,
                           Message("praxis: the screw jog window was given no shared pose to hold"));
}

TEST_CASE("a screw jog panel left with no publication draws one panel stating that", "[manipulator]")
{
    const std::shared_ptr<arm_publisher> published = publishing(chosen_snapshot());
    screw_jog_window panel("Screw jog", published->reader(), std::weak_ptr<owned_arm>(), reference, std::make_shared<edited_pose>());
    published->publish(nullptr);

    CHECK(geometry_of([&panel] { panel.render(); }) == stating_absence("Screw jog"));
}

TEST_CASE("a screw jog window draws the start pose, the axis, the angle and the buttons the mode carries", "[manipulator][controls]")
{
    const std::shared_ptr<arm_publisher> published = publishing(chosen_snapshot());
    const std::shared_ptr<edited_pose> held        = holding(chosen_position, chosen_euler_degrees);
    screw_jog_window previewing("Screw jog", published->reader(), std::weak_ptr<owned_arm>(), reference, held, {mode::preview});
    screw_jog_window simulating("Screw jog", published->reader(), std::weak_ptr<owned_arm>(), reference, held, {mode::simulation});

    const std::size_t drawn_preview    = geometry_of([&previewing] { previewing.render(); });
    const std::size_t drawn_simulation = geometry_of([&simulating] { simulating.render(); });

    CHECK(drawn_preview == geometry_of([&held] { draw_reference(*held, mode::preview); }));
    CHECK(drawn_simulation == geometry_of([&held] { draw_reference(*held, mode::simulation); }));
    CHECK(drawn_preview != drawn_simulation);
}

TEST_CASE("an angle entered at a screw jog slider is previewed in radians about the shared pose", "[manipulator][controls]")
{
    previewed.clear();
    executed.clear();
    commanded.clear();

    praxis::scheduler::scheduler loop(inline_workers, clock_source{&reading});
    const composed_arm placed               = screwing(loop);
    const std::shared_ptr<edited_pose> held = holding(chosen_position, chosen_euler_degrees);
    placed.publishing->publish(std::make_shared<const arm_snapshot>(chosen_snapshot()));
    screw_jog_window panel("Screw jog", placed.seen, placed.owned, reference, held, {mode::preview});

    imgui_frame frames;
    const drawing draw = over(panel);
    start_navigating(frames, draw);
    enter_screw_angle(frames, draw);
    static_cast<void>(loop.drain());

    REQUIRE(!previewed.empty());
    CHECK(executed.empty());
    starts_at(previewed.back().start, chosen_position, chosen_orientation);
    CHECK(previewed.back().direction.isApprox(Eigen::Vector3d::UnitZ(), float_step));
    CHECK(previewed.back().angle == Catch::Approx(screw_angle_degrees * radians_per_degree).margin(conversion_tolerance));
}

TEST_CASE("an angle entered at a screw jog slider is moved along in radians, and not at all without a direction", "[manipulator][controls]")
{
    previewed.clear();
    executed.clear();
    commanded.clear();

    const bool directed = GENERATE(true, false);
    praxis::scheduler::scheduler loop(inline_workers, clock_source{&reading});
    const composed_arm placed = screwing(loop);
    placed.publishing->publish(std::make_shared<const arm_snapshot>(chosen_snapshot()));
    screw_jog_window panel("Screw jog", placed.seen, placed.owned, reference, holding(chosen_position, chosen_euler_degrees), {mode::simulation});

    imgui_frame frames;
    const drawing draw = over(panel);
    start_navigating(frames, draw);
    enter_screw_angle(frames, draw);
    if(!directed)
        zero_direction(frames, draw);
    press_move(frames, draw);
    static_cast<void>(loop.drain());

    CHECK(previewed.empty());
    if(!directed)
    {
        CHECK(commanded.empty());
        CHECK(executed.empty());
        return;
    }

    REQUIRE(!commanded.empty());
    CHECK(commanded.back().isApprox(Eigen::Vector3d::UnitZ(), float_step));
    REQUIRE(!executed.empty());
    CHECK(executed.back() == Catch::Approx(screw_angle_degrees * radians_per_degree).margin(conversion_tolerance));
}

TEST_CASE("a screw jog window previews nothing while the direction it was left with is zero", "[manipulator][controls]")
{
    previewed.clear();
    executed.clear();
    commanded.clear();

    praxis::scheduler::scheduler loop(inline_workers, clock_source{&reading});
    const composed_arm placed = screwing(loop);
    placed.publishing->publish(std::make_shared<const arm_snapshot>(chosen_snapshot()));
    screw_jog_window panel("Screw jog", placed.seen, placed.owned, reference, holding(chosen_position, chosen_euler_degrees), {mode::preview});

    imgui_frame frames;
    const drawing draw = over(panel);
    start_navigating(frames, draw);
    zero_direction(frames, draw);
    enter_screw_angle(frames, draw);
    static_cast<void>(loop.drain());

    CHECK(previewed.empty());
    CHECK(executed.empty());
}

TEST_CASE("a screw jog window's reset returns the axis to the z axis at the origin and previews from there", "[manipulator][controls]")
{
    previewed.clear();
    executed.clear();
    commanded.clear();

    praxis::scheduler::scheduler loop(inline_workers, clock_source{&reading});
    const composed_arm placed = screwing(loop);
    placed.publishing->publish(std::make_shared<const arm_snapshot>(chosen_snapshot()));
    screw_jog_window panel("Screw jog", placed.seen, placed.owned, reference, holding(chosen_position, chosen_euler_degrees), {mode::preview});

    imgui_frame frames;
    const drawing draw = over(panel);
    start_navigating(frames, draw);
    enter_screw_angle(frames, draw);
    static_cast<void>(loop.drain());
    REQUIRE(!previewed.empty());
    CHECK(previewed.back().angle == Catch::Approx(screw_angle_degrees * radians_per_degree).margin(conversion_tolerance));

    const std::size_t entered = previewed.size();
    press_reset(frames, draw);
    static_cast<void>(loop.drain());

    REQUIRE(previewed.size() > entered);
    CHECK(previewed.back().angle == Catch::Approx(0.0).margin(conversion_tolerance));
    CHECK(previewed.back().pitch == Catch::Approx(0.0).margin(conversion_tolerance));
    CHECK(previewed.back().at.isZero(float_step));
    CHECK(previewed.back().direction.isApprox(Eigen::Vector3d::UnitZ(), float_step));
}

TEST_CASE("a screw jog window's reset issues nothing while its mode is simulation", "[manipulator][controls]")
{
    previewed.clear();
    executed.clear();
    commanded.clear();

    praxis::scheduler::scheduler loop(inline_workers, clock_source{&reading});
    const composed_arm placed = screwing(loop);
    placed.publishing->publish(std::make_shared<const arm_snapshot>(chosen_snapshot()));
    screw_jog_window panel("Screw jog", placed.seen, placed.owned, reference, holding(chosen_position, chosen_euler_degrees), {mode::simulation});

    imgui_frame frames;
    const drawing draw = over(panel);
    start_navigating(frames, draw);
    enter_screw_angle(frames, draw);
    press_reset(frames, draw);
    static_cast<void>(loop.drain());

    CHECK(previewed.empty());
    CHECK(executed.empty());
    CHECK(commanded.empty());
}

TEST_CASE("a screw jog window's seeding control reaches the shared pose, and only over a published tool pose", "[manipulator][controls]")
{
    const bool carried                             = GENERATE(true, false);
    const std::shared_ptr<arm_publisher> published = publishing(carried ? chosen_snapshot() : poseless_snapshot());
    const std::shared_ptr<edited_pose> held        = std::make_shared<edited_pose>();
    screw_jog_window panel("Screw jog", published->reader(), std::weak_ptr<owned_arm>(), reference, held, {mode::preview});

    imgui_frame frames;
    const drawing draw = over(panel);
    start_navigating(frames, draw);
    press_reset_start(frames, draw);

    const Eigen::Vector3d seeded = carried ? chosen_position : Eigen::Vector3d::Zero();
    const Eigen::Vector3d angles = carried ? chosen_euler_degrees : Eigen::Vector3d::Zero();
    CHECK(held->position.cast<double>().isApprox(seeded, float_step));
    CHECK(held->euler_degrees.cast<double>().isApprox(angles, float_step));
}

TEST_CASE("a screw jog window's seeding control clears its own axis, and only over a published tool pose", "[manipulator][controls]")
{
    CHECK(seeded_over(chosen_snapshot(), true) == seeded_over(chosen_snapshot(), false));
    CHECK(seeded_over(poseless_snapshot(), true) != seeded_over(poseless_snapshot(), false));
}

TEST_CASE("a screw jog window reads the pose a task space window sharing it wrote", "[manipulator][controls]")
{
    previewed.clear();
    executed.clear();
    commanded.clear();

    praxis::scheduler::scheduler loop(inline_workers, clock_source{&reading});
    const composed_arm placed               = screwing(loop);
    const std::shared_ptr<edited_pose> held = std::make_shared<edited_pose>();
    placed.publishing->publish(std::make_shared<const arm_snapshot>(chosen_snapshot()));
    task_space_window seeding("Task space", placed.seen, placed.owned, reference, held, {task_space_window::motion_shape::ptp, mode::simulation});
    screw_jog_window turned("Screw jog", placed.seen, placed.owned, reference, held, {mode::preview});

    imgui_frame frames;
    start_navigating(frames, over_both(seeding, turned, "Task space"));
    reach(frames, over_both(seeding, turned, "Task space"), ImGuiKey_End);
    tap(frames, over_both(seeding, turned, "Task space"), ImGuiKey_Space);
    enter_screw_angle(frames, over_both(seeding, turned, "Screw jog"));
    static_cast<void>(loop.drain());

    REQUIRE(!previewed.empty());
    starts_at(previewed.back().start, chosen_position, chosen_orientation);
}

TEST_CASE("a screw jog window in preview turns about the axis its point and its direction were moved to", "[manipulator][controls]")
{
    const bool pointed             = GENERATE(true, false);
    const std::vector<turn> issued = previewed_over(holding(chosen_position, chosen_euler_degrees), pointed ? &enter_axis_point : &enter_axis_direction);

    INFO((pointed ? "the axis point" : "the axis direction"));
    REQUIRE(issued.size() == 1);
    starts_at(issued.back().start, chosen_position, chosen_orientation);
    along(issued.back().at, pointed ? Eigen::Vector3d{axis_point_x, 0.0, 0.0} : Eigen::Vector3d::Zero());
    along(issued.back().direction, pointed ? Eigen::Vector3d::UnitZ() : Eigen::Vector3d{direction_x, 0.0, 1.0}.normalized());
    CHECK(issued.back().angle == Catch::Approx(0.0).margin(conversion_tolerance));
    CHECK(issued.back().pitch == Catch::Approx(0.0).margin(conversion_tolerance));
}

TEST_CASE("a screw jog window in preview carries the pitch its field was moved to", "[manipulator][controls]")
{
    const std::vector<turn> issued = previewed_over(holding(chosen_position, chosen_euler_degrees), &enter_screw_pitch);

    REQUIRE(issued.size() == 1);
    starts_at(issued.back().start, chosen_position, chosen_orientation);
    CHECK(issued.back().pitch == Catch::Approx(screw_pitch).margin(float_step));
    along(issued.back().direction, Eigen::Vector3d::UnitZ());
    CHECK(issued.back().angle == Catch::Approx(0.0).margin(conversion_tolerance));
}

TEST_CASE("a screw jog window in preview normalizes the direction it turns about where the control is pressed", "[manipulator][controls]")
{
    const std::vector<turn> issued = previewed_over(holding(chosen_position, chosen_euler_degrees), &press_normalize);

    REQUIRE(issued.size() == 1);
    starts_at(issued.back().start, chosen_position, chosen_orientation);
    along(issued.back().direction, Eigen::Vector3d::UnitZ());
    CHECK(issued.back().angle == Catch::Approx(0.0).margin(conversion_tolerance));
}

TEST_CASE("a screw jog window in preview turns about the start pose the orientation order it was moved to composes", "[manipulator][controls]")
{
    const std::vector<turn> issued = previewed_over(holding(chosen_position, chosen_euler_degrees), &take_first_euler_order, &over_alone);

    REQUIRE(issued.size() == 1);
    starts_at(issued.back().start, chosen_position, reference.rotation_matrix_from_euler(chosen_euler_degrees * radians_per_degree, first_offered));
    CHECK(!issued.back().start.topLeftCorner<3, 3>().isApprox(chosen_orientation, float_step));
    along(issued.back().direction, Eigen::Vector3d::UnitZ());
    CHECK(issued.back().angle == Catch::Approx(0.0).margin(conversion_tolerance));
}
