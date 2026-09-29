#include "panel_keys.h"
#include "imgui_frame.h"

#include "praxis/manipulator/arm_snapshot.h"
#include "praxis/manipulator/pose_readout.h"

#include "praxis/rigid_motion/angles.h"
#include "praxis/rigid_motion/capabilities.h"

#include "praxis/evaluation/tolerance.h"

#include <catch2/catch_test_macros.hpp>

#include <imgui.h>
#include <imgui_internal.h>

#include <Eigen/Core>

#include <span>
#include <memory>
#include <string>
#include <vector>
#include <cstddef>
#include <algorithm>
#include <functional>

using namespace praxis;
using namespace praxis::tests;
using namespace praxis::manipulator;

namespace {

const rigid_motion::frame_ops reference = rigid_motion::baseline().frame;

const char *const position_labels[3]{"X", "Y", "Z"};
const char *const angle_labels[3]{"A", "B", "C"};

struct extraction
{
    Eigen::Vector3d angles;
    axis_order order;
};

std::vector<extraction> extracted;

Eigen::Vector3d recorded_extraction(const rotation &r, axis_order order)
{
    const Eigen::Vector3d angles = reference.euler_from_rotation_matrix(r, order);
    extracted.push_back(extraction{angles, order});

    return angles;
}

// The reference bindings with the extraction the readout displays intercepted, which is the only
// route to what the readout computed: it renders the angles and keeps none of them.
rigid_motion::frame_ops recording()
{
    rigid_motion::frame_ops ops    = reference;
    ops.euler_from_rotation_matrix = &recorded_extraction;

    return ops;
}

rotation posed()
{
    return reference.rotate_z(to_radians(37.0)) * reference.rotate_y(to_radians(52.0)) * reference.rotate_x(to_radians(-19.0));
}

arm_snapshot showing(const expected<Eigen::Vector3d, refusal> &position, const expected<rotation, refusal> &orientation)
{
    return arm_snapshot{joint_vector(),
                        joint_limits{},
                        transform::Identity(),
                        transform::Identity(),
                        transform::Identity(),
                        position,
                        position,
                        orientation,
                        orientation,
                        recording_parameters{},
                        1.0,
                        false,
                        scheduler::task_counters{},
                        {},
                        praxis::unexpected(refusal::not_implemented),
                        praxis::unexpected(refusal::not_implemented),
                        jacobian_manipulability{praxis::unexpected(refusal::not_implemented), praxis::unexpected(refusal::not_implemented)},
                        jacobian_manipulability{praxis::unexpected(refusal::not_implemented), praxis::unexpected(refusal::not_implemented)},
                        {},
                        {},
                        nullptr,
                        nullptr,
                        {},
                        {},
                        false};
}

arm_snapshot valued(const rotation &orientation)
{
    return showing(Eigen::Vector3d::Constant(0.25), orientation);
}

std::shared_ptr<arm_publisher> publishing(const arm_snapshot &seen)
{
    auto published = std::make_shared<arm_publisher>();
    published->publish(std::make_shared<const arm_snapshot>(seen));

    return published;
}

robot_slot_set left_at_defaults(std::initializer_list<robot_slot> slots)
{
    robot_slot_set held;
    for(robot_slot slot : slots)
        held.set(slot);

    return held;
}

std::string worded(const arm_publisher &published, robot_slot_set inert)
{
    const pose_readout readout(published.reader(), recording(), inert);
    const scene::readout shown = readout.reading();

    CHECK(shown.rows.empty());

    return shown.message;
}

bool same_rotation(const rotation &first, const rotation &second)
{
    return is_approx_equal(reference.transformation_matrix_from_rotation(first), reference.transformation_matrix_from_rotation(second));
}

bool differs(const Eigen::Vector3d &first, const Eigen::Vector3d &second)
{
    return !is_approx_equal(std::span<const double>(first.data(), 3), std::span<const double>(second.data(), 3));
}

using drawing = std::function<void()>;

// The interface library identifies a panel by its title, so what a frame opened is the set of titles
// the context holds once it has been drawn; the implicit debug panel is the library's own.
std::vector<std::string> panels_of(imgui_frame &frames, const drawing &draw)
{
    frames.draw(draw);

    std::vector<std::string> named;
    for(const ImGuiWindow *held : ImGui::GetCurrentContext()->Windows)
        if(held->WasActive && (held->Flags & ImGuiWindowFlags_ChildWindow) == 0 && held->Name != std::string("Debug##Default"))
            named.push_back(held->Name);

    return named;
}

void tap(imgui_frame &frames, const drawing &draw, ImGuiKey key)
{
    ImGui::GetIO().AddKeyEvent(key, true);
    frames.draw_frame(draw);
    ImGui::GetIO().AddKeyEvent(key, false);
    frames.draw_frame(draw);
}

// The readout carries two selectors and the axis order is the second, so its entry is reached with
// the keyboard alone: two steps to the selector, one to open it, one per entry below the first, and
// one to take the entry that is standing.
void select_entry(imgui_frame &frames, const drawing &draw, std::size_t entry)
{
    ImGui::GetIO().ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    frames.draw(
            [&draw]
            {
                ImGui::SetNextWindowFocus();
                draw();
            });

    tap(frames, draw, ImGuiKey_DownArrow);
    tap(frames, draw, ImGuiKey_DownArrow);
    tap(frames, draw, ImGuiKey_Space);

    for(std::size_t step = 0; step < entry; ++step)
        tap(frames, draw, ImGuiKey_DownArrow);

    tap(frames, draw, ImGuiKey_Space);
}

// Every entry is driven from a context of its own, so what the selector answers is reached from one
// state rather than from wherever the entry before it left the readout standing.
std::vector<extraction> driven_through_every_entry(const arm_reader &seen)
{
    std::vector<extraction> selected;
    for(std::size_t entry = 0; entry < axis_order_labels().size(); ++entry)
    {
        imgui_frame frames;
        const std::shared_ptr<scene::labeled_value_window> readout = compose_pose_readout("Pose display", seen, recording(), robot_slot_set());
        const drawing draw                                         = [&readout] { readout->render(); };

        select_entry(frames, draw, entry);
        extracted.clear();
        frames.draw(draw);

        REQUIRE(frames.vertices() > 0);
        REQUIRE_FALSE(extracted.empty());
        selected.push_back(extracted.back());
    }

    return selected;
}

// One frame refused and the other valued at its own position, so rows read off the wrong frame are
// told apart from rows read off the right one.
arm_snapshot refusing_the(bool tool)
{
    arm_snapshot seen                                  = valued(posed());
    seen.flange_position                               = Eigen::Vector3d::Constant(0.5);
    (tool ? seen.tool_position : seen.flange_position) = praxis::unexpected(refusal::not_implemented);

    return seen;
}

float first_position(const scene::readout &shown)
{
    REQUIRE(shown.message.empty());
    REQUIRE(shown.rows.size() == 3u);

    return shown.rows[0][0].value;
}

std::size_t offered_controls(const arm_reader &seen, pose_readout::offered_frames offered)
{
    imgui_frame frames;
    const std::shared_ptr<scene::labeled_value_window> readout = compose_pose_readout("Pose display", seen, recording(), robot_slot_set(), offered);

    return fixture::navigable_items(frames, [&readout] { readout->render(); });
}

}

TEST_CASE("a readout whose arm has published nothing draws its own account of that", "[manipulator]")
{
    imgui_frame frames;
    arm_publisher published;
    const std::shared_ptr<scene::labeled_value_window> readout = compose_pose_readout("Pose display", published.reader(), recording(), robot_slot_set());

    extracted.clear();
    frames.draw([&readout] { readout->render(); });

    CHECK(frames.has_draw_data());
    CHECK(frames.command_lists() >= 1);
    CHECK(frames.vertices() > 0);
    CHECK(extracted.empty());
}

TEST_CASE("a readout answers each state that is not a value as its own sentence and no values at all", "[manipulator]")
{
    arm_publisher nothing;
    const std::shared_ptr<arm_publisher> refusing = publishing(showing(praxis::unexpected(refusal::not_implemented), praxis::unexpected(refusal::not_implemented)));
    const std::shared_ptr<arm_publisher> valuing  = publishing(valued(posed()));

    CHECK(worded(nothing, robot_slot_set()) == "The arm has published nothing yet.");
    CHECK(worded(*refusing, robot_slot_set()) == "The pose was refused.");
    CHECK(worded(*valuing, left_at_defaults({robot_slot::position_from_pose})) == "The position is not bound.");
    CHECK(worded(*valuing, left_at_defaults({robot_slot::orientation_from_pose})) == "The orientation is not bound.");
    CHECK(worded(*valuing, left_at_defaults({robot_slot::position_from_pose, robot_slot::orientation_from_pose})) == "The position and the orientation are not bound.");
}

TEST_CASE("a readout over a published pose answers three rows of two, the angles in degrees beside the position", "[manipulator]")
{
    const std::shared_ptr<arm_publisher> published = publishing(valued(posed()));
    const pose_readout readout(published->reader(), recording(), robot_slot_set());

    extracted.clear();
    const scene::readout shown = readout.reading();

    REQUIRE(shown.message.empty());
    REQUIRE(shown.rows.size() == 3u);
    REQUIRE(extracted.size() == 1u);
    CHECK(extracted.back().order == axis_order::zyx);
    for(std::size_t axis = 0; axis < 3u; ++axis)
    {
        REQUIRE(shown.rows[axis].size() == 2u);
        CHECK(shown.rows[axis][0].label == std::string(position_labels[axis]));
        CHECK(shown.rows[axis][1].label == std::string(angle_labels[axis]));
        CHECK(shown.rows[axis][0].value == 0.25f);
        CHECK(shown.rows[axis][1].value == static_cast<float>(to_degrees(extracted.back().angles[static_cast<Eigen::Index>(axis)])));
    }
}

TEST_CASE("a composed readout opens the panel it was named for and keeps its adapter for as long as it lives", "[manipulator]")
{
    const std::shared_ptr<arm_publisher> published = publishing(valued(posed()));

    std::shared_ptr<scene::imgui_window> panel;
    {
        const rigid_motion::frame_ops injected = recording();
        panel                                  = compose_pose_readout("Pose display", published->reader(), injected, robot_slot_set());
    }

    REQUIRE(panel != nullptr);
    CHECK(panel->display_name() == "Pose display");

    imgui_frame frames;
    extracted.clear();

    CHECK(panels_of(frames, [&panel] { panel->render(); }) == std::vector<std::string>{"Pose display"});
    CHECK(frames.vertices() > 0);
    CHECK(extracted.size() == 2u);
}

TEST_CASE("the axis order a readout's selector is driven to is the one its angles are extracted under", "[manipulator]")
{
    const rotation orientation                     = posed();
    const std::shared_ptr<arm_publisher> published = publishing(valued(orientation));

    const std::vector<extraction> selected = driven_through_every_entry(published->reader());

    REQUIRE(selected.size() == axis_order_labels().size());
    for(std::size_t entry = 0; entry < selected.size(); ++entry)
    {
        CHECK(selected[entry].order == static_cast<axis_order>(entry));
        CHECK(same_rotation(reference.rotation_matrix_from_euler(selected[entry].angles, selected[entry].order), orientation));
    }

    CHECK(std::any_of(selected.begin() + 1, selected.end(), [&selected](const extraction &e) { return differs(e.angles, selected.front().angles); }));
}

TEST_CASE("a readout offering only the flange reads it where the tool is refused", "[manipulator]")
{
    const std::shared_ptr<arm_publisher> published = publishing(refusing_the(true));
    const pose_readout flange_only(published->reader(), recording(), robot_slot_set(), pose_readout::offered_frames::flange);
    const pose_readout both(published->reader(), recording(), robot_slot_set());

    CHECK(first_position(flange_only.reading()) == 0.5f);
    CHECK(both.reading().message == "The pose was refused.");
}

TEST_CASE("a readout offering only the tool reads it where the flange is refused", "[manipulator]")
{
    const std::shared_ptr<arm_publisher> published = publishing(refusing_the(false));
    const pose_readout tool_only(published->reader(), recording(), robot_slot_set(), pose_readout::offered_frames::tool);

    CHECK(first_position(tool_only.reading()) == 0.25f);
}

TEST_CASE("a readout offering one frame draws no frame control", "[manipulator]")
{
    const std::shared_ptr<arm_publisher> published = publishing(valued(posed()));
    const std::size_t both                         = offered_controls(published->reader(), pose_readout::offered_frames::both);

    CHECK(offered_controls(published->reader(), pose_readout::offered_frames::flange) + 1u == both);
    CHECK(offered_controls(published->reader(), pose_readout::offered_frames::tool) + 1u == both);
}

TEST_CASE("a readout offering one frame over an unbound position answers what one offering both does", "[manipulator]")
{
    const std::shared_ptr<arm_publisher> published = publishing(valued(posed()));
    const robot_slot_set inert                     = left_at_defaults({robot_slot::position_from_pose});
    const pose_readout both(published->reader(), recording(), inert);

    for(const pose_readout::offered_frames offered : {pose_readout::offered_frames::flange, pose_readout::offered_frames::tool})
    {
        const pose_readout one(published->reader(), recording(), inert, offered);
        CHECK(one.reading().message == both.reading().message);
        CHECK(one.reading().message == "The position is not bound.");
    }
}
