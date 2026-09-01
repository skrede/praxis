#include "imgui_frame.h"

#include "praxis/scene/widgets.h"

#include <catch2/catch_test_macros.hpp>

#include <imgui.h>

#include <Eigen/Core>

#include <utility>
#include <functional>

namespace {

using praxis::tests::imgui_frame;

const char *const axis_labels[3] = {"X", "Y", "Z"};

// A helper places its own items, so the only points a caller can name are the row's origin, read
// before the call, and the trailing item's box, read after it. Both are screen coordinates and are
// valid only inside the frame that produced them.
struct row_geometry
{
    ImVec2 origin;
    ImVec2 last_min;
    ImVec2 last_max;
    float width;
    float height;
};

std::function<void()> drawing(row_geometry &into, std::function<void()> widgets)
{
    return [&into, held = std::move(widgets)]()
    {
        // An item's width is derived from the window's, so a window left to fit its own content
        // never converges on a row that ends in a button: the size is stated instead.
        ImGui::SetNextWindowSize(ImVec2(640.f, 320.f));
        ImGui::Begin("Widgets");
        into.origin = ImGui::GetCursorScreenPos();
        into.width  = ImGui::CalcItemWidth();
        into.height = ImGui::GetFrameHeight();
        held();
        into.last_min = ImGui::GetItemRectMin();
        into.last_max = ImGui::GetItemRectMax();
        ImGui::End();
    };
}

// A triple stacks three rows of one height, so the middle row's origin is the midpoint of the first
// row's and the trailing item's.
ImVec2 row_origin(const row_geometry &row, int index)
{
    return ImVec2(row.origin.x, row.origin.y + (row.last_min.y - row.origin.y) * static_cast<float>(index) * 0.5f);
}

// A stepping numeric input ends with its decrement and increment buttons, each one frame height
// wide, so the increment occupies the last of the item's width.
ImVec2 step_up(const row_geometry &row, ImVec2 group)
{
    return ImVec2(group.x + row.width - row.height * 0.5f, group.y + row.height * 0.5f);
}

ImVec2 along_track(const row_geometry &row, ImVec2 frame)
{
    return ImVec2(frame.x + row.width * 0.25f, frame.y + row.height * 0.5f);
}

ImVec2 reset_button(const row_geometry &row, int index)
{
    return ImVec2((row.last_min.x + row.last_max.x) * 0.5f, row_origin(row, index).y + row.height * 0.5f);
}

// Hovering, pressing and releasing are three separate frames: an item becomes active on the press
// and reports on the release, and neither can be seen within one frame.
void click_at(imgui_frame &frame, const std::function<void()> &contents, ImVec2 spot)
{
    ImGuiIO &io = ImGui::GetIO();
    io.AddMousePosEvent(spot.x, spot.y);
    frame.draw_frame(contents);
    io.AddMouseButtonEvent(0, true);
    frame.draw_frame(contents);
    io.AddMouseButtonEvent(0, false);
    frame.draw_frame(contents);
}

}

TEST_CASE("a numeric triple accepts an edit on every axis with no callable supplied", "[scene]")
{
    imgui_frame frame;
    Eigen::Vector3f vector = Eigen::Vector3f::Zero();
    row_geometry row{};
    const std::function<void()> body = drawing(row, [&]() { praxis::scene::render_float3_inputs(vector, axis_labels, 0.25f, 1.f); });

    frame.draw(body);
    for(int axis = 0; axis < 3; ++axis)
    {
        click_at(frame, body, step_up(row, row_origin(row, axis)));
        REQUIRE(vector[axis] == 0.25f);
    }

    REQUIRE(frame.has_draw_data());
}

TEST_CASE("a numeric input and its reset both run with no callable supplied", "[scene]")
{
    imgui_frame frame;
    float value = 3.f;
    row_geometry row{};
    const std::function<void()> body = drawing(row, [&]() { praxis::scene::render_float_inputs_with_reset(value, "Offset", 0.25f, 1.f); });

    frame.draw(body);
    click_at(frame, body, step_up(row, row.origin));
    REQUIRE(value == 3.25f);

    click_at(frame, body, reset_button(row, 0));

    REQUIRE(value == 0.f);
    REQUIRE(frame.has_draw_data());
}

TEST_CASE("a numeric triple resets every axis with no callable supplied", "[scene]")
{
    imgui_frame frame;
    Eigen::Vector3f vector{1.f, 2.f, 3.f};
    row_geometry row{};
    const std::function<void()> body = drawing(row, [&]() { praxis::scene::render_float3_inputs_with_reset(vector, axis_labels, 0.25f, 1.f); });

    frame.draw(body);
    for(int axis = 0; axis < 3; ++axis)
    {
        click_at(frame, body, reset_button(row, axis));
        REQUIRE(vector[axis] == 0.f);
    }

    REQUIRE(frame.has_draw_data());
}

TEST_CASE("a slider and its reset both run with no callable supplied", "[scene]")
{
    imgui_frame frame;
    float value = 0.5f;
    row_geometry row{};
    const std::function<void()> body = drawing(row, [&]() { praxis::scene::render_float_slider_with_reset(value, "Angle", -1.f, 1.f); });

    frame.draw(body);
    click_at(frame, body, along_track(row, row.origin));
    REQUIRE(value != 0.5f);

    click_at(frame, body, reset_button(row, 0));

    REQUIRE(value == 0.f);
    REQUIRE(frame.has_draw_data());
}

TEST_CASE("a slider triple accepts an edit on every axis with no callable supplied", "[scene]")
{
    imgui_frame frame;
    Eigen::Vector3f vector = Eigen::Vector3f::Zero();
    row_geometry row{};
    const std::function<void()> body = drawing(row, [&]() { praxis::scene::render_float3_slider(vector, axis_labels, -1.f, 1.f); });

    frame.draw(body);
    for(int axis = 0; axis < 3; ++axis)
    {
        click_at(frame, body, along_track(row, row_origin(row, axis)));
        REQUIRE(vector[axis] != 0.f);
    }

    REQUIRE(frame.has_draw_data());
}

TEST_CASE("a slider triple resets every axis with no callable supplied", "[scene]")
{
    imgui_frame frame;
    Eigen::Vector3f vector{0.25f, 0.5f, 0.75f};
    row_geometry row{};
    const std::function<void()> body = drawing(row, [&]() { praxis::scene::render_float3_slider_with_reset(vector, axis_labels, -1.f, 1.f); });

    frame.draw(body);
    for(int axis = 0; axis < 3; ++axis)
    {
        click_at(frame, body, reset_button(row, axis));
        REQUIRE(vector[axis] == 0.f);
    }

    REQUIRE(frame.has_draw_data());
}
