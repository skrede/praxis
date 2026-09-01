#include "panel_keys.h"
#include "imgui_frame.h"
#include "panel_labels.h"

#include "praxis/rigid_motion/frame.h"
#include "praxis/rigid_motion/angles.h"
#include "praxis/rigid_motion/frame_stencil.h"
#include "praxis/rigid_motion/matrix_readout.h"

#include "praxis/scene/labeled_value_window.h"

#include <catch2/catch_test_macros.hpp>

#include <threepp/scenes/Scene.hpp>

#include <Eigen/Core>

#include <string>
#include <vector>
#include <cstddef>

using namespace praxis;
using namespace praxis::rigid_motion;

namespace {

using rows = std::vector<std::vector<scene::labeled_value>>;

transform placed(double x, double turn_degrees)
{
    transform tf         = transform::Identity();
    tf.block<3, 3>(0, 0) = inert::rotate_z(to_radians(turn_degrees));
    tf(0, 3)             = x;

    return tf;
}

std::vector<stencil_object> objects(std::size_t count)
{
    std::vector<stencil_object> made(count);
    for(std::size_t index = 0; index < count; ++index)
        made[index].name = "Frame" + std::to_string(index);

    return made;
}

// Every cell of a matrix reading is unlabeled, which is what the generic window draws as a grid, so
// the flattening asserts that on its way past.
std::vector<float> flattened(const rows &shown)
{
    std::vector<float> values;
    for(const std::vector<scene::labeled_value> &row : shown)
        for(const scene::labeled_value &cell : row)
        {
            REQUIRE(cell.label.empty());
            values.push_back(cell.value);
        }

    return values;
}

std::vector<float> block_of(const transform &tf, Eigen::Index extent)
{
    std::vector<float> values;
    for(Eigen::Index row = 0; row < extent; ++row)
        for(Eigen::Index column = 0; column < extent; ++column)
            values.push_back(static_cast<float>(tf(row, column)));

    return values;
}

void require_shape(const rows &shown, std::size_t extent)
{
    REQUIRE(shown.size() == extent);
    for(const std::vector<scene::labeled_value> &row : shown)
        REQUIRE(row.size() == extent);
}

std::size_t geometry_of(matrix_readout &readout)
{
    scene::labeled_value_window window("Matrix", [&readout] { readout.render_controls(); }, [&readout] { return readout.reading(); });

    tests::imgui_frame frames;
    frames.draw([&window] { window.render(); });

    REQUIRE(frames.has_draw_data());
    REQUIRE(frames.vertices() > 0);

    return frames.signature();
}

}

TEST_CASE("each form reads as unlabeled rows of its own extent")
{
    threepp::Scene target;
    frame_stencil body(target, objects(1), frame_ops{});
    body.set_pose(0, placed(1.5, 37.0));

    const matrix_readout turned(body, matrix_form::rotation, [] { return std::size_t{0}; });
    const matrix_readout whole(body, matrix_form::transformation, [] { return std::size_t{0}; });
    const scene::readout three = turned.reading();
    const scene::readout four  = whole.reading();

    REQUIRE(three.message.empty());
    REQUIRE(four.message.empty());
    require_shape(three.rows, 3);
    require_shape(four.rows, 4);
    REQUIRE(flattened(three.rows) == block_of(body.pose(0), 3));
    REQUIRE(flattened(four.rows) == block_of(body.pose(0), 4));
    REQUIRE(flattened(rows{four.rows[3]}) == std::vector<float>{0.f, 0.f, 0.f, 1.f});
}

TEST_CASE("the two frame readings differ for an object with a parent")
{
    threepp::Scene target;
    frame_stencil body(target, objects(2), frame_ops{});
    body.set_pose(0, placed(2.0, 90.0));
    body.set_pose(1, placed(1.0, 15.0));
    REQUIRE(body.set_parent(1, 0).has_value());

    matrix_readout child(body, matrix_form::transformation, [] { return std::size_t{1}; });
    REQUIRE(child.frame() == matrix_frame::parent_relative);
    const std::vector<float> relative = flattened(child.reading().rows);

    child.set_frame(matrix_frame::world);

    REQUIRE(relative == block_of(body.pose(1), 4));
    REQUIRE(flattened(child.reading().rows) == block_of(body.world_pose(1), 4));
    REQUIRE(flattened(child.reading().rows) != relative);
}

TEST_CASE("the two frame readings agree for an object with no parent")
{
    threepp::Scene target;
    frame_stencil body(target, objects(2), frame_ops{});
    body.set_pose(0, placed(2.0, 90.0));
    body.set_pose(1, placed(1.0, 15.0));
    REQUIRE(body.set_parent(1, 0).has_value());

    matrix_readout root(body, matrix_form::transformation, [] { return std::size_t{0}; });
    const std::vector<float> standing = flattened(root.reading().rows);

    root.set_frame(matrix_frame::world);

    REQUIRE(flattened(root.reading().rows) == standing);
}

TEST_CASE("the readout follows the route it was given between readings")
{
    threepp::Scene target;
    frame_stencil body(target, objects(2), frame_ops{});
    body.set_pose(0, placed(2.0, 90.0));
    body.set_pose(1, placed(-3.0, 15.0));

    std::size_t selected = 0;
    const matrix_readout readout(body, matrix_form::transformation, [&selected] { return selected; });
    REQUIRE(flattened(readout.reading().rows) == block_of(body.pose(0), 4));

    selected = 1;
    REQUIRE(flattened(readout.reading().rows) == block_of(body.pose(1), 4));
}

TEST_CASE("an index past the end reads as the identity")
{
    threepp::Scene target;
    frame_stencil body(target, objects(1), frame_ops{});
    body.set_pose(0, placed(2.0, 90.0));

    const matrix_readout turned(body, matrix_form::rotation, [] { return std::size_t{7}; });
    const matrix_readout whole(body, matrix_form::transformation, [] { return std::size_t{7}; });

    REQUIRE(flattened(turned.reading().rows) == block_of(transform::Identity(), 3));
    REQUIRE(flattened(whole.reading().rows) == block_of(transform::Identity(), 4));
}

TEST_CASE("the frame choice names one reading per enumerator under a control naming what it chooses")
{
    REQUIRE(matrix_frame_labels().size() == 2);
    REQUIRE(static_cast<std::size_t>(matrix_frame::parent_relative) == std::size_t{0});
    REQUIRE(static_cast<std::size_t>(matrix_frame::world) == std::size_t{1});
    REQUIRE(std::string(matrix_frame_labels()[0]) == "Parent frame");
    REQUIRE(std::string(matrix_frame_labels()[1]) == "World frame");

    threepp::Scene target;
    frame_stencil body(target, objects(1), frame_ops{});
    matrix_readout readout(body, matrix_form::transformation, [] { return std::size_t{0}; });
    scene::labeled_value_window window("Matrix", [&readout] { readout.render_controls(); }, [&readout] { return readout.reading(); });

    tests::imgui_frame frames;
    fixture::stand_on(frames, [&window] { window.render(); }, "Matrix", "POV");
}

TEST_CASE("the drawn grid follows the form and the frame choice")
{
    threepp::Scene target;
    frame_stencil body(target, objects(2), frame_ops{});
    body.set_pose(0, placed(2.0, 90.0));
    body.set_pose(1, placed(1.0, 15.0));
    REQUIRE(body.set_parent(1, 0).has_value());

    matrix_readout turned(body, matrix_form::rotation, [] { return std::size_t{1}; });
    matrix_readout whole(body, matrix_form::transformation, [] { return std::size_t{1}; });
    const std::size_t relative = geometry_of(whole);

    REQUIRE(geometry_of(turned) != relative);

    whole.set_frame(matrix_frame::world);
    REQUIRE(geometry_of(whole) != relative);
}
