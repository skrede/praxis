#include "robot/column_arrow.h"

#include <catch2/catch_test_macros.hpp>

#include <threepp/materials/interfaces.hpp>
#include <threepp/materials/MeshBasicMaterial.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cmath>
#include <memory>

using namespace praxis::manipulator;

namespace {

// The placement is written into the renderer's single-precision nodes, so a length read back off one
// agrees to about a part in ten million and no closer.
constexpr double read_back = 1.0e-6;

drawn_arrow an_arrow()
{
    return arrow_object("column", threepp::MeshBasicMaterial::create());
}

// The head's own length is the file's to choose, so it is measured off a long arrow rather than
// spelled here: the shaft carries whatever the whole length is not spent on the head.
double head_length(const drawn_arrow &drawn)
{
    place_arrow(drawn, Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitZ(), 1.0);

    return 1.0 - static_cast<double>(drawn.shaft->scale.y);
}

double apart(const Eigen::Vector3d &one, const Eigen::Vector3d &other)
{
    return (one - other).cwiseAbs().maxCoeff();
}

Eigen::Quaterniond turn_of(const threepp::Object3D &drawn)
{
    return Eigen::Quaterniond(drawn.quaternion.w, drawn.quaternion.x, drawn.quaternion.y, drawn.quaternion.z);
}

}

TEST_CASE("An arrow is built as a shaft and a tip the placement holds directly", "[manipulator][drawing]")
{
    const drawn_arrow drawn = an_arrow();

    REQUIRE(drawn.object != nullptr);
    REQUIRE(drawn.shaft != nullptr);
    REQUIRE(drawn.tip != nullptr);

    CHECK(drawn.object->name == "column");
    CHECK(drawn.shaft->name == "shaft");
    CHECK(drawn.tip->name == "tip");
    CHECK(drawn.object->children.size() == 2u);
}

TEST_CASE("An arrow spends its length on the shaft and stands the tip at the far end", "[manipulator][drawing]")
{
    const drawn_arrow drawn = an_arrow();
    const double head       = head_length(drawn);
    REQUIRE(head > 0.0);

    const Eigen::Vector3d from{0.25, -0.5, 0.125};
    place_arrow(drawn, from, Eigen::Vector3d(0.0, 0.0, 2.0), 0.5);

    CHECK(drawn.object->visible);
    CHECK(drawn.shaft->visible);
    CHECK(std::abs(static_cast<double>(drawn.shaft->scale.y) - (0.5 - head)) < read_back);
    CHECK(std::abs(static_cast<double>(drawn.shaft->position.y) - (0.5 - head) / 2.0) < read_back);
    CHECK(std::abs(static_cast<double>(drawn.tip->position.y) - (0.5 - head / 2.0)) < read_back);

    // The parts are built along the renderer's +Y, so the group is what turns them onto +Z.
    CHECK(apart(turn_of(*drawn.object) * Eigen::Vector3d::UnitY(), Eigen::Vector3d::UnitZ()) < read_back);
    CHECK(apart(Eigen::Vector3d(drawn.object->position.x, drawn.object->position.y, drawn.object->position.z), from) < read_back);
}

TEST_CASE("A longer arrow carries the same head", "[manipulator][drawing]")
{
    const drawn_arrow drawn = an_arrow();
    const double head       = head_length(drawn);

    place_arrow(drawn, Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitZ(), 0.5);
    const double at_a_half = static_cast<double>(drawn.tip->position.y) - static_cast<double>(drawn.shaft->scale.y);
    const float scaled     = drawn.tip->scale.y;

    place_arrow(drawn, Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitZ(), 1.5);
    const double at_one_and_a_half = static_cast<double>(drawn.tip->position.y) - static_cast<double>(drawn.shaft->scale.y);

    CHECK(std::abs(at_a_half - at_one_and_a_half) < read_back);
    CHECK(std::abs(at_a_half - head / 2.0) < read_back);
    CHECK(scaled == 1.f);
    CHECK(drawn.tip->scale.y == 1.f);
}

TEST_CASE("An arrow no longer than its head is the head alone", "[manipulator][drawing]")
{
    const drawn_arrow drawn = an_arrow();
    const double head       = head_length(drawn);

    place_arrow(drawn, Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitZ(), head / 2.0);

    CHECK(drawn.object->visible);
    CHECK_FALSE(drawn.shaft->visible);
    CHECK(std::abs(static_cast<double>(drawn.tip->scale.x) - 0.5) < read_back);
    CHECK(std::abs(static_cast<double>(drawn.tip->scale.y) - 0.5) < read_back);
    CHECK(std::abs(static_cast<double>(drawn.tip->scale.z) - 0.5) < read_back);
    CHECK(std::abs(static_cast<double>(drawn.tip->position.y) - head / 4.0) < read_back);
}

TEST_CASE("An arrow of no length is not drawn", "[manipulator][drawing]")
{
    const drawn_arrow drawn = an_arrow();

    place_arrow(drawn, Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitZ(), 0.0);

    CHECK_FALSE(drawn.object->visible);
}

// Anything else showing a part's tone reads the same function the material is built from, so a key
// naming a tone and the arrow wearing it cannot drift apart.
TEST_CASE("A part's material carries the tone that part answers, and the two parts answer different tones", "[manipulator][drawing]")
{
    for(const jacobian_block part : {jacobian_block::angular, jacobian_block::linear})
    {
        const std::shared_ptr<threepp::Material> worn = column_material(part);
        REQUIRE(worn != nullptr);

        threepp::MaterialWithColor *const toned = worn->as<threepp::MaterialWithColor>();
        REQUIRE(toned != nullptr);
        CHECK(toned->color == column_tone(part));
    }

    CHECK(column_tone(jacobian_block::angular) != column_tone(jacobian_block::linear));
}

TEST_CASE("An arrow along no axis is not drawn and composes no turn", "[manipulator][drawing]")
{
    const drawn_arrow drawn = an_arrow();

    place_arrow(drawn, Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitZ(), 1.0);
    const Eigen::Quaterniond standing = turn_of(*drawn.object);

    place_arrow(drawn, Eigen::Vector3d(1.0, 1.0, 1.0), Eigen::Vector3d::Zero(), 1.0);

    CHECK_FALSE(drawn.object->visible);
    CHECK(std::abs(std::abs(standing.dot(turn_of(*drawn.object))) - 1.0) < read_back);
}
