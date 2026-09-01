#include "robot/manipulability.h"

#include "praxis/manipulator/types.h"

#include <catch2/catch_test_macros.hpp>

#include <Eigen/QR>
#include <Eigen/SVD>
#include <Eigen/Core>

#include <cmath>
#include <limits>
#include <cstdint>

using namespace praxis;
using namespace praxis::manipulator;

namespace {

// The blocks are built rather than drawn from a file, so the stream is a stated recurrence and the
// same run of blocks is measured on every machine.
struct rolling
{
    std::uint64_t state = 0x9e3779b97f4a7c15ull;

    double next()
    {
        state = state * 6364136223846793005ull + 1442695040888963407ull;

        return static_cast<double>((state >> 11) & ((1ull << 53) - 1)) / static_cast<double>(1ull << 53) * 2.0 - 1.0;
    }
};

Eigen::MatrixXd orthogonal(rolling &roll, Eigen::Index size)
{
    Eigen::MatrixXd filled(size, size);
    for(Eigen::Index row = 0; row < size; ++row)
        for(Eigen::Index column = 0; column < size; ++column)
            filled(row, column) = roll.next();

    return Eigen::HouseholderQR<Eigen::MatrixXd>(filled).householderQ() * Eigen::MatrixXd::Identity(size, size);
}

// The singular values are prescribed rather than inferred, so what the decomposition should answer
// is known before it is asked.
Eigen::MatrixXd block_of(rolling &roll, const Eigen::Vector3d &values, Eigen::Index width)
{
    const Eigen::MatrixXd left  = orthogonal(roll, 3);
    const Eigen::MatrixXd right = orthogonal(roll, width);

    Eigen::MatrixXd scaled = Eigen::MatrixXd::Zero(3, width);
    scaled.diagonal()      = values;

    return left * scaled * right.transpose();
}

constexpr double exact = 1.0e-12;

}

TEST_CASE("a block built from prescribed singular values answers them in descending order", "[manipulator][manipulability]")
{
    rolling roll;
    const expected<manipulability_ellipsoid, refusal> answered = ellipsoid_of(block_of(roll, Eigen::Vector3d(4.0, 2.0, 1.0), 6));

    REQUIRE(answered.has_value());
    CHECK(std::abs(answered->singular_values(0) - 4.0) < exact);
    CHECK(std::abs(answered->singular_values(1) - 2.0) < exact);
    CHECK(std::abs(answered->singular_values(2) - 1.0) < exact);
    CHECK(std::abs(answered->measure - 8.0) < exact);
    REQUIRE(answered->condition.has_value());
    CHECK(std::abs(*answered->condition - 4.0) < exact);
}

// A left-handed basis composes no rotation, so the columns are corrected before they are published;
// the count is what shows the correction was reached rather than merely written.
TEST_CASE("the principal axes are orthonormal and right-handed even where the raw ones are not", "[manipulator][manipulability]")
{
    rolling roll;
    int left_handed = 0;

    for(int trial = 0; trial < 64; ++trial)
    {
        const Eigen::MatrixXd block                                = block_of(roll, Eigen::Vector3d(4.0, 2.0, 1.0), 6);
        const Eigen::MatrixXd raw                                  = Eigen::JacobiSVD<Eigen::MatrixXd>(block, Eigen::ComputeFullU).matrixU();
        const expected<manipulability_ellipsoid, refusal> answered = ellipsoid_of(block);

        if(raw.determinant() < 0.0)
            ++left_handed;

        REQUIRE(answered.has_value());
        CHECK((answered->principal_axes.transpose() * answered->principal_axes - Eigen::Matrix3d::Identity()).cwiseAbs().maxCoeff() < exact);
        CHECK(std::abs(answered->principal_axes.determinant() - 1.0) < exact);
    }

    CHECK(left_handed > 0);
}

TEST_CASE("a block narrower than three columns names no ellipsoid", "[manipulator][manipulability]")
{
    rolling roll;
    const Eigen::MatrixXd narrow = block_of(roll, Eigen::Vector3d(4.0, 2.0, 1.0), 6).leftCols(2);

    const expected<manipulability_ellipsoid, refusal> answered = ellipsoid_of(narrow);

    REQUIRE_FALSE(answered.has_value());
    CHECK(answered.error() == refusal::unsupported_input);
}

TEST_CASE("a block carrying an entry that is not a finite number names no ellipsoid", "[manipulator][manipulability]")
{
    rolling roll;
    Eigen::MatrixXd block = block_of(roll, Eigen::Vector3d(4.0, 2.0, 1.0), 6);
    block(1, 3)           = std::numeric_limits<double>::infinity();

    const expected<manipulability_ellipsoid, refusal> answered = ellipsoid_of(block);

    REQUIRE_FALSE(answered.has_value());
    CHECK(answered.error() == refusal::degenerate);
}

// The rows are the frame's own axes scaled, so the third singular value is zero as a written number
// rather than as a rounding, and what the published values carry is the zero itself.
TEST_CASE("a block of rank two answers a smallest value of zero, a measure of zero and no condition number", "[manipulator][manipulability]")
{
    Eigen::MatrixXd flat = Eigen::MatrixXd::Zero(3, 6);
    flat(0, 0)           = 4.0;
    flat(1, 1)           = 2.0;

    const expected<manipulability_ellipsoid, refusal> answered = ellipsoid_of(flat);

    REQUIRE(answered.has_value());
    CHECK(answered->singular_values(2) == 0.0);
    CHECK(answered->measure == 0.0);
    CHECK_FALSE(answered->condition.has_value());
    CHECK(answered->singular_values.allFinite());
    CHECK(answered->principal_axes.allFinite());
}

TEST_CASE("a Jacobian that is a refusal carries that refusal into both of its ellipsoids", "[manipulator][manipulability]")
{
    const jacobian_manipulability answered = manipulability_of(unexpected(refusal::no_solution));

    REQUIRE_FALSE(answered.angular.has_value());
    REQUIRE_FALSE(answered.linear.has_value());
    CHECK(answered.angular.error() == refusal::no_solution);
    CHECK(answered.linear.error() == refusal::no_solution);
}

// The two ellipsoids are taken over different rows of the same matrix, so a swapped pair of entries
// in the publication is caught by the values disagreeing with the rows they were meant to come from.
TEST_CASE("the two ellipsoids of one Jacobian are taken over its top and its bottom three rows", "[manipulator][manipulability]")
{
    rolling roll;
    jacobian taken(6, 6);
    taken.topRows(3)    = block_of(roll, Eigen::Vector3d(4.0, 2.0, 1.0), 6);
    taken.bottomRows(3) = block_of(roll, Eigen::Vector3d(9.0, 3.0, 3.0), 6);

    const jacobian_manipulability answered = manipulability_of(taken);

    REQUIRE(answered.angular.has_value());
    REQUIRE(answered.linear.has_value());
    CHECK(std::abs(answered.angular->measure - 8.0) < exact);
    CHECK(std::abs(answered.linear->measure - 81.0) < exact);
    CHECK(std::abs(*answered.linear->condition - 3.0) < exact);
}
