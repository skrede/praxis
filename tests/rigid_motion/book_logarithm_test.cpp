#include "fixtures.h"
#include "baseline/book_logarithm.h"

#include "praxis/rigid_motion/types.h"

#include <catch2/catch_test_macros.hpp>

#include <limits>
#include <utility>

using namespace praxis;
using namespace praxis::conformance;

TEST_CASE("the_book_pose_logarithm_refuses_a_translation_that_is_not_finite_in_either_form")
{
    const Eigen::Vector3d far{std::numeric_limits<double>::infinity(), 0.0, 0.0};
    const expected<std::pair<screw_axis, double>, refusal> from_rp = rigid_motion::book::matrix_logarithm_se3_rp(rotation::Identity(), far);
    const expected<std::pair<screw_axis, double>, refusal> from_tf = rigid_motion::book::matrix_logarithm_se3(assembled(rotation::Identity(), far));

    REQUIRE_FALSE(from_rp.has_value());
    REQUIRE_FALSE(from_tf.has_value());

    CHECK(from_rp.error() == refusal::degenerate);
    CHECK(from_tf.error() == refusal::degenerate);
}
