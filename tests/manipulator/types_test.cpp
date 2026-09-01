#include "praxis/manipulator/types.h"

#include <catch2/catch_test_macros.hpp>

#include <Eigen/Core>

#include <type_traits>

using namespace praxis;
using namespace praxis::manipulator;

static_assert(jacobian::RowsAtCompileTime == 6 && jacobian::ColsAtCompileTime == Eigen::Dynamic);
static_assert(joint_vector::RowsAtCompileTime == Eigen::Dynamic && joint_vector::ColsAtCompileTime == 1);

static_assert(std::is_same_v<joint_vector::Scalar, double>);
static_assert(std::is_same_v<jacobian::Scalar, double>);

TEST_CASE("a_jacobian_carries_one_column_per_joint_and_six_rows_whatever_the_joint_count")
{
    for(Eigen::Index joints = 0; joints < 8; ++joints)
    {
        const jacobian j     = jacobian::Zero(6, joints);
        const joint_vector q = joint_vector::Zero(joints);

        REQUIRE(j.rows() == 6);
        REQUIRE(j.cols() == joints);
        REQUIRE(j.cols() == q.size());
    }
}
