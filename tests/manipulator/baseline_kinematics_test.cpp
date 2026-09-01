#include "captured_log.h"

#include "praxis/manipulator/capabilities.h"
#include "praxis/manipulator/baseline/kinematics.h"
#include "praxis/rigid_motion/baseline/screw.h"
#include "praxis/rigid_motion/capabilities.h"

#include "praxis/evaluation/tolerance.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <span>
#include <limits>
#include <string>
#include <vector>
#include <cstddef>
#include <cstdint>
#include <numbers>
#include <utility>
#include <optional>
#include <functional>

using namespace praxis;
using namespace praxis::manipulator;

namespace {

constexpr double upper_arm = 0.5;
constexpr double forearm   = 0.4;

const rigid_motion::screw_ops reference_screw  = rigid_motion::baseline().screw;
const rigid_motion::frame_ops reference_frames = rigid_motion::baseline().frame;

// A planar two-link arm: both joints rotate about the world z, the second sits at the far end of the
// first link, and the tool frame is the far end of the second. Lynch & Park, Modern Robotics,
// example 4.3 -- a screw axis of a revolute joint is (w, -w x q) for a point q on the axis.
screw_chain planar_arm()
{
    screw_axis shoulder;
    shoulder << 0.0, 0.0, 1.0, 0.0, 0.0, 0.0;
    screw_axis elbow;
    elbow << 0.0, 0.0, 1.0, 0.0, -upper_arm, 0.0;

    transform home = transform::Identity();
    home(0, 3)     = upper_arm + forearm;

    joint_limits bounds{};
    bounds.velocity       = joint_vector::Constant(2, 1.0);
    bounds.acceleration   = joint_vector::Constant(2, 2.0);
    bounds.lower_position = joint_vector::Constant(2, -3.0);
    bounds.upper_position = joint_vector::Constant(2, 3.0);

    return screw_chain(home, {shoulder, elbow}, bounds);
}

joint_vector configuration(double first, double second)
{
    joint_vector q(2);
    q << first, second;

    return q;
}

const solver_parameters tight_parameters(1.0e-10, 1.0e-10, 400u);

kinematics reference(const screw_chain &chain)
{
    expected<kinematics, refusal> solver =
            manipulator::make_kinematics(chain, manipulator::baseline().fk, manipulator::baseline().dk, manipulator::baseline().ik, reference_screw, reference_frames);
    REQUIRE(solver);

    return std::move(*solver);
}

transform reached(const kinematics &solver, const joint_vector &q)
{
    const expected<transform, refusal> pose = solver.fk_solve(q);
    REQUIRE(pose);

    return *pose;
}

joint_vector solved(const kinematics &solver, const transform &target, const joint_vector &seed, const solver_parameters &parameters)
{
    const expected<joint_vector, refusal> solution = solver.ik_solve(target, seed, parameters);
    REQUIRE(solution);

    return *solution;
}

}

TEST_CASE("forward_kinematics_places_the_tool_where_the_arm_reaches")
{
    const screw_chain chain = planar_arm();
    const kinematics solver = reference(chain);

    REQUIRE(solver.joint_count() == 2u);
    CHECK(is_approx_equal(reached(solver, joint_vector::Zero(2)), chain.home));

    const transform folded = reached(solver, configuration(0.0, std::numbers::pi));
    CHECK(is_approx_equal(folded(0, 3), upper_arm - forearm));
    CHECK(is_approx_equal(folded(1, 3), 0.0));
}

// The two forms the chapter publishes differ in the frame the screws are written in and in where the
// home pose is composed, and answer the same pose. Lynch & Park, Modern Robotics, chapter 4.
TEST_CASE("the_body_form_reaches_the_pose_the_space_form_reaches_over_the_same_arm")
{
    const screw_chain chain                               = planar_arm();
    const kinematics solver                               = reference(chain);
    const expected<std::vector<screw_axis>, refusal> body = manipulator::body_screws_from_space(reference_screw, reference_frames, chain.home, chain.space_screws);
    REQUIRE(body);

    for(const joint_vector &q : {configuration(0.0, 0.0), configuration(0.4, -0.7), configuration(std::numbers::pi / 2.0, 0.3)})
    {
        const expected<transform, refusal> from_space = manipulator::forward_kinematics(chain.home, chain.space_screws, q);
        const expected<transform, refusal> from_body  = manipulator::body_forward_kinematics(reference_frames, chain.home, *body, q);

        REQUIRE(from_space);
        REQUIRE(from_body);
        CHECK(is_approx_equal(*from_space, *from_body));

        // The holder derives the body chain once at composition and asks the slot over that, so the
        // route through it answers what the slot answers over the screws derived beside it.
        const expected<transform, refusal> through_holder = solver.body_fk_solve(q);
        REQUIRE(through_holder);
        CHECK(is_approx_equal(*through_holder, *from_space));
    }
}

TEST_CASE("inverse_kinematics_recovers_a_configuration_reaching_the_target")
{
    const kinematics solver = reference(planar_arm());
    const transform target  = reached(solver, configuration(0.4, -0.7));

    const joint_vector solution = solved(solver, target, configuration(0.1, -0.1), tight_parameters);
    CHECK(is_approx_equal(reached(solver, solution), target, 1.0e-9));
}

TEST_CASE("the_solution_selector_is_offered_the_solutions_the_solver_found")
{
    const kinematics solver = reference(planar_arm());
    const transform target  = reached(solver, configuration(-0.3, 0.9));

    std::size_t offered                            = 0u;
    const expected<joint_vector, refusal> solution = solver.ik_solve(target, configuration(-0.1, 0.5), tight_parameters,
                                                                     [&offered](std::span<const joint_vector> solutions)
                                                                     {
                                                                         offered = solutions.size();
                                                                         return std::optional<std::size_t>(0u);
                                                                     });

    REQUIRE(solution);
    CHECK(offered == 1u);
    CHECK(is_approx_equal(reached(solver, *solution), target, 1.0e-9));
}

TEST_CASE("an_unreachable_target_is_refused_rather_than_answered_with_the_seed")
{
    const kinematics solver = reference(planar_arm());
    const joint_vector j0   = configuration(0.2, 0.2);

    transform far_away = transform::Identity();
    far_away(0, 3)     = 10.0 * (upper_arm + forearm);

    const expected<joint_vector, refusal> solution = solver.ik_solve(far_away, j0, tight_parameters);
    REQUIRE_FALSE(solution.has_value());
    CHECK(solution.error() == refusal::no_solution);
}

TEST_CASE("a_configuration_the_chain_does_not_have_is_refused_by_every_shape_guarded_answer")
{
    const kinematics solver  = reference(planar_arm());
    const joint_vector wrong = joint_vector::Constant(3, 0.2);

    const expected<transform, refusal> pose       = solver.fk_solve(wrong);
    const expected<jacobian, refusal> space_frame = solver.space_jacobian(wrong);
    const expected<jacobian, refusal> body_frame  = solver.body_jacobian(wrong);

    REQUIRE_FALSE(pose.has_value());
    REQUIRE_FALSE(space_frame.has_value());
    REQUIRE_FALSE(body_frame.has_value());
    CHECK(pose.error() == refusal::unsupported_input);
    CHECK(space_frame.error() == refusal::unsupported_input);
    CHECK(body_frame.error() == refusal::unsupported_input);
}

// The four answers below are guarded before the solver library is reached at all: its chain rejects a
// joint count, an empty axis list and a nonfinite input by throwing, and a slot answers on the refusal
// channel instead.
TEST_CASE("a_joint_vector_the_screw_span_does_not_cover_is_refused_by_every_delegating_answer")
{
    const screw_chain chain  = planar_arm();
    const joint_vector wrong = joint_vector::Constant(3, 0.2);

    const expected<transform, refusal> pose       = manipulator::forward_kinematics(chain.home, chain.space_screws, wrong);
    const expected<transform, refusal> body_pose  = manipulator::body_forward_kinematics(reference_frames, chain.home, chain.space_screws, wrong);
    const expected<jacobian, refusal> space_frame = manipulator::space_jacobian(chain.space_screws, wrong);
    const expected<jacobian, refusal> body_frame  = manipulator::body_jacobian(chain.space_screws, wrong);

    REQUIRE_FALSE(pose.has_value());
    REQUIRE_FALSE(body_pose.has_value());
    REQUIRE_FALSE(space_frame.has_value());
    REQUIRE_FALSE(body_frame.has_value());
    CHECK(pose.error() == refusal::unsupported_input);
    CHECK(body_pose.error() == refusal::unsupported_input);
    CHECK(space_frame.error() == refusal::unsupported_input);
    CHECK(body_frame.error() == refusal::unsupported_input);

    // The shape is answered before any value is read, so a mismatch over a chain that is degenerate as
    // well earns the shape refusal rather than the numeric one.
    transform unbounded = transform::Identity();
    unbounded(0, 3)     = std::numeric_limits<double>::infinity();

    const expected<transform, refusal> shaped      = manipulator::forward_kinematics(unbounded, chain.space_screws, wrong);
    const expected<transform, refusal> shaped_body = manipulator::body_forward_kinematics(reference_frames, unbounded, chain.space_screws, wrong);
    REQUIRE_FALSE(shaped.has_value());
    REQUIRE_FALSE(shaped_body.has_value());
    CHECK(shaped.error() == refusal::unsupported_input);
    CHECK(shaped_body.error() == refusal::unsupported_input);
}

TEST_CASE("a_span_of_no_screws_is_answered_rather_than_refused")
{
    transform home = transform::Identity();
    home(2, 3)     = 0.75;

    const joint_vector none                       = joint_vector::Zero(0);
    const expected<transform, refusal> pose       = manipulator::forward_kinematics(home, {}, none);
    const expected<transform, refusal> body_pose  = manipulator::body_forward_kinematics(reference_frames, home, {}, none);
    const expected<jacobian, refusal> space_frame = manipulator::space_jacobian({}, none);
    const expected<jacobian, refusal> body_frame  = manipulator::body_jacobian({}, none);

    REQUIRE(pose);
    REQUIRE(body_pose);
    REQUIRE(space_frame);
    REQUIRE(body_frame);
    CHECK(is_approx_equal(*pose, home));
    CHECK(is_approx_equal(*body_pose, home));
    CHECK(space_frame->rows() == 6);
    CHECK(space_frame->cols() == 0);
    CHECK(body_frame->rows() == 6);
    CHECK(body_frame->cols() == 0);
}

TEST_CASE("a_chain_carrying_a_nonfinite_value_is_refused_rather_than_thrown_over")
{
    const screw_chain chain = planar_arm();
    const joint_vector q    = configuration(0.2, -0.4);

    transform unbounded = transform::Identity();
    unbounded(0, 3)     = std::numeric_limits<double>::infinity();

    std::vector<screw_axis> blunted = chain.space_screws;
    blunted[1]                      = screw_axis::Constant(std::numeric_limits<double>::quiet_NaN());

    const expected<transform, refusal> pose       = manipulator::forward_kinematics(unbounded, chain.space_screws, q);
    const expected<transform, refusal> body_pose  = manipulator::body_forward_kinematics(reference_frames, chain.home, blunted, q);
    const expected<jacobian, refusal> space_frame = manipulator::space_jacobian(blunted, q);
    const expected<jacobian, refusal> body_frame  = manipulator::body_jacobian(blunted, q);

    REQUIRE_FALSE(pose.has_value());
    REQUIRE_FALSE(body_pose.has_value());
    REQUIRE_FALSE(space_frame.has_value());
    REQUIRE_FALSE(body_frame.has_value());
    CHECK(pose.error() == refusal::degenerate);
    CHECK(body_pose.error() == refusal::degenerate);
    CHECK(space_frame.error() == refusal::degenerate);
    CHECK(body_frame.error() == refusal::degenerate);
}

TEST_CASE("both_jacobians_carry_one_column_per_joint")
{
    const kinematics solver                       = reference(planar_arm());
    const joint_vector q                          = configuration(0.3, -0.6);
    const expected<jacobian, refusal> space_frame = solver.space_jacobian(q);
    const expected<jacobian, refusal> body_frame  = solver.body_jacobian(q);

    REQUIRE(space_frame);
    REQUIRE(body_frame);
    REQUIRE(space_frame->rows() == 6);
    REQUIRE(space_frame->cols() == 2);
    REQUIRE(body_frame->rows() == 6);
    REQUIRE(body_frame->cols() == 2);

    // J_b = [Ad_{T^-1}] J_s, which ties the two together at every configuration rather than only at
    // the one the recorded columns below were taken at.
    const transform pose                          = reached(solver, q);
    const rotation transposed                     = pose.block<3, 3>(0, 0).transpose();
    const expected<adjoint, refusal> inverse_pose = rigid_motion::adjoint_matrix_from_rotation_position(transposed, -(transposed * pose.block<3, 1>(0, 3)));
    REQUIRE(inverse_pose);
    CHECK((*body_frame - *inverse_pose * *space_frame).isZero(default_tolerance));
}

TEST_CASE("the_space_jacobian_columns_are_the_screws_carried_by_the_joints_below_them")
{
    const screw_chain chain                       = planar_arm();
    const kinematics solver                       = reference(chain);
    const expected<jacobian, refusal> space_frame = solver.space_jacobian(configuration(std::numbers::pi / 2.0, 0.0));

    // The first column is the first screw unchanged; the second is the first joint's rotation applied
    // to the second screw, so a quarter turn at the shoulder swings the elbow's axis point onto +y.
    REQUIRE(space_frame);
    CHECK((space_frame->col(0) - chain.space_screws[0]).isZero(default_tolerance));
    screw_axis swung;
    swung << 0.0, 0.0, 1.0, upper_arm, 0.0, 0.0;
    CHECK((space_frame->col(1) - swung).isZero(default_tolerance));
}

TEST_CASE("the_body_chain_is_the_space_chain_seen_from_the_home_pose")
{
    const screw_chain chain = planar_arm();
    const kinematics solver = reference(chain);

    const expected<std::reference_wrapper<const screw_chain>, refusal> derived = solver.body_chain();
    REQUIRE(derived.has_value());
    const screw_chain &body = *derived;

    REQUIRE(body.joint_count() == chain.joint_count());
    CHECK(is_approx_equal(body.home, chain.home));

    // Both axes are the world z through a point on the arm; seen from the tool frame the shoulder is
    // a distance (upper_arm + forearm) behind it and the elbow a distance forearm behind it.
    screw_axis shoulder;
    shoulder << 0.0, 0.0, 1.0, 0.0, upper_arm + forearm, 0.0;
    screw_axis elbow;
    elbow << 0.0, 0.0, 1.0, 0.0, forearm, 0.0;
    CHECK((body.space_screws[0] - shoulder).isZero(default_tolerance));
    CHECK((body.space_screws[1] - elbow).isZero(default_tolerance));
}

TEST_CASE("a_home_pose_that_is_not_a_rigid_motion_is_refused_by_both_answers_that_read_it_as_a_frame")
{
    transform skewed = transform::Identity();
    skewed(0, 1)     = 0.5;

    // The derivation reads the rotation block and the position column alone, so a bottom row that is
    // not affine reaches the adjoint as a proper rotation and is caught by nothing but the home pose
    // being read as a member of SE(3).
    transform unscaled = transform::Identity();
    unscaled.row(3)    = Eigen::Vector4d{0.0, 0.0, 0.0, 2.0}.transpose();

    const expected<std::vector<screw_axis>, refusal> derived = manipulator::body_screws_from_space(reference_screw, reference_frames, skewed, planar_arm().space_screws);
    const expected<std::vector<screw_axis>, refusal> unread  = manipulator::body_screws_from_space(reference_screw, reference_frames, unscaled, planar_arm().space_screws);

    REQUIRE_FALSE(derived.has_value());
    CHECK(derived.error() == refusal::degenerate);
    REQUIRE_FALSE(unread.has_value());
    CHECK(unread.error() == refusal::degenerate);

    // The body form composes the home pose itself rather than handing it to the solver library, so the
    // frame it is given is read here or nowhere.
    const expected<transform, refusal> composed = manipulator::body_forward_kinematics(reference_frames, skewed, planar_arm().space_screws, configuration(0.2, -0.4));
    REQUIRE_FALSE(composed.has_value());
    CHECK(composed.error() == refusal::degenerate);
}

// The inert adjoint construction answers a refusal rather than a matrix, so the derivation carries
// that refusal out rather than fabricating a body chain without one.
TEST_CASE("a_derivation_handed_screw_operations_left_on_their_defaults_answers_on_the_refusal_channel")
{
    const screw_chain chain = planar_arm();

    const expected<std::vector<screw_axis>, refusal> derived = manipulator::body_screws_from_space(rigid_motion::screw_ops{}, reference_frames, chain.home, chain.space_screws);

    REQUIRE_FALSE(derived.has_value());
    CHECK(derived.error() == refusal::not_implemented);
}

// Binding one capability is never the price of another: the composition stands and answers over the
// space chain, and only the answers taken over a body chain carry what the derivation refused with.
TEST_CASE("a_solver_composed_against_defaulted_screw_operations_refuses_only_the_answers_taken_over_a_body_chain")
{
    const screw_chain chain = planar_arm();
    const joint_vector q    = configuration(0.3, -0.6);

    const expected<kinematics, refusal> composed =
            manipulator::make_kinematics(chain, manipulator::baseline().fk, manipulator::baseline().dk, manipulator::baseline().ik, rigid_motion::screw_ops{}, reference_frames);

    REQUIRE(composed.has_value());
    CHECK(composed->fk_solve(q).has_value());
    CHECK(composed->space_jacobian(q).has_value());

    const expected<std::reference_wrapper<const screw_chain>, refusal> body = composed->body_chain();
    const expected<transform, refusal> reached                              = composed->body_fk_solve(q);
    const expected<jacobian, refusal> body_frame                            = composed->body_jacobian(q);

    REQUIRE_FALSE(body.has_value());
    REQUIRE_FALSE(reached.has_value());
    REQUIRE_FALSE(body_frame.has_value());
    CHECK(body.error() == refusal::not_implemented);
    CHECK(reached.error() == refusal::not_implemented);
    CHECK(body_frame.error() == refusal::not_implemented);
}

TEST_CASE("the_iteration_sequence_is_the_one_the_solve_passed_through")
{
    const kinematics solver = reference(planar_arm());
    const transform target  = reached(solver, configuration(0.5, -0.8));
    const joint_vector seed = configuration(0.05, -0.05);

    REQUIRE(solver.iterations().empty());
    const joint_vector solution = solved(solver, target, seed, tight_parameters);

    const std::span<const iteration_state> states = solver.iterations();
    REQUIRE(states.size() > 1u);
    CHECK(states.back().angular_error < states.front().angular_error);
    CHECK(states.back().linear_error < states.front().linear_error);
    CHECK(is_approx_equal(states.back().joint_positions, solution));

    // Each recorded step is the distance from the iterate before it, so a record that carried the
    // wrong configuration would disagree with the distance stored beside it.
    for(std::size_t i = 0; i < states.size(); ++i)
    {
        const joint_vector &before = i == 0u ? seed : states[i - 1u].joint_positions;
        CHECK(states[i].index == static_cast<std::uint32_t>(i));
        CHECK(is_approx_equal(states[i].step_norm, (states[i].joint_positions - before).norm()));
    }
}

// A chain the solver library cannot represent is one every solve through the holder would fail on, so
// the factory refuses it rather than handing back a holder that reads as bound.
TEST_CASE("a_chain_the_solver_library_cannot_represent_is_refused_rather_than_held")
{
    screw_axis degenerate;
    degenerate << 0.0, 0.0, 0.5, 0.0, 0.0, 0.0;

    const capabilities bindings = manipulator::baseline();

    const expected<kinematics, refusal> blunted =
            manipulator::make_kinematics(screw_chain(transform::Identity(), {degenerate}, joint_limits{}), bindings.fk, bindings.dk, bindings.ik, reference_screw, reference_frames);
    const expected<kinematics, refusal> empty = manipulator::make_kinematics(screw_chain(), bindings.fk, bindings.dk, bindings.ik, reference_screw, reference_frames);

    REQUIRE_FALSE(blunted.has_value());
    REQUIRE_FALSE(empty.has_value());
    CHECK(blunted.error() == refusal::degenerate);
    CHECK(empty.error() == refusal::degenerate);
}

// The refusal a chain of uncovered limits earns is fatal, so a caller that receives it bare is told a
// composition was torn down and nothing about which of the four vectors was short.
TEST_CASE("a_chain_whose_limits_do_not_cover_its_joints_is_refused_with_the_mismatch_stated")
{
    screw_chain uncovered     = planar_arm();
    uncovered.limits.velocity = joint_vector::Constant(1, 1.0);

    const capabilities bindings            = manipulator::baseline();
    expected<kinematics, refusal> composed = praxis::unexpected(refusal::not_implemented);
    const std::string reported =
            praxis::tests::reported_by([&] { composed = manipulator::make_kinematics(uncovered, bindings.fk, bindings.dk, bindings.ik, reference_screw, reference_frames); });

    REQUIRE_FALSE(composed.has_value());
    CHECK(composed.error() == refusal::unsupported_input);
    CHECK_THAT(reported, Catch::Matchers::ContainsSubstring("manipulator.make_kinematics"));
    CHECK_THAT(reported, Catch::Matchers::ContainsSubstring("1, 2, 2 and 2"));
}
