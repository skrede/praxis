#include "book_inverse_kinematics.h"

#include "praxis/manipulator/kinematics.h"
#include "praxis/manipulator/capabilities.h"

#include "praxis/rigid_motion/capabilities.h"

#include <catch2/catch_test_macros.hpp>

#include <set>
#include <span>
#include <string>
#include <vector>
#include <numbers>
#include <utility>

using namespace praxis;
using namespace praxis::manipulator;
using namespace praxis::tests::book;

namespace {

constexpr double example_tolerance = 1.0e-7;

screw_axis screw_of(double wx, double wy, double wz, double vx, double vy, double vz)
{
    screw_axis s;
    s << wx, wy, wz, vx, vy, vz;
    return s;
}

joint_vector joints(double a, double b, double c)
{
    joint_vector q(3);
    q << a, b, c;
    return q;
}

screw_chain documented_chain(double bound)
{
    transform m;
    m << -1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 6.0, 0.0, 0.0, -1.0, 2.0, 0.0, 0.0, 0.0, 1.0;
    std::vector<screw_axis> screws{screw_of(0.0, 0.0, 1.0, 4.0, 0.0, 0.0), screw_of(0.0, 0.0, 0.0, 0.0, 1.0, 0.0), screw_of(0.0, 0.0, -1.0, -6.0, 0.0, -0.1)};
    const joint_vector wide = joints(bound, bound, bound);
    return screw_chain(m, std::move(screws), joint_limits{wide, wide, -wide, wide});
}

transform documented_target()
{
    transform t;
    t << 0.0, 1.0, 0.0, -5.0, 1.0, 0.0, 0.0, 4.0, 0.0, 0.0, -1.0, 1.6858, 0.0, 0.0, 0.0, 1.0;
    return t;
}

// The IKinBody and IKinSpace examples of the Modern Robotics companion code, modern_robotics/core.py.
joint_vector documented_answer(const literal_form &form)
{
    return form.frame == twist_frame::body ? joints(1.57073819, 2.999667, 3.14153913) : joints(1.57073783, 2.99966384, 3.1415342);
}

joint_vector documented_seed()
{
    return joints(1.5, 2.5, 3.0);
}

// Answers the reference forward map at the documented seed and refuses everywhere else.
expected<transform, refusal> only_at_the_seed(const rigid_motion::screw_ops &screw, const transform &m, std::span<const screw_axis> screws, const joint_vector &theta)
{
    if(theta != documented_seed())
        return unexpected(refusal::degenerate);
    return manipulator::baseline().fk.forward_kinematics(screw, m, screws, theta);
}

kinematics composed(const literal_form &form, forward_kinematics_ops forward = manipulator::baseline().fk, double bound = 10.0)
{
    return kinematics::compose(documented_chain(bound), forward, manipulator::baseline().dk, inverse_kinematics_ops{form.solve}, rigid_motion::baseline().screw,
                               rigid_motion::baseline().frame)
            .value();
}

bool refused_with_no_solution(const expected<joint_vector, refusal> &q)
{
    return !q.has_value() && q.error() == refusal::no_solution;
}

double deviation(const joint_vector &answer, const joint_vector &documented)
{
    return (answer - documented).cwiseAbs().maxCoeff();
}

}

TEST_CASE("each_literal_inverse_kinematics_form_reaches_the_answer_the_companion_code_documents")
{
    for(const literal_form &form : literal_forms)
    {
        const kinematics solver                 = composed(form);
        const expected<joint_vector, refusal> q = solver.ik_solve(documented_target(), documented_seed(), solver_parameters(0.001, 0.01, 70u));

        INFO(form_label(form));
        REQUIRE(q.has_value());
        CHECK(deviation(*q, documented_answer(form)) <= example_tolerance);
    }
}

TEST_CASE("a_literal_form_that_does_not_converge_refuses_with_no_solution_and_names_no_configuration")
{
    for(const literal_form &form : literal_forms)
    {
        const kinematics solver                 = composed(form);
        const expected<joint_vector, refusal> q = solver.ik_solve(documented_target(), documented_seed(), solver_parameters(0.001, 0.01, 0u));

        INFO(form_label(form));
        if(form.cap == iteration_cap::handed)
        {
            CHECK(refused_with_no_solution(q));
            CHECK(solver.solutions().empty());
        }
        else
        {
            REQUIRE(q.has_value());
            CHECK(deviation(*q, documented_answer(form)) <= example_tolerance);
        }
    }
}

TEST_CASE("every_literal_inverse_kinematics_form_carries_its_own_label")
{
    std::set<std::string> labels;
    for(const literal_form &form : literal_forms)
        labels.insert(form_label(form));

    CHECK(literal_forms.size() == 24u);
    CHECK(labels.size() == literal_forms.size());
}

// The example converges on its second step, inside 1e-5 rad and 1e-3 m there.
TEST_CASE("a_literal_form_takes_no_step_beyond_its_cap_and_reads_each_tolerance_as_its_own_half")
{
    for(const literal_form &form : literal_forms)
    {
        const kinematics solver = composed(form);

        INFO(form_label(form));
        CHECK((solver.ik_solve(documented_target(), documented_answer(form), solver_parameters(0.001, 0.01, 0u)).value() == documented_answer(form)));
        CHECK(deviation(solver.ik_solve(documented_target(), documented_seed(), solver_parameters(0.001, 1.0e-5, 70u)).value(), documented_answer(form)) <= example_tolerance);
        if(form.cap == iteration_cap::handed)
        {
            CHECK(deviation(solver.ik_solve(documented_target(), documented_seed(), solver_parameters(0.001, 0.01, 2u)).value(), documented_answer(form)) <= example_tolerance);
            CHECK(refused_with_no_solution(solver.ik_solve(documented_target(), documented_seed(), solver_parameters(0.001, 0.01, 1u))));
        }
    }
}

TEST_CASE("a_literal_form_whose_handed_map_refuses_mid_solve_refuses_with_no_solution")
{
    forward_kinematics_ops refusing = manipulator::baseline().fk;
    refusing.forward_kinematics     = &only_at_the_seed;
    for(const literal_form &form : literal_forms)
    {
        const kinematics solver                 = composed(form, refusing);
        const expected<joint_vector, refusal> q = solver.ik_solve(documented_target(), documented_seed(), solver_parameters(0.001, 0.01, 70u));

        INFO(form_label(form));
        if(form.source == helpers::handed)
            CHECK((refused_with_no_solution(q) && solver.solutions().empty()));
        else
            CHECK(deviation(q.value(), documented_answer(form)) <= example_tolerance);
    }
}

// The first joint is a pure rotation, so a whole turn on it reaches the same pose.
TEST_CASE("a_literal_form_answers_its_configuration_as_reached_with_no_whole_turn_naming_and_no_bound")
{
    const joint_vector turned = joints(2.0 * std::numbers::pi, 0.0, 0.0);
    for(const literal_form &form : literal_forms)
    {
        const kinematics solver = composed(form, manipulator::baseline().fk, 1.0);

        INFO(form_label(form));
        CHECK(deviation(solver.ik_solve(documented_target(), documented_seed(), solver_parameters(0.001, 0.01, 70u)).value(), documented_answer(form)) <= example_tolerance);
        CHECK(deviation(solver.ik_solve(documented_target(), documented_seed() + turned, solver_parameters(0.001, 0.01, 70u)).value(), documented_answer(form) + turned) <=
              example_tolerance);
    }
}
