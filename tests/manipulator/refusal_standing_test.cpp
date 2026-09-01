#include "captured_log.h"
#include "six_axis_machine.h"

#include "praxis/manipulator/arm_state.h"
#include "praxis/manipulator/compose_arm.h"
#include "praxis/manipulator/scene_robot.h"
#include "praxis/manipulator/capabilities.h"
#include "praxis/manipulator/robot_controller.h"

#include "praxis/trajectory/capabilities.h"

#include "praxis/rigid_motion/capabilities.h"

#include "praxis/scheduler/scheduler.h"

#include <catch2/catch_test_macros.hpp>

#include <threepp/scenes/Scene.hpp>

#include <span>
#include <cmath>
#include <memory>
#include <string>
#include <vector>
#include <limits>
#include <cstddef>

using namespace praxis;
using namespace praxis::tests;
using namespace praxis::fixture;
using namespace praxis::manipulator;

namespace {

// Six rows and one column per joint, so either block is three by six and an ellipsoid can be taken
// from it. One entry is not a number past a first joint value, so one composition is decomposable at
// some of its configurations and not at others.
expected<jacobian, refusal> banded_space_jacobian(std::span<const screw_axis>, const joint_vector &theta)
{
    constexpr double deranged_beyond = 0.5;
    jacobian columns(6, theta.size());
    for(Eigen::Index joint = 0; joint < theta.size(); ++joint)
        for(Eigen::Index row = 0; row < 6; ++row)
            columns(row, joint) = std::cos(0.25 * static_cast<double>(row) + static_cast<double>(joint) + theta[joint]);

    if(theta.size() > 0 && theta[0] >= deranged_beyond)
        columns(1, 0) = std::numeric_limits<double>::quiet_NaN();

    return columns;
}

capabilities banded()
{
    capabilities arm = baseline();
    arm.dk           = differential_kinematics_ops{.space_jacobian = &banded_space_jacobian};
    return arm;
}

const scene::window_route unreached = [](const std::shared_ptr<scene::imgui_window> &) {};

// A scene is created headlessly and a renderer robot needs no graphics context. The unload route is
// the one the composition is offered, so what a case reads is the decision reaching for it.
struct counting_site
{
    explicit counting_site(scheduler::scheduler &loop)
            : scene(threepp::Scene::create())
            , unloaded(0)
            , site{*scene, loop.main_strand(), *loop.make_strand(), [this] { ++unloaded; }, unreached, unreached, {}}
    {
    }

    std::shared_ptr<threepp::Scene> scene;
    int unloaded;
    scene::preset_site site;
};

std::weak_ptr<owned_arm> open(counting_site &at, const capabilities &arm, std::shared_ptr<scene::preset> &composed)
{
    std::weak_ptr<owned_arm> taken;
    const arm_window_composer capturing = [&taken](const arm_window_inputs &offered)
    {
        taken = offered.arm;
        return std::vector<std::shared_ptr<scene::imgui_window>>{};
    };

    composed = compose_arm(six_axis_machine(), at.site, attached_models{}, arm, trajectory::baseline(), rigid_motion::baseline(), joint_vector{}, capturing);
    REQUIRE(composed != nullptr);

    return taken;
}

int unloads_after(refusal reason, refusal_standing standing, bool routed)
{
    scheduler::scheduler loop(scheduler::inline_workers);
    counting_site at(loop);
    if(!routed)
        at.site.ask_unload = nullptr;
    std::shared_ptr<scene::preset> composed;
    const std::weak_ptr<owned_arm> taken = open(at, baseline(), composed);
    command(taken, [reason, standing](robot_controller &control, scene_robot &) { control.report_refusal("dk.manipulability", reason, standing); });
    REQUIRE(loop.drain().has_value());

    return at.unloaded;
}

struct outcome
{
    std::string reported;
    int unloaded;
};

outcome driving(const capabilities &arm, std::span<const double> through)
{
    scheduler::scheduler loop(scheduler::inline_workers);
    counting_site at(loop);
    captured_log captured;
    std::shared_ptr<scene::preset> composed;
    const std::weak_ptr<owned_arm> taken = open(at, arm, composed);
    for(double every : through)
        command(taken, [every](robot_controller &, scene_robot &driven) { driven.set_joint_positions(joint_vector::Constant(static_cast<Eigen::Index>(axes), every)); });
    REQUIRE(loop.drain().has_value());

    return outcome{captured.text(), at.unloaded};
}

std::size_t occurrences(const std::string &within, const std::string &named)
{
    std::size_t counted = 0;
    for(std::size_t at = within.find(named); at != std::string::npos; at = within.find(named, at + named.size()))
        ++counted;

    return counted;
}

}

TEST_CASE("a fatal refusal that is a fact about one request asks the composition it was made against to unload", "[manipulator][refusal]")
{
    CHECK(unloads_after(refusal::degenerate, refusal_standing::per_request, true) == 1);
}

TEST_CASE("the same fatal kind held true of the whole composition leaves it loaded", "[manipulator][refusal]")
{
    CHECK(unloads_after(refusal::degenerate, refusal_standing::composition_wide, true) == 0);
}

TEST_CASE("a kind that is not fatal leaves the composition loaded under either standing", "[manipulator][refusal]")
{
    CHECK(unloads_after(refusal::no_solution, refusal_standing::per_request, true) == 0);
    CHECK(unloads_after(refusal::no_solution, refusal_standing::composition_wide, true) == 0);
}

// A composition offering no route to say it cannot continue is a state rather than a defect.
TEST_CASE("a fatal refusal reported where no unload route was offered counts nothing and does not fault", "[manipulator][refusal]")
{
    CHECK(unloads_after(refusal::degenerate, refusal_standing::per_request, false) == 0);
}

TEST_CASE("a composition whose decomposition is refused at every configuration is named once and is not unloaded", "[manipulator][refusal]")
{
    const std::vector<double> repeatedly{0.75, 0.75, 0.75, 0.75};
    const outcome after = driving(banded(), repeatedly);

    CHECK(occurrences(after.reported, "dk.manipulability") == 1u);
    CHECK(after.reported.find("at every configuration") != std::string::npos);
    CHECK(after.unloaded == 0);
}

TEST_CASE("the standing report is made again after the refusal clears and returns", "[manipulator][refusal]")
{
    const std::vector<double> away_and_back{0.75, 0.0, 0.75};
    const outcome after = driving(banded(), away_and_back);

    CHECK(occurrences(after.reported, "dk.manipulability") == 2u);
    CHECK(after.unloaded == 0);
}

TEST_CASE("a composition decomposed at every configuration it is driven through names nothing", "[manipulator][refusal]")
{
    const std::vector<double> decomposable{0.25, 0.1};
    const outcome after = driving(banded(), decomposable);

    CHECK(after.reported.empty());
    CHECK(after.unloaded == 0);
}

// Nothing answers for a pose here, so the two latched reports stand beside each other.
TEST_CASE("the decomposition report and the pose refusal beside it each stand once and neither unloads", "[manipulator][refusal]")
{
    capabilities poseless = banded();
    poseless.fk           = forward_kinematics_ops{};

    const std::vector<double> repeatedly{0.75, 0.75};
    const outcome after = driving(poseless, repeatedly);

    CHECK(occurrences(after.reported, "dk.manipulability") == 1u);
    CHECK(occurrences(after.reported, "fk.forward_kinematics") == 1u);
    CHECK(after.unloaded == 0);
}
