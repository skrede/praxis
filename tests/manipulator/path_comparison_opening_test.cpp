#include "praxis/manipulator/types.h"
#include "praxis/manipulator/kinematics.h"
#include "praxis/manipulator/capabilities.h"
#include "praxis/manipulator/solver_adapters.h"
#include "praxis/manipulator/screw_chain_builder.h"
#include "praxis/manipulator/path_comparison_window.h"

#include "captured_log.h"

#include "praxis/trajectory/capabilities.h"

#include "praxis/rigid_motion/capabilities.h"

#include "praxis/optimization/capabilities.h"
#include "praxis/optimization/projected/solver.h"

#include <catch2/catch_test_macros.hpp>

#include <meios/urdf/load.h>
#include <meios/model.h>

#include <array>
#include <memory>
#include <string>
#include <vector>
#include <cstddef>
#include <optional>
#include <filesystem>

#ifndef PRAXIS_DEMO_RESOURCE_DIR
    #define PRAXIS_DEMO_RESOURCE_DIR ""
#endif

using namespace praxis;
using namespace praxis::manipulator;

namespace {

using task_space_path = decltype(trajectory::path_ops::screw);

struct deployed_arm
{
    const char *path;
    bool parameterized;
};

constexpr std::array<deployed_arm, 2> deployed_arms{deployed_arm{"ur_description/urdf/ur.urdf.xacro", true}, deployed_arm{"kuka_kr6_support/urdf/kr6r900sixx.xacro", false}};

std::optional<screw_chain> deployed_chain(const deployed_arm &arm)
{
    const std::filesystem::path root{PRAXIS_DEMO_RESOURCE_DIR};
    if(root.empty() || !std::filesystem::exists(root / arm.path))
        return std::nullopt;

    meios::load_options options;
    options.package_roots.push_back(root);
    options.eval       = meios::eval_policy::fail;
    options.on_missing = meios::missing_asset::fail;
    if(arm.parameterized)
    {
        options.args["ur_type"] = "ur3e";
        options.args["name"]    = "ur3e";
    }

    auto loaded = meios::load(root / arm.path, options);
    REQUIRE(loaded.has_value());

    const expected<screw_chain, refusal> chain = build_screw_chain(loaded->robot);
    REQUIRE(chain.has_value());

    return *chain;
}

std::vector<kinematics> solve_routes(const screw_chain &chain)
{
    const capabilities arm              = manipulator::baseline();
    optimization::solver_ops projected  = optimization::baseline().solver;
    projected.solve                     = &optimization::projected::solve;
    inverse_kinematics_ops adapted      = arm.ik;
    adapted.inverse_kinematics          = &body_inverse_kinematics;
    const rigid_motion::capabilities rm = rigid_motion::baseline();

    return {kinematics::compose(chain, arm.fk, arm.dk, arm.ik, rm.screw, rm.frame, optimization::baseline().solver).value(),
            kinematics::compose(chain, arm.fk, arm.dk, adapted, rm.screw, rm.frame, projected).value()};
}

std::vector<transform> sampled(task_space_path shape, const kinematics &solver, const joint_vector &first, const joint_vector &second)
{
    const rigid_motion::capabilities rm = rigid_motion::baseline();
    const transform from                = solver.fk_solve(first).value();
    const transform to                  = solver.fk_solve(second).value();

    std::vector<transform> poses;
    for(std::size_t step = 0; step < path_comparison_window::drawn_points; ++step)
        poses.push_back(shape(rm.screw, rm.frame, from, to, static_cast<double>(step) / static_cast<double>(path_comparison_window::drawn_points - 1u)).value());

    return poses;
}

}

TEST_CASE("both task-space shapes between the ends a comparison opens at resolve on each deployed arm without a reported turn", "[manipulator][comparison]")
{
    bool any = false;
    for(const deployed_arm &arm : deployed_arms)
    {
        const std::optional<screw_chain> chain = deployed_chain(arm);
        if(!chain)
            continue;

        any                               = true;
        const joint_vector first          = path_comparison_window::opening_first(chain->joint_count());
        const joint_vector second         = path_comparison_window::opening_second(chain->joint_count());
        const trajectory::path_ops shapes = trajectory::baseline().path;
        for(const kinematics &solver : solve_routes(*chain))
            for(const task_space_path shape : {shapes.decoupled, shapes.screw})
            {
                INFO(arm.path);
                tests::captured_log captured;

                const auto motion = manipulator::baseline().trajectory.task_space_waypoints(trajectory::baseline().trajectory, manipulator::baseline().motion, solver,
                                                                                            sampled(shape, solver, first, second), first, chain->limits);

                CHECK(motion.has_value());
                CHECK(captured.text().empty());
            }
    }
    if(!any)
        SKIP("no robot description is deployed for this configuration");
}
