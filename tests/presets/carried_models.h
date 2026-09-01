#ifndef HPP_GUARD_PRAXIS_TESTS_PRESETS_CARRIED_MODELS_H
#define HPP_GUARD_PRAXIS_TESTS_PRESETS_CARRIED_MODELS_H

#include "opened_arm.h"

#include "praxis/presets/arm.h"

#include "praxis/manipulator/tool_window.h"
#include "praxis/manipulator/world_object_window.h"

#include "praxis/rigid_motion/axis_order.h"

#include <Eigen/Core>

#include <string>
#include <fstream>
#include <filesystem>
#include <system_error>

namespace praxis::fixture {

// A one-facet model written into a scratch directory of its own that it removes as a tree. Both the
// tool and the world object reach the scene through the loader and nowhere else, so a case driving
// either of them needs one.
struct written_model
{
    written_model(const std::string &named, float edge)
            : directory(scratch_directory())
            , where(directory / named)
    {
        std::ofstream document(where);
        document << "solid praxis\nfacet normal 0 0 1\nouter loop\n"
                 << "vertex 0 0 0\nvertex " << edge << " 0 0\nvertex 0 " << edge << " 0\n"
                 << "endloop\nendfacet\nendsolid praxis\n";
    }

    written_model(const written_model &) = delete;

    ~written_model()
    {
        std::error_code ignored;
        std::filesystem::remove_all(directory, ignored);
    }

    std::filesystem::path directory;
    std::filesystem::path where;
};

// A scenario naming both models and asking for each of them on opening, which is what a composition
// drawing them is read against: a path alone leaves the tool window at its loader and the model it
// names unattached.
inline presets::arm_scenario carrying_models(const std::filesystem::path &description, const std::filesystem::path &tool, const std::filesystem::path &world)
{
    const Eigen::Vector3f none = Eigen::Vector3f::Zero();
    const Eigen::Vector3f unit = Eigen::Vector3f::Ones();

    presets::arm_scenario chosen = described_by(description);
    chosen.tool         = manipulator::tool_window::settings(true, tool.string(), manipulator::tool_window::tool_view::kinematics_transform, none, axis_order::zyx, unit, none, none,
                                                             axis_order::zyx, none);
    chosen.world_object = manipulator::world_object_window::settings(true, world.string(), manipulator::world_object_window::world_view::transform, unit, none, none);

    return chosen;
}

}

#endif
