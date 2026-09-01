#ifndef HPP_GUARD_PRAXIS_MANIPULATOR_SCENE_ROBOT_BUILDER_H
#define HPP_GUARD_PRAXIS_MANIPULATOR_SCENE_ROBOT_BUILDER_H

#include "praxis/extension/refusal.h"

#include "praxis/compat/expected.h"

#include <threepp/objects/Robot.hpp>

#include <meios/model.h>

#include <memory>
#include <filesystem>

namespace praxis::manipulator {

struct scene_robot_options
{
    bool visuals;
    bool colliders;
    bool show_colliders;
    std::filesystem::path mesh_root;

    scene_robot_options();
};

// The nodes are laid out in the frame of the model's root link, which is also the frame the screw
// axes derived from the same model are taken in. An empty mesh root resolves a relative mesh path
// against the directory the link was declared in.
expected<std::shared_ptr<threepp::Robot>, refusal> build_scene_robot(const meios::model<> &model, const scene_robot_options &options);

expected<std::shared_ptr<threepp::Robot>, refusal> build_scene_robot(const meios::model<> &model);

}

#endif
