#ifndef HPP_GUARD_PRAXIS_MANIPULATOR_ROBOT_SCENE_NODES_H
#define HPP_GUARD_PRAXIS_MANIPULATOR_ROBOT_SCENE_NODES_H

#include "robot/mesh_library.h"

#include "praxis/manipulator/scene_robot_builder.h"

#include <threepp/core/Object3D.hpp>

#include <meios/model.h>

#include <memory>

namespace praxis::manipulator {

void apply_origin(threepp::Object3D &object, const meios::transform<double> &origin);

std::shared_ptr<threepp::Object3D> build_link_node(const meios::model<> &model, const meios::link<> &link, mesh_library &meshes, const scene_robot_options &options);

}

#endif
