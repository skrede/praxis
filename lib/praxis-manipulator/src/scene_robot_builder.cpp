#include "model_chain.h"
#include "praxis/manipulator/scene_robot_builder.h"

#include "robot/emit_order.h"
#include "robot/mesh_library.h"
#include "robot/scene_nodes.h"

#include <threepp/core/Object3D.hpp>

#include <threepp/math/Vector3.hpp>

#include <string>
#include <cstddef>
#include <optional>

namespace praxis::manipulator {

namespace {

constexpr float axis_epsilon = 1e-6f;

threepp::Robot::JointType to_joint_type(meios::joint_kind kind)
{
    switch(kind)
    {
        case meios::joint_kind::revolute:
        case meios::joint_kind::continuous:
            return threepp::Robot::JointType::Revolute;
        case meios::joint_kind::prismatic:
            return threepp::Robot::JointType::Prismatic;
        default:
            return threepp::Robot::JointType::Fixed;
    }
}

std::optional<threepp::Robot::JointRange> to_joint_range(const meios::joint<> &joint)
{
    if(joint.kind == meios::joint_kind::continuous || !joint.limits)
        return std::nullopt;

    return threepp::Robot::JointRange{static_cast<float>(joint.limits->lower), static_cast<float>(joint.limits->upper)};
}

threepp::Vector3 joint_axis(const meios::joint<> &joint)
{
    threepp::Vector3 axis(static_cast<float>(joint.axis.x), static_cast<float>(joint.axis.y), static_cast<float>(joint.axis.z));
    if(axis.length() < axis_epsilon)
        return {1.f, 0.f, 0.f};

    return axis.normalize();
}

void add_joint(threepp::Robot &robot, const meios::joint<> &joint)
{
    auto object  = std::make_shared<threepp::Object3D>();
    object->name = joint.name;
    apply_origin(*object, joint.origin);

    robot.addJoint(
            object,
            threepp::Robot::JointInfo{
                    .axis = joint_axis(joint), .type = to_joint_type(joint.kind), .name = joint.name, .range = to_joint_range(joint), .parent = joint.parent, .child = joint.child});
}

// Through joint_above, which bounds the table's index and its entry alike: the topology check that
// admitted this model leaves the joint table to it, so the two reads here have no bound of their own.
void add_joints(threepp::Robot &robot, const meios::model<> &model, const std::vector<int> &order)
{
    for(const int index : order)
        if(const meios::joint<> *joint = joint_above(model, index))
            add_joint(robot, *joint);
}

}

scene_robot_options::scene_robot_options()
        : visuals(true)
        , colliders(true)
        , show_colliders(false)
        , mesh_root()
{
}

expected<std::shared_ptr<threepp::Robot>, refusal> build_scene_robot(const meios::model<> &model, const scene_robot_options &options)
{
    const auto root = root_link(model);
    if(!root)
        return unexpected(root.error());

    const auto order = emit_order(model);
    mesh_library meshes;

    auto robot  = std::make_shared<threepp::Robot>();
    robot->name = model.name;
    for(const int index : order)
        robot->addLink(build_link_node(model, model.links[static_cast<std::size_t>(index)], meshes, options));

    add_joints(*robot, model, order);
    robot->finalize();
    robot->showColliders(options.show_colliders);

    return robot;
}

expected<std::shared_ptr<threepp::Robot>, refusal> build_scene_robot(const meios::model<> &model)
{
    return build_scene_robot(model, scene_robot_options());
}

}
