#include "model_chain.h"
#include "praxis/manipulator/conversions.h"
#include "praxis/manipulator/screw_chain_builder.h"

#include <Eigen/Geometry>

#include <spdlog/spdlog.h>

#include <string>
#include <vector>
#include <cstddef>
#include <numbers>
#include <utility>

namespace praxis::manipulator {

namespace {

constexpr double axis_epsilon = 1e-9;

struct joint_bounds
{
    std::vector<double> velocity;
    std::vector<double> acceleration;
    std::vector<double> lower;
    std::vector<double> upper;
};

struct chain_accumulator
{
    transform pose;
    std::vector<screw_axis> screws;
    joint_bounds bounds;
};

Eigen::Vector3d to_eigen(const meios::vector3<double> &vector)
{
    return {vector.x, vector.y, vector.z};
}

transform origin_transform(const meios::transform<double> &origin)
{
    const Eigen::AngleAxisd yaw(origin.rotation.yaw, Eigen::Vector3d::UnitZ());
    const Eigen::AngleAxisd pitch(origin.rotation.pitch, Eigen::Vector3d::UnitY());
    const Eigen::AngleAxisd roll(origin.rotation.roll, Eigen::Vector3d::UnitX());

    transform pose         = transform::Identity();
    pose.block<3, 3>(0, 0) = (yaw * pitch * roll).toRotationMatrix();
    pose.block<3, 1>(0, 3) = to_eigen(origin.translation);

    return pose;
}

expected<screw_axis, refusal> joint_screw(const meios::joint<> &joint, const transform &pose)
{
    // The axis a description gives lives in the child link frame; a space-form screw needs it in
    // the root link frame, hence the rotation by the frame's zero-configuration orientation.
    const Eigen::Vector3d axis = pose.block<3, 3>(0, 0) * to_eigen(joint.axis);
    if(axis.norm() < axis_epsilon)
    {
        spdlog::error("praxis: joint '{}' has a degenerate axis", joint.name);
        return unexpected(refusal::degenerate);
    }

    const Eigen::Vector3d s = axis.normalized();
    const Eigen::Vector3d q = pose.block<3, 1>(0, 3);

    // Modern Robotics Eq. 3.24/3.25: a revolute screw is (s, -s x q) about the point q on the axis,
    // a prismatic one is (0, s).
    screw_axis screw;
    if(joint.kind == meios::joint_kind::prismatic)
        screw << Eigen::Vector3d::Zero(), s;
    else
        screw << s, -s.cross(q);

    return screw;
}

void append_bounds(joint_bounds &bounds, const meios::joint<> &joint, const screw_chain_options &options)
{
    // A continuous joint carries no position bounds; the description format still records the
    // element for its effort and velocity, leaving both bounds at zero.
    const bool bounded = joint.limits.has_value() && joint.kind != meios::joint_kind::continuous;
    bounds.lower.push_back(bounded ? joint.limits->lower : -options.unbounded_position);
    bounds.upper.push_back(bounded ? joint.limits->upper : options.unbounded_position);

    const double velocity = joint.limits && joint.limits->velocity > 0.0 ? joint.limits->velocity : options.default_velocity;
    bounds.velocity.push_back(velocity);
    bounds.acceleration.push_back(velocity * options.acceleration_ratio);
}

expected<void, refusal> fold_joint(chain_accumulator &chain, const meios::joint<> &joint, const screw_chain_options &options)
{
    // An origin maps the parent link frame onto the child link frame, so accumulating them walks
    // the zero-configuration pose down the chain; a fixed joint contributes only this factor, which
    // is how it folds into the next screw's origin -- or into the home pose.
    chain.pose = chain.pose * origin_transform(joint.origin);
    if(!is_actuated(joint.kind))
        return {};

    auto screw = joint_screw(joint, chain.pose);
    if(!screw)
        return unexpected(screw.error());

    chain.screws.push_back(std::move(*screw));
    append_bounds(chain.bounds, joint, options);
    return {};
}

expected<void, refusal> fold_link(chain_accumulator &chain, const meios::model<> &model, int link, const screw_chain_options &options)
{
    const meios::joint<> *joint = joint_above(model, link);
    if(!joint)
    {
        spdlog::error("praxis: model '{}' does not address a joint above link index {}", model.name, link);
        return unexpected(refusal::degenerate);
    }

    return fold_joint(chain, *joint, options);
}

joint_limits to_joint_limits(const joint_bounds &bounds)
{
    return {to_joint_vector(bounds.velocity), to_joint_vector(bounds.acceleration), to_joint_vector(bounds.lower), to_joint_vector(bounds.upper)};
}

}

screw_chain_options::screw_chain_options()
        : tip_link()
        , default_velocity(std::numbers::pi)
        , acceleration_ratio(0.5)
        , unbounded_position(2.0 * std::numbers::pi)
{
}

expected<screw_chain, refusal> build_screw_chain(const meios::model<> &model, const screw_chain_options &options)
{
    const auto links = link_chain(model, options.tip_link);
    if(!links)
        return unexpected(links.error());

    chain_accumulator chain{transform::Identity(), {}, {}};
    for(std::size_t i = 1; i < links->size(); ++i)
    {
        const auto folded = fold_link(chain, model, (*links)[i], options);
        if(!folded)
            return unexpected(folded.error());
    }

    if(chain.screws.empty())
    {
        spdlog::error("praxis: model '{}' has no actuated joint between its root and tip links", model.name);
        return unexpected(refusal::unsupported_input);
    }

    return screw_chain{chain.pose, std::move(chain.screws), to_joint_limits(chain.bounds)};
}

expected<screw_chain, refusal> build_screw_chain(const meios::model<> &model)
{
    return build_screw_chain(model, screw_chain_options());
}

expected<std::vector<std::string>, refusal> actuated_joint_names(const meios::model<> &model, const screw_chain_options &options)
{
    const auto links = link_chain(model, options.tip_link);
    if(!links)
        return unexpected(links.error());

    std::vector<std::string> names;
    for(std::size_t i = 1; i < links->size(); ++i)
    {
        const meios::joint<> *joint = joint_above(model, (*links)[i]);
        if(joint && is_actuated(joint->kind))
            names.push_back(joint->name);
    }

    return names;
}

expected<std::string, refusal> tip_link_name(const meios::model<> &model, const screw_chain_options &options)
{
    const auto links = link_chain(model, options.tip_link);
    if(!links)
        return unexpected(links.error());

    return model.links[static_cast<std::size_t>(links->back())].name;
}

}
