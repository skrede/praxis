#ifndef HPP_GUARD_PRAXIS_TESTS_MANIPULATOR_SIX_AXIS_MACHINE_H
#define HPP_GUARD_PRAXIS_TESTS_MANIPULATOR_SIX_AXIS_MACHINE_H

#include "praxis/manipulator/types.h"

#include <meios/model.h>

#include <string>
#include <cstddef>
#include <utility>

namespace praxis::fixture {

using namespace manipulator;

inline constexpr std::size_t axes   = 6u;
inline constexpr double link_length = 0.25;

inline meios::link<> link_named(std::string name)
{
    meios::link<> record{};
    record.name = std::move(name);

    return record;
}

inline meios::joint<> revolute(std::string name, std::string parent, std::string child)
{
    meios::joint<> record{};
    record.name               = std::move(name);
    record.kind               = meios::joint_kind::revolute;
    record.parent             = std::move(parent);
    record.child              = std::move(child);
    record.axis               = {0.0, 0.0, 1.0};
    record.origin.translation = {link_length, 0.0, 0.0};

    return record;
}

// Six revolute joints in a row about the world z, each one link further along x than the one before,
// with the topology tables written out rather than reconstructed.
inline meios::model<> six_axis_machine()
{
    meios::model<> model;
    model.name  = "six_axis_machine";
    model.links = {link_named("base")};
    model.topo  = meios::robot_topology{{-1}, {-1}, {0}, {0}};

    for(std::size_t axis = 0u; axis < axes; ++axis)
    {
        const std::string child = "link" + std::to_string(axis + 1u);

        model.joints.push_back(revolute("joint" + std::to_string(axis + 1u), axis == 0u ? "base" : "link" + std::to_string(axis), child));
        model.links.push_back(link_named(child));
        model.topo.parent_of.push_back(static_cast<int>(axis));
        model.topo.joint_of.push_back(static_cast<int>(axis));
        model.topo.order.push_back(static_cast<int>(axis + 1u));
    }

    return model;
}

inline joint_vector folded()
{
    joint_vector q(axes);
    q << 0.3, -0.7, 0.45, 0.2, -0.55, 0.9;

    return q;
}

inline joint_vector outstretched()
{
    joint_vector q(axes);
    q << -0.8, 0.25, -0.15, 1.1, 0.6, -0.35;

    return q;
}

}

#endif
