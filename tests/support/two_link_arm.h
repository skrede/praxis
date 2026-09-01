#ifndef HPP_GUARD_PRAXIS_TESTS_SUPPORT_TWO_LINK_ARM_H
#define HPP_GUARD_PRAXIS_TESTS_SUPPORT_TWO_LINK_ARM_H

#include <meios/model.h>

#include <string>
#include <utility>

namespace praxis::fixture {

inline meios::link<> link_named(std::string name)
{
    meios::link<> record{};
    record.name = std::move(name);

    return record;
}

// One revolute joint from base to tool, with the topology tables written out rather than
// reconstructed, so a fixture that perturbs exactly one entry can attribute a refusal to that entry.
inline meios::model<> two_link_arm(std::string name, meios::robot_topology topo)
{
    meios::model<> model;
    model.name  = std::move(name);
    model.links = {link_named("base"), link_named("tool")};

    meios::joint<> shoulder{};
    shoulder.name   = "shoulder";
    shoulder.kind   = meios::joint_kind::revolute;
    shoulder.parent = "base";
    shoulder.child  = "tool";
    shoulder.axis   = {0.0, 0.0, 1.0};
    model.joints    = {shoulder};
    model.topo      = std::move(topo);

    return model;
}

inline meios::model<> well_formed_arm()
{
    return two_link_arm("well_formed_arm", meios::robot_topology{{-1, 0}, {-1, 0}, {0}, {0, 1}});
}

struct malformation
{
    std::string what;
    meios::model<> model;
};

}

#endif
