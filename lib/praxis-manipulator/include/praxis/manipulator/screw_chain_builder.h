#ifndef HPP_GUARD_PRAXIS_MANIPULATOR_SCREW_CHAIN_BUILDER_H
#define HPP_GUARD_PRAXIS_MANIPULATOR_SCREW_CHAIN_BUILDER_H

#include "praxis/manipulator/screw_chain.h"

#include "praxis/extension/refusal.h"

#include "praxis/compat/expected.h"

#include <meios/model.h>

#include <string>
#include <vector>

namespace praxis::manipulator {

struct screw_chain_options
{
    std::string tip_link;
    double default_velocity;
    double acceleration_ratio;
    double unbounded_position;

    screw_chain_options();
};

// The screw axes are taken in the frame of the model's root link, which is also the frame the
// rendered robot is expressed in; that is what lets one parsed model feed both this derivation and
// the scene graph built from it. An empty tip link selects the leaf reachable through the most
// actuated joints.
expected<screw_chain, refusal> build_screw_chain(const meios::model<> &model, const screw_chain_options &options);

expected<screw_chain, refusal> build_screw_chain(const meios::model<> &model);

expected<std::vector<std::string>, refusal> actuated_joint_names(const meios::model<> &model, const screw_chain_options &options);

expected<std::string, refusal> tip_link_name(const meios::model<> &model, const screw_chain_options &options);

}

#endif
