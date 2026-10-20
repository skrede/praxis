#ifndef HPP_GUARD_PRAXIS_MANIPULATOR_PATH_COMPARISON_PATHS_H
#define HPP_GUARD_PRAXIS_MANIPULATOR_PATH_COMPARISON_PATHS_H

#include "praxis/manipulator/types.h"
#include "praxis/manipulator/path_comparison_window.h"

#include "praxis/trajectory/path.h"

#include "praxis/extension/refusal.h"

#include "praxis/compat/expected.h"

#include <vector>

namespace praxis::manipulator {

// The configurations the joint-space shape passes through, at the shared count and the shared path
// parameters, or the refusal the shape answered, so a caller runs what it was given rather than a
// run with holes in it.
expected<std::vector<joint_vector>, refusal> configurations_along(const trajectory::path_ops &shapes, const joint_vector &from, const joint_vector &to);

// Every pose carried through `convert`, or the first refusal it answered.
template<typename Convert>
expected<std::vector<transform>, refusal> each_converted(const std::vector<transform> &poses, Convert convert)
{
    std::vector<transform> converted;
    converted.reserve(poses.size());
    for(const transform &pose : poses)
    {
        const expected<transform, refusal> one = convert(pose);
        if(!one)
            return praxis::unexpected(one.error());
        converted.push_back(*one);
    }

    return converted;
}

}

#endif
