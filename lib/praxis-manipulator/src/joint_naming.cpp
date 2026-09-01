#include "joint_naming.h"

#include <string>
#include <vector>
#include <cstddef>

namespace praxis::manipulator {

std::vector<std::string> named_joints(std::size_t joints)
{
    std::vector<std::string> named(joints);
    for(std::size_t joint = 0u; joint < named.size(); ++joint)
        named[joint] = "j" + std::to_string(joint + 1u);

    return named;
}

}
