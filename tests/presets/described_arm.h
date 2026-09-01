#ifndef HPP_GUARD_PRAXIS_TESTS_PRESETS_DESCRIBED_ARM_H
#define HPP_GUARD_PRAXIS_TESTS_PRESETS_DESCRIBED_ARM_H

#include "scratch_directory.h"

#include "praxis/presets/arm.h"

#include <string>
#include <cstddef>
#include <fstream>
#include <filesystem>
#include <system_error>

namespace praxis::fixture {

// A serial arm of the named width, every joint a decimetre above the one before it and turning about
// an axis alternating between two of the frame's own, written into a scratch directory of its own
// that it removes as a tree. The axes are not all parallel, so a placement taken from anything but
// the derived chain misses them.
struct described_arm
{
    described_arm(std::size_t joints, const std::string &named)
            : directory(scratch_directory())
            , where(directory / (named + ".urdf"))
    {
        std::ofstream document(where);
        document << "<?xml version=\"1.0\"?>\n<robot name=\"" << named << "\">\n  <link name=\"link0\"/>\n";
        for(std::size_t joint = 0; joint < joints; ++joint)
        {
            const std::string above = std::to_string(joint);
            const std::string below = std::to_string(joint + 1);
            document << "  <link name=\"link" << below << "\"/>\n  <joint name=\"joint" << below << "\" type=\"revolute\">\n"
                     << "    <parent link=\"link" << above << "\"/>\n    <child link=\"link" << below << "\"/>\n"
                     << "    <origin xyz=\"0 0 0.1\" rpy=\"0 0 0\"/>\n    <axis xyz=\"" << (joint % 2 == 0 ? "0 0 1" : "0 1 0") << "\"/>\n"
                     << "    <limit lower=\"-3.14\" upper=\"3.14\" effort=\"10\" velocity=\"1\"/>\n  </joint>\n";
        }
        document << "</robot>\n";
    }

    described_arm(const described_arm &) = delete;

    ~described_arm()
    {
        std::error_code ignored;
        std::filesystem::remove_all(directory, ignored);
    }

    std::filesystem::path directory;
    std::filesystem::path where;
};

inline presets::arm_scenario described_by(const std::filesystem::path &description)
{
    presets::arm_scenario chosen;
    chosen.description = description;

    return chosen;
}

}

#endif
