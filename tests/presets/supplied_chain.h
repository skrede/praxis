#ifndef HPP_GUARD_PRAXIS_TESTS_PRESETS_SUPPLIED_CHAIN_H
#define HPP_GUARD_PRAXIS_TESTS_PRESETS_SUPPLIED_CHAIN_H

#include "drawn_lines.h"
#include "scratch_directory.h"

#include "praxis/presets/screw_table.h"

#include "praxis/rigid_motion/capabilities.h"

#include "praxis/config/error.h"
#include "praxis/config/store.h"
#include "praxis/config/writer.h"
#include "praxis/config/binding.h"
#include "praxis/config/document.h"

#include <catch2/catch_test_macros.hpp>

#include <threepp/scenes/Scene.hpp>

#include <Eigen/Core>

#include <string>
#include <vector>
#include <cstddef>
#include <fstream>
#include <filesystem>
#include <system_error>

namespace praxis::fixture {

inline const std::filesystem::path &chain_scratch()
{
    return shared_scratch_directory();
}

// One axis per joint, each through a point further out along x, so no two rows are the same line
// and none of them stands where the described arm puts a joint of its own.
inline std::vector<screw_axis> a_supplied_chain(std::size_t joints)
{
    const rigid_motion::capabilities motions = rigid_motion::baseline();

    std::vector<screw_axis> named;
    for(std::size_t joint = 0u; joint < joints; ++joint)
    {
        const Eigen::Vector3d through(0.05 * static_cast<double>(joint + 1u), 0.0, 0.0);
        const Eigen::Vector3d along = joint % 2u == 0u ? Eigen::Vector3d::UnitZ() : Eigen::Vector3d::UnitY();

        const expected<screw_axis, refusal> built = motions.screw.screw_axis_from_point_direction_pitch(through, along, 0.0);
        REQUIRE(built.has_value());

        named.push_back(built.value());
    }

    return named;
}

inline std::string chain_triple(const char *named, const Eigen::Vector3d &value)
{
    return "<" + std::string(named) + " x=\"" + config::exact_text(value.x()) + "\" y=\"" + config::exact_text(value.y()) + "\" z=\"" + config::exact_text(value.z()) + "\"/>";
}

// A table written by hand rather than by the window, read through the keyspace the preset declares
// rather than through anything the suite spells for itself. The home pose is written as a position
// with no turn, which is what the arms these cases are written over stand at.
inline config::document kept_chain(const std::vector<screw_axis> &screws, const Eigen::Vector3d &home, const char *name)
{
    const std::filesystem::path where = chain_scratch() / name;
    std::ofstream out(where, std::ios::binary | std::ios::trunc);
    out << "<screw_table><screws>";
    out << "<home>" << chain_triple("position", home) << chain_triple("orientation", Eigen::Vector3d::Zero()) << "</home>";
    for(std::size_t joint = 0u; joint < screws.size(); ++joint)
        out << "<joint index=\"" << joint + 1u << "\">" << chain_triple("angular", screws[joint].head<3>()) << chain_triple("linear", screws[joint].tail<3>()) << "</joint>";
    out << "</screws></screw_table>\n";
    out.close();

    const expected<config::document, config::error> read = config::load(presets::screw_table_keyspace(), config::resolve(where, chain_scratch()));
    INFO((read ? std::string() : read.error().message));
    REQUIRE(read.has_value());

    return read.value();
}

inline config::document kept_chain(const std::vector<screw_axis> &screws, const char *name)
{
    return kept_chain(screws, Eigen::Vector3d::Zero(), name);
}

inline config::binding chain_binding(const char *name)
{
    std::error_code ignored;
    std::filesystem::remove(chain_scratch() / name, ignored);

    return presets::screw_table_binding(name, chain_scratch());
}

inline presets::screw_table_source supplied_from(const config::document &values, const config::binding &into)
{
    return presets::screw_table_source{std::string(presets::screw_table_path), values, presets::screw_table_route(into, rigid_motion::baseline().frame)};
}

}

#endif
