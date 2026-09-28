#ifndef HPP_GUARD_PRAXIS_TESTS_PRESETS_SAVED_DOCUMENT_H
#define HPP_GUARD_PRAXIS_TESTS_PRESETS_SAVED_DOCUMENT_H

#include "praxis/presets/arm_registration.h"

#include "praxis/scene/preset.h"
#include "praxis/scene/imgui_window.h"

#include "praxis/config/error.h"
#include "praxis/config/store.h"
#include "praxis/config/writer.h"
#include "praxis/config/binding.h"
#include "praxis/config/document.h"
#include "praxis/config/configurable.h"

#include "praxis/compat/expected.h"

#include <catch2/catch_test_macros.hpp>

#include <span>
#include <memory>
#include <string>
#include <vector>
#include <cstddef>
#include <fstream>
#include <filesystem>

namespace praxis::fixture {

// Six joints at zero, matching the description every document here names, with `beside` written after
// the start.
inline std::string arm_body(const std::string &scenario, const std::string &beside)
{
    std::string joints;
    for(std::size_t axis = 0u; axis < 6u; ++axis)
        joints += "<joint index=\"" + std::to_string(axis) + "\" degrees=\"0\"/>";

    return "<arm><preset name=\"Saved\" scenario=\"" + scenario + "\"/><description path=\"six.urdf\"/><initial>" + joints + "</initial>" + beside + "</arm>";
}

inline config::location arm_document(const std::filesystem::path &directory, const std::string &file, const std::string &body)
{
    std::ofstream out(directory / file, std::ios::binary | std::ios::trunc);
    out << body << "\n";
    out.close();

    return config::resolve(file, directory);
}

inline config::document read_arm_document(const config::location &at)
{
    const config::outcome read = config::load_or_defaults(presets::arm_keyspace(), at, config::expectation::partial);
    INFO((read.failure.has_value() ? read.failure->message : std::string()));
    REQUIRE_FALSE(read.failure.has_value());

    return read.values;
}

// What a composition's windows would write into `carried`, gathered the way a save gathers them.
inline std::vector<config::edit> offered_by(const scene::preset &composed, const config::document &carried)
{
    std::vector<const config::configurable *> shown;
    for(const std::shared_ptr<scene::imgui_window> &panel : composed.windows)
        if(const config::configurable *one = panel->as_configurable(); one != nullptr)
            shown.push_back(one);

    return config::shown_edits(shown, carried);
}

inline config::document saved_into(const config::location &at, std::span<const config::edit> changes)
{
    const expected<void, config::error> written = config::save(presets::arm_keyspace(), at, changes);
    INFO((written.has_value() ? std::string() : written.error().message));
    REQUIRE(written.has_value());

    return read_arm_document(at);
}

inline void write_model(const std::filesystem::path &where)
{
    std::filesystem::create_directories(where.parent_path());

    std::ofstream document(where);
    document << "solid praxis\nfacet normal 0 0 1\nouter loop\n"
             << "vertex 0 0 0\nvertex 0.1 0 0\nvertex 0 0.1 0\n"
             << "endloop\nendfacet\nendsolid praxis\n";
}

inline std::string text_in(const config::document &values, const std::string &key)
{
    const expected<std::string, config::error> read = values.text(key);

    return read.has_value() ? *read : std::string();
}

}

#endif
