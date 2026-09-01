#ifndef HPP_GUARD_PRAXIS_MANIPULATOR_IK_SEED_WINDOW_H
#define HPP_GUARD_PRAXIS_MANIPULATOR_IK_SEED_WINDOW_H

#include "praxis/manipulator/types.h"
#include "praxis/manipulator/arm_snapshot.h"

#include "praxis/scene/imgui_window.h"

#include "praxis/config/writer.h"
#include "praxis/config/document.h"
#include "praxis/config/configurable.h"

#include <Eigen/Core>

#include <string>
#include <vector>
#include <cstddef>
#include <cstdint>
#include <string_view>

namespace praxis::manipulator {

// The starting configurations a solve over several starts runs from, in the order it runs them. The
// list is held here and nothing is commanded from it: the only thing read off the publication is
// how many joint values a start has.
class ik_seed_window : public scene::imgui_window, public config::configurable
{
    // What one row's controls asked for on the frame they were drawn.
    enum class row_edit : std::uint8_t
    {
        none,
        remove,
        raise
    };

public:
    struct settings
    {
        std::vector<joint_vector> seeds;

        explicit settings(std::vector<joint_vector> chosen = std::vector<joint_vector>());
    };

    // The starts a chain of `joints` joints opens at: its home configuration first, then a Halton
    // spread over a whole turn in every joint -- Halton, Numerische Mathematik 2 (1960), 84-90 --
    // so the first n of them are the spread of n. A chain of no joints has no start to run from.
    // A settings value carrying no start at all opens at these, an empty list being the absence of
    // a choice rather than one.
    static std::vector<joint_vector> opening_seeds(std::size_t joints);

    ik_seed_window(std::string name, arm_reader seen);
    ik_seed_window(std::string name, arm_reader seen, const settings &state, std::string at = std::string());

    settings state() const;

    void render() override;

    std::string_view settings_path() const override
    {
        return m_settings_at;
    }

    std::vector<config::edit> settings_edits(const config::document &carried) const override;

    // A window no key path was named for has nowhere to write, so it offers nothing.
    const config::configurable *as_configurable() const override
    {
        return m_settings_at.empty() ? nullptr : this;
    }

private:
    arm_reader m_seen;
    std::string m_settings_at;
    // Each row in degrees, which is the unit the rows are read and typed in; `state()` is where the
    // radians every other surface carries are taken.
    std::vector<Eigen::VectorXf> m_degrees;

    void append(std::size_t joints);
    void render_seeds(std::size_t joints);
    row_edit render_row(std::size_t row);
    void accept(const joint_vector &seed, std::size_t row, std::size_t joints);
};

}

#endif
