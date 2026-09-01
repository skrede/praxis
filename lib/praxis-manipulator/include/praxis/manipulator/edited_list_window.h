#ifndef HPP_GUARD_PRAXIS_MANIPULATOR_EDITED_LIST_WINDOW_H
#define HPP_GUARD_PRAXIS_MANIPULATOR_EDITED_LIST_WINDOW_H

#include "praxis/manipulator/types.h"
#include "praxis/manipulator/edited_pose.h"
#include "praxis/manipulator/arm_snapshot.h"
#include "praxis/manipulator/edited_list_rows.h"

#include "praxis/scene/imgui_window.h"

#include "praxis/config/writer.h"
#include "praxis/config/document.h"
#include "praxis/config/configurable.h"

#include "praxis/rigid_motion/frame.h"

#include <string>
#include <vector>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <string_view>

namespace praxis::manipulator {

// A list of rows edited in place: rows are added from where the arm stands, removed, reordered and
// typed into. The list is held here and nothing is commanded from it. What a row is, what its
// controls are and where it is written is `list_row_traits<Row>`, and two lists differ nowhere else.
template<typename Row>
class edited_list_window : public scene::imgui_window, public config::configurable
{
    // What one row's controls asked for on the frame they were drawn.
    enum class row_edit : std::uint8_t
    {
        none,
        remove,
        raise,
        lower
    };

    // What one row answered on the frame it was drawn: the edit of the list it asked for, and
    // whether its own values were typed into.
    struct row_asked
    {
        row_edit edit;
        bool typed;
    };

public:
    using row_traits = list_row_traits<Row>;
    using shown_row  = typename row_traits::shown_row;

    struct settings
    {
        std::vector<Row> rows;

        explicit settings(std::vector<Row> chosen = std::vector<Row>());
    };

    // The rows a chain of `joints` joints opens at. A settings value carrying no row at all opens at
    // these, an empty list being the absence of a choice rather than one.
    static std::vector<Row> opening_rows(std::size_t joints);

    // `edited` is told whenever a row is added, removed, reordered or typed into, so a composer can
    // route a change to whatever elsewhere was computed from these rows. The list itself keeps no
    // interest in what that is.
    edited_list_window(std::string name, arm_reader seen, const rigid_motion::frame_ops &injected, std::function<void()> edited = {});
    edited_list_window(std::string name, arm_reader seen, const rigid_motion::frame_ops &injected, const settings &state, std::string at = std::string(),
                       std::function<void()> edited = {});

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
    // Each row in the unit its own controls are typed in, which the trait names; `state()` is where
    // the unit every other surface carries is taken.
    std::vector<shown_row> m_shown;
    rigid_motion::frame_ops m_frame;
    std::function<void()> m_edited_cb;

    // True where the list changed, which a row an arm cannot answer for and a reordering that would
    // carry a row past either end both leave it unchanged.
    bool append(const arm_snapshot &seen);
    bool applied(row_edit asked, std::size_t named);

    void render_rows(const arm_snapshot &seen);

    // `values_at` is where the row's values begin, which is the offset the header naming the columns
    // stands at, so that a column and the values under it are drawn from the same number.
    row_asked render_row(std::size_t row, float values_at);
    void accept(const Row &given, std::size_t row, std::size_t joints);
};

using joint_waypoint_list = edited_list_window<joint_vector>;
using pose_waypoint_list  = edited_list_window<edited_pose>;

}

#endif
