#ifndef HPP_GUARD_PRAXIS_SCENE_WINDOWS_PRESET_WINDOW_H
#define HPP_GUARD_PRAXIS_SCENE_WINDOWS_PRESET_WINDOW_H

#include "praxis/scene/visualizer.h"
#include "praxis/scene/imgui_window.h"

#include <string>
#include <cstddef>
#include <optional>

namespace praxis::scene {

class preset_window : public imgui_window
{
public:
    preset_window(std::string name, visualizer &v);

    void render() override;

private:
    bool m_remember;
    std::size_t m_selected;
    visualizer &m_visualizer;
    // What was asked for next, held only until nothing is composed any more, so a question the
    // release asks is asked before the change rather than after it.
    std::optional<std::string> m_requested;

    void render_body();
    void render_question();

    void take_requested();
    void request(std::string name);

    void answer(bool keep);

    bool render_preset_selection();
};

}

#endif
