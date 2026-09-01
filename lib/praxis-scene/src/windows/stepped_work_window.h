#ifndef HPP_GUARD_PRAXIS_SCENE_WINDOWS_STEPPED_WORK_WINDOW_H
#define HPP_GUARD_PRAXIS_SCENE_WINDOWS_STEPPED_WORK_WINDOW_H

#include "praxis/scene/visualizer.h"
#include "praxis/scene/imgui_window.h"

#include <string>
#include <vector>
#include <cstddef>

namespace praxis::scene {

class stepped_work_window : public imgui_window
{
public:
    stepped_work_window(std::string name, visualizer &v);
    stepped_work_window(const stepped_work_window &)            = delete;
    stepped_work_window(stepped_work_window &&)                 = delete;
    stepped_work_window &operator=(const stepped_work_window &) = delete;
    stepped_work_window &operator=(stepped_work_window &&)      = delete;
    ~stepped_work_window() override                             = default;

    void render() override;

private:
    visualizer &m_visualizer;

    void place_on_first_use() const;

    void render_table(const std::vector<stepped_work_report> &reported) const;

    void render_row(std::size_t index, const stepped_work_report &reported) const;
};

}

#endif
