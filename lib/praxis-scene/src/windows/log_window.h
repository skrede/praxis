#ifndef HPP_GUARD_PRAXIS_SCENE_WINDOWS_LOG_WINDOW_H
#define HPP_GUARD_PRAXIS_SCENE_WINDOWS_LOG_WINDOW_H

#include "praxis/scene/log_buffer.h"
#include "praxis/scene/imgui_window.h"

#include <deque>
#include <memory>
#include <string>

namespace praxis::scene {

class log_window : public imgui_window
{
public:
    log_window(std::string name, std::shared_ptr<log_buffer> messages);
    log_window(const log_window &)            = delete;
    log_window(log_window &&)                 = delete;
    log_window &operator=(const log_window &) = delete;
    log_window &operator=(log_window &&)      = delete;
    ~log_window() override                    = default;

    void render() override;

private:
    severity m_level;
    std::deque<log_entry> m_shown;
    std::shared_ptr<log_buffer> m_messages;

    void render_level_control();

    void accept(log_entry entry);

    void place_on_first_use() const;

    void render_entry(const log_entry &entry) const;
};

}

#endif
