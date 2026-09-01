#ifndef HPP_GUARD_PRAXIS_SCENE_PLOT_WINDOW_H
#define HPP_GUARD_PRAXIS_SCENE_PLOT_WINDOW_H

#include "praxis/scene/imgui_window.h"

#include <string>
#include <vector>
#include <functional>

namespace praxis::scene {

// A curve whose two spans differ in length is drawn to the shorter of them.
struct plot_series
{
    std::string name;
    std::vector<double> abscissa;
    std::vector<double> ordinate;
};

// A frame carries no abscissa of its own, because the reading it belongs to carries one for all of
// its frames.
struct plot_frame
{
    std::string ordinate_label;
    // The scale governs this frame's ordinate alone, and a value at or below zero is not plotted on it.
    bool logarithmic_ordinate;
    std::vector<plot_series> series;
};

// A non-empty message is drawn in place of the plot rather than above it: a reading that carries one
// has no curve to show.
struct plot_reading
{
    std::string message;
    std::string abscissa_label;
    std::vector<plot_frame> frames;
};

using plot_source = std::function<plot_reading()>;

class plot_window : public imgui_window
{
public:
    plot_window(std::string name, std::function<void()> controls, plot_source source);

    void render() override;

private:
    plot_source m_source;
    std::function<void()> m_controls;
};

}

#endif
