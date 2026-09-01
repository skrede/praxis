#ifndef HPP_GUARD_PRAXIS_SCENE_LABELED_VALUE_WINDOW_H
#define HPP_GUARD_PRAXIS_SCENE_LABELED_VALUE_WINDOW_H

#include "praxis/scene/imgui_window.h"

#include <string>
#include <vector>
#include <functional>

namespace praxis::scene {

// An empty label marks a cell as unlabeled. A row whose every cell is unlabeled is drawn as aligned
// columns rather than as label-value pairs, and a run of consecutive such rows shares one alignment
// sized to the widest of them. An unlabeled cell in any other row carries a number and nothing to
// call it, drawn alone in the digits a labeled cell prints, which is how several values stand under
// one label. A non-empty statement is drawn in place of the cell's number, which is how a reading
// carrying no number for a cell says so rather than standing a zero there.
struct labeled_value
{
    float value;
    std::string label;
    std::string stated = std::string();
};

// A non-empty message is drawn in place of the rows rather than above them: a reading that carries
// one has no values to show.
struct readout
{
    std::string message;
    std::vector<std::vector<labeled_value>> rows;
};

using readout_source = std::function<readout()>;

class labeled_value_window : public imgui_window
{
public:
    labeled_value_window(std::string name, std::function<void()> controls, readout_source source);

    void render() override;

private:
    readout_source m_source;
    std::function<void()> m_controls;
};

}

#endif
