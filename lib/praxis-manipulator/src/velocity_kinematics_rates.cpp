#include "praxis/manipulator/velocity_kinematics_window.h"

#include <imgui.h>

namespace praxis::manipulator {

void velocity_kinematics_window::render_rate_switches()
{
    if(m_rate_weighted && ImGui::Checkbox("Columns times joint rates", &*m_rate_weighted))
        m_drawn.set_columns_weighted(*m_rate_weighted);
    if(m_twist && ImGui::Checkbox("Twist J times rates", &*m_twist))
        m_drawn.set_twist_shown(*m_twist);
}

void velocity_kinematics_window::tell_rates() const
{
    if(m_rate_weighted)
        m_drawn.set_columns_weighted(*m_rate_weighted);
    if(m_twist)
        m_drawn.set_twist_shown(*m_twist);
}

}
