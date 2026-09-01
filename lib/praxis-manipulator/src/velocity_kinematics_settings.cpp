#include "praxis/manipulator/velocity_configuration.h"
#include "praxis/manipulator/velocity_kinematics_window.h"

#include <vector>

namespace praxis::manipulator {

std::vector<config::edit> velocity_kinematics_window::settings_edits(const config::document &carried) const
{
    return config::unsaved_edits(carried, write_velocity_kinematics(state(), m_settings_at));
}

}
