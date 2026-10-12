#include "praxis/manipulator/velocity_configuration.h"
#include "praxis/manipulator/velocity_kinematics_window.h"

#include <vector>
#include <cstddef>

namespace praxis::manipulator {

namespace {

constexpr std::size_t angular = static_cast<std::size_t>(jacobian_block::angular);
constexpr std::size_t linear  = static_cast<std::size_t>(jacobian_block::linear);

}

velocity_kinematics_window::settings velocity_kinematics_window::state() const
{
    return settings{.frame         = m_frame.value(),
                    .reading       = m_reading.value(),
                    .angular       = m_shown[angular],
                    .linear        = m_shown[linear],
                    .columns       = m_columns,
                    .capped        = m_capped,
                    .ellipsoids    = m_ellipsoids,
                    .highlighted   = m_highlighted,
                    .rate_weighted = m_rate_weighted,
                    .twist         = m_twist};
}

std::vector<config::edit> velocity_kinematics_window::settings_edits(const config::document &carried) const
{
    return unsaved_velocity_kinematics(carried, state(), m_settings_at);
}

}
