#include "praxis/manipulator/loadable_robot_stencil.h"

#include <spdlog/spdlog.h>

#include <span>
#include <cmath>
#include <cstddef>
#include <algorithm>

namespace praxis::manipulator {

namespace {

refusal rates_of_another_width(std::size_t told, std::size_t columns)
{
    spdlog::error("praxis: {} joint rates were told to a drawing of {} columns, so no column is weighted by them and the rates standing are left as they were", told, columns);

    return refusal::unsupported_input;
}

refusal rate_not_finite()
{
    spdlog::error("praxis: a joint rate told to the drawing is not a finite number, so no column is weighted by the rates told and the rates standing are left as they were");

    return refusal::unsupported_input;
}

}

expected<void, refusal> loadable_robot_stencil::set_joint_rates(std::span<const double> rates)
{
    if(rates.empty())
    {
        m_rates.clear();
        return {};
    }
    if(rates.size() != m_column_arrows.size())
        return unexpected(rates_of_another_width(rates.size(), m_column_arrows.size()));
    if(!std::ranges::all_of(rates, [](double rate) { return std::isfinite(rate); }))
        return unexpected(rate_not_finite());

    m_rates.assign(rates.begin(), rates.end());

    return {};
}

void loadable_robot_stencil::set_columns_weighted(bool weighted)
{
    m_weighted = weighted;
}

bool loadable_robot_stencil::columns_weighted() const
{
    return m_weighted;
}

}
