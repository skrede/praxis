#ifndef HPP_GUARD_PRAXIS_TESTS_SUPPORT_BOOK_VIA_SLOTS_H
#define HPP_GUARD_PRAXIS_TESTS_SUPPORT_BOOK_VIA_SLOTS_H

#include "book_via_forms.h"
#include "book_via_points.h"

#include "praxis/trajectory/trajectory.h"

#include <span>
#include <array>
#include <cmath>
#include <memory>
#include <vector>
#include <cstddef>
#include <cstdint>
#include <utility>
#include <algorithm>

namespace praxis::tests {

// The via-point slot and the step choosing its via points, each written in one of the arrangements
// a reader of Lynch & Park, Modern Robotics, sec. 9.2.2.1 and 9.3, eq. (9.26)-(9.29), and Biagiotti &
// Melchiorri, Trajectory Planning for Automatic Machines and Robots, sec. 2.1.4, eq. (2.3), arrives at.
// Neither checks its input.

enum class coefficient_spelling : std::uint8_t
{
    printed,
    expanded,
    difference
};

enum class evaluation_order : std::uint8_t
{
    powers,
    nested
};

enum class short_run : std::uint8_t
{
    pipeline,
    closed_form
};

struct via_slot_form
{
    coefficient_spelling spelling;
    segment_lookup lookup;
    evaluation_order order;
};

inline constexpr std::array<via_slot_form, 12> via_slot_forms = []
{
    std::array<via_slot_form, 12> forms{};
    for(std::size_t i = 0; i < forms.size(); ++i)
        forms[i] = via_slot_form{static_cast<coefficient_spelling>(i / 4u), static_cast<segment_lookup>(i / 2u % 2u), static_cast<evaluation_order>(i % 2u)};
    return forms;
}();

inline book_cubic spelled_segment(coefficient_spelling spelling, double q0, double q1, double v0, double v1, double T)
{
    if(spelling == coefficient_spelling::expanded)
        return {q0, v0, 3.0 * (q1 - q0) / (T * T) - (2.0 * v0 + v1) / T, 2.0 * (q0 - q1) / std::pow(T, 3) + (v0 + v1) / (T * T)};
    if(spelling == coefficient_spelling::difference)
    {
        const double d = q1 - q0;
        return {q0, v0, (3.0 * d - (2.0 * v0 + v1) * T) / (T * T), ((v0 + v1) * T - 2.0 * d) / (T * T * T)};
    }

    return book_segment(q0, q1, v0, v1, T);
}

class literal_via_slot_run : public trajectory::trajectory_generator
{
public:
    literal_via_slot_run(const via_slot_form &form, std::span<const trajectory::via_point> via)
            : m_form(form)
            , m_via(via.begin(), via.end())
    {
    }

    expected<trajectory::trajectory_sample, refusal> sample(double t) const override
    {
        t                                = std::clamp(t, 0.0, m_via.back().time);
        const std::size_t j              = segment(t);
        const trajectory::via_point &q0  = m_via[j];
        const trajectory::via_point &q1  = m_via[j + 1];
        const double dt                  = t - q0.time;
        trajectory::trajectory_sample at = {q0.position, q0.position, q0.position};
        for(Eigen::Index i = 0; i < q0.position.size(); ++i)
        {
            const book_cubic c = spelled_segment(m_form.spelling, q0.position[i], q1.position[i], q0.velocity[i], q1.velocity[i], q1.time - q0.time);
            at.position[i]     = m_form.order == evaluation_order::nested ? c.a0 + dt * (c.a1 + dt * (c.a2 + dt * c.a3)) : c.a0 + c.a1 * dt + c.a2 * dt * dt + c.a3 * dt * dt * dt;
            at.velocity[i]     = c.a1 + 2.0 * c.a2 * dt + 3.0 * c.a3 * dt * dt;
            at.acceleration[i] = 2.0 * c.a2 + 6.0 * c.a3 * dt;
        }

        return at;
    }

    double duration() const override
    {
        return m_via.back().time;
    }

private:
    via_slot_form m_form;
    std::vector<trajectory::via_point> m_via;

    std::size_t segment(double t) const
    {
        const std::size_t last = m_via.size() - 2;
        if(m_form.lookup == segment_lookup::upper_bound)
            return std::min(static_cast<std::size_t>(std::upper_bound(m_via.begin(), m_via.end(), t, [](double at, const trajectory::via_point &p) { return at < p.time; }) -
                                                     m_via.begin()),
                            last + 1) -
                    1;

        std::size_t j = 0;
        while(j < last && m_via[j + 1].time < t)
            ++j;

        return j;
    }
};

template<std::size_t Form>
expected<std::unique_ptr<trajectory::trajectory_generator>, refusal> literal_via_slot(std::span<const trajectory::via_point> via)
{
    return std::unique_ptr<trajectory::trajectory_generator>(std::make_unique<literal_via_slot_run>(via_slot_forms[Form], via));
}

using via_slot = expected<std::unique_ptr<trajectory::trajectory_generator>, refusal> (*)(std::span<const trajectory::via_point> via_points);

template<std::size_t... Index>
std::array<via_slot, sizeof...(Index)> literal_via_slots(std::index_sequence<Index...>)
{
    return {&literal_via_slot<Index>...};
}

inline std::array<via_slot, via_slot_forms.size()> literal_via_slots()
{
    return literal_via_slots(std::make_index_sequence<via_slot_forms.size()>{});
}

}

#endif
