#ifndef HPP_GUARD_PRAXIS_TESTS_SUPPORT_BOOK_VIA_POINTS_H
#define HPP_GUARD_PRAXIS_TESTS_SUPPORT_BOOK_VIA_POINTS_H

#include "book_via_forms.h"

#include "praxis/trajectory/baseline/trajectory.h"

#include "praxis/trajectory/trajectory.h"

#include <span>
#include <array>
#include <cmath>
#include <memory>
#include <vector>
#include <cstddef>
#include <utility>
#include <algorithm>

namespace praxis::tests {

struct book_cubic
{
    double a0, a1, a2, a3;
};

// Lynch & Park, Modern Robotics, eq. (9.26)-(9.29).
inline book_cubic book_segment(double q0, double q1, double v0, double v1, double T)
{
    return {q0, v0, (3.0 * q1 - 3.0 * q0 - 2.0 * v0 * T - v1 * T) / (T * T), (2.0 * q0 + (v0 + v1) * T - 2.0 * q1) / (T * T * T)};
}

class book_via_run : public trajectory::trajectory_generator
{
public:
    book_via_run(const via_form &form, std::span<const trajectory::configuration> q, const trajectory::configuration &bound)
            : m_form(form)
            , m_first(q.front())
            , m_last(q.back())
    {
        m_t.push_back(0.0);
        for(std::size_t k = 1; k < q.size(); ++k)
            m_t.push_back(m_t.back() + slowest(q[k - 1], q[k], bound));
        fit(q);
        stretch(q, bound);
    }

    expected<trajectory::trajectory_sample, refusal> sample(double t) const override
    {
        const double T = m_t.back();
        if(m_form.ends == via_ends::at_rest && (t <= 0.0 || t >= T))
            return trajectory::trajectory_sample{t <= 0.0 ? m_first : m_last, trajectory::configuration::Zero(m_first.size()), trajectory::configuration::Zero(m_first.size())};

        t                   = std::clamp(t, 0.0, T);
        const std::size_t j = segment(t);
        const double dt     = t - m_t[j];
        trajectory::trajectory_sample out{m_first, m_first, m_first};
        for(Eigen::Index i = 0; i < m_first.size(); ++i)
        {
            const book_cubic &c = m_a[j][static_cast<std::size_t>(i)];
            out.position[i]     = c.a0 + c.a1 * dt + c.a2 * dt * dt + c.a3 * dt * dt * dt;
            out.velocity[i]     = c.a1 + 2.0 * c.a2 * dt + 3.0 * c.a3 * dt * dt;
            out.acceleration[i] = 2.0 * c.a2 + 6.0 * c.a3 * dt;
        }

        return out;
    }

    double duration() const override
    {
        return m_t.back();
    }

private:
    via_form m_form;
    trajectory::configuration m_first, m_last;
    std::vector<double> m_t;
    std::vector<std::vector<book_cubic>> m_a;

    static double slowest(const trajectory::configuration &from, const trajectory::configuration &to, const trajectory::configuration &bound)
    {
        double T = 0.0;
        for(Eigen::Index i = 0; i < from.size(); ++i)
            if(bound[i] > 0.0)
                T = std::max(T, std::abs(to[i] - from[i]) / bound[i]);

        return T;
    }

    double velocity_at(std::span<const trajectory::configuration> q, std::size_t k, Eigen::Index i) const
    {
        if(k == 0 || k + 1 == q.size())
            return 0.0;

        const double before = (q[k][i] - q[k - 1][i]) / (m_t[k] - m_t[k - 1]);
        const double after  = (q[k + 1][i] - q[k][i]) / (m_t[k + 1] - m_t[k]);

        return signs_differ(m_form.sign, before, after) ? 0.0 : 0.5 * (before + after);
    }

    void fit(std::span<const trajectory::configuration> q)
    {
        m_a.assign(q.size() - 1, std::vector<book_cubic>(static_cast<std::size_t>(m_first.size())));
        for(std::size_t j = 0; j + 1 < q.size(); ++j)
            for(Eigen::Index i = 0; i < m_first.size(); ++i)
                m_a[j][static_cast<std::size_t>(i)] = book_segment(q[j][i], q[j + 1][i], velocity_at(q, j, i), velocity_at(q, j + 1, i), m_t[j + 1] - m_t[j]);
    }

    double peak(std::size_t i) const
    {
        double fastest = 0.0;
        for(std::size_t j = 0; j < m_a.size(); ++j)
        {
            const book_cubic &c = m_a[j][i];
            const double T      = m_t[j + 1] - m_t[j];
            const double turn   = m_form.peak == peak_root::printed ? -c.a2 / (3.0 * c.a3) : -2.0 * c.a2 / (6.0 * c.a3);
            fastest             = std::max({fastest, std::abs(c.a1), std::abs(c.a1 + 2.0 * c.a2 * T + 3.0 * c.a3 * T * T)});
            if(c.a3 != 0.0 && turn > 0.0 && turn < T)
                fastest = std::max(fastest, std::abs(c.a1 + 2.0 * c.a2 * turn + 3.0 * c.a3 * turn * turn));
        }

        return fastest;
    }

    void stretch(std::span<const trajectory::configuration> q, const trajectory::configuration &bound)
    {
        double s = 1.0;
        for(Eigen::Index i = 0; i < m_first.size(); ++i)
            if(bound[i] > 0.0)
                s = std::max(s, peak(static_cast<std::size_t>(i)) / bound[i]);
        if(s <= 1.0)
            return;

        for(double &t : m_t)
            t *= s;
        if(m_form.stretch == stretch_fit::refit)
            fit(q);
        else
            for(std::vector<book_cubic> &cubics : m_a)
                for(book_cubic &c : cubics)
                    c = book_cubic{c.a0, c.a1 / s, c.a2 / (s * s), c.a3 / (s * s * s)};
    }

    std::size_t segment(double t) const
    {
        const std::size_t last = m_a.size() - 1;
        if(m_form.lookup == segment_lookup::upper_bound)
            return std::min(static_cast<std::size_t>(std::upper_bound(m_t.begin(), m_t.end(), t) - m_t.begin()), m_a.size()) - 1;

        std::size_t j = 0;
        while(j < last && m_t[j + 1] < t)
            ++j;

        return j;
    }
};

template<const auto &Forms, std::size_t Form>
expected<std::unique_ptr<trajectory::trajectory_generator>, refusal> book_via_points(std::span<const trajectory::configuration> waypoints, const trajectory::configuration &j0,
                                                                                     const trajectory::configuration_limits &limits)
{
    if(waypoints.size() < 3)
        return trajectory::joint_space_waypoints(waypoints, j0, limits);

    return std::unique_ptr<trajectory::trajectory_generator>(std::make_unique<book_via_run>(Forms[Form], waypoints, limits.velocity));
}

using via_point_slot = expected<std::unique_ptr<trajectory::trajectory_generator>, refusal> (*)(std::span<const trajectory::configuration> waypoints,
                                                                                                const trajectory::configuration &j0, const trajectory::configuration_limits &limits);

template<const auto &Forms, std::size_t... Index>
std::array<via_point_slot, sizeof...(Index)> book_via_point_slots(std::index_sequence<Index...>)
{
    return {&book_via_points<Forms, Index>...};
}

template<const auto &Forms>
std::array<via_point_slot, Forms.size()> book_via_point_slots()
{
    return book_via_point_slots<Forms>(std::make_index_sequence<Forms.size()>{});
}

}

#endif
