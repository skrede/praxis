#ifndef HPP_GUARD_PRAXIS_TESTS_SUPPORT_BOOK_VIA_SCHEDULES_H
#define HPP_GUARD_PRAXIS_TESTS_SUPPORT_BOOK_VIA_SCHEDULES_H

#include "book_via_slots.h"

#include "praxis/trajectory/trajectory.h"

#include <span>
#include <array>
#include <cmath>
#include <vector>
#include <cstddef>
#include <utility>
#include <algorithm>

namespace praxis::tests {

// The step choosing the via points of a run from its configurations: each segment the time its
// slowest joint needs at its bound, eq. (2.3) inside under the form's sign test, rest at both ends,
// then the whole run stretched once by the largest ratio of a joint's peak speed to its bound. A run
// of one or two configurations goes through the same steps or, in closed form, takes 1.5 times its
// slowest joint's time, the peak of the cubic of Lynch & Park, Modern Robotics, sec. 9.2.2.1.
struct schedule_form
{
    peak_root peak;
    stretch_fit stretch;
    via_sign sign;
    short_run shorter;
};

inline constexpr std::array<schedule_form, 8> schedule_forms = []
{
    std::array<schedule_form, 8> forms{};
    for(std::size_t i = 0; i < forms.size(); ++i)
        forms[i] = schedule_form{static_cast<peak_root>(i / 4u), static_cast<stretch_fit>(i / 2u % 2u), via_sign::three_valued, static_cast<short_run>(i % 2u)};
    return forms;
}();

inline constexpr std::array<schedule_form, 3> schedule_sign_controls{
        schedule_form{peak_root::printed, stretch_fit::refit, via_sign::product, short_run::pipeline},
        schedule_form{peak_root::printed, stretch_fit::refit, via_sign::sign_bit, short_run::pipeline},
        schedule_form{peak_root::printed, stretch_fit::refit, via_sign::two_valued, short_run::pipeline},
};

inline double slowest_time(const trajectory::configuration &from, const trajectory::configuration &to, const trajectory::configuration &bound)
{
    double T = 0.0;
    for(Eigen::Index i = 0; i < from.size(); ++i)
        if(bound[i] > 0.0)
            T = std::max(T, std::abs(to[i] - from[i]) / bound[i]);

    return T;
}

inline void literal_velocities(const schedule_form &form, std::vector<trajectory::via_point> &run)
{
    for(std::size_t k = 0; k < run.size(); ++k)
        for(Eigen::Index i = 0; i < run[k].position.size(); ++i)
        {
            if(k == 0 || k + 1 == run.size())
            {
                run[k].velocity[i] = 0.0;
                continue;
            }
            const double before = (run[k].position[i] - run[k - 1].position[i]) / (run[k].time - run[k - 1].time);
            const double after  = (run[k + 1].position[i] - run[k].position[i]) / (run[k + 1].time - run[k].time);
            run[k].velocity[i]  = signs_differ(form.sign, before, after) ? 0.0 : 0.5 * (before + after);
        }
}

inline double literal_peak(const schedule_form &form, const std::vector<trajectory::via_point> &run, Eigen::Index i)
{
    double fastest = 0.0;
    for(std::size_t j = 0; j + 1 < run.size(); ++j)
    {
        const double T     = run[j + 1].time - run[j].time;
        const book_cubic c = book_segment(run[j].position[i], run[j + 1].position[i], run[j].velocity[i], run[j + 1].velocity[i], T);
        const double turn  = form.peak == peak_root::printed ? -c.a2 / (3.0 * c.a3) : -2.0 * c.a2 / (6.0 * c.a3);
        fastest            = std::max({fastest, std::abs(c.a1), std::abs(c.a1 + 2.0 * c.a2 * T + 3.0 * c.a3 * T * T)});
        if(c.a3 != 0.0 && turn > 0.0 && turn < T)
            fastest = std::max(fastest, std::abs(c.a1 + 2.0 * c.a2 * turn + 3.0 * c.a3 * turn * turn));
    }

    return fastest;
}

inline double literal_stretch(const schedule_form &form, const std::vector<trajectory::via_point> &run, const trajectory::configuration &bound)
{
    double s = 1.0;
    for(Eigen::Index i = 0; i < bound.size(); ++i)
        if(bound[i] > 0.0)
            s = std::max(s, literal_peak(form, run, i) / bound[i]);

    return s;
}

inline std::vector<trajectory::via_point> literal_schedule(const schedule_form &form, std::span<const trajectory::configuration> positions, const trajectory::configuration &j0,
                                                           const trajectory::configuration_limits &limits)
{
    std::vector<trajectory::configuration> q(positions.begin(), positions.end());
    if(q.size() == 1)
        q.insert(q.begin(), j0);

    std::vector<trajectory::via_point> run;
    for(std::size_t k = 0; k < q.size(); ++k)
        run.push_back({k == 0 ? 0.0 : run.back().time + slowest_time(q[k - 1], q[k], limits.velocity), q[k], trajectory::configuration::Zero(q[k].size())});
    if(run.size() == 2 && form.shorter == short_run::closed_form)
    {
        run.back().time = 1.5 * slowest_time(q[0], q[1], limits.velocity);
        return run;
    }

    literal_velocities(form, run);
    const double s = literal_stretch(form, run, limits.velocity);
    for(trajectory::via_point &at : run)
        at = trajectory::via_point{at.time * s, at.position, at.velocity / s};
    if(form.stretch == stretch_fit::refit)
        literal_velocities(form, run);

    return run;
}

template<const auto &Forms, std::size_t Form>
expected<std::vector<trajectory::via_point>, refusal> literal_via_point_schedule(std::span<const trajectory::configuration> positions, const trajectory::configuration &j0,
                                                                                 const trajectory::configuration_limits &limits)
{
    return literal_schedule(Forms[Form], positions, j0, limits);
}

using schedule_slot = expected<std::vector<trajectory::via_point>, refusal> (*)(std::span<const trajectory::configuration> positions, const trajectory::configuration &j0,
                                                                                const trajectory::configuration_limits &limits);

template<const auto &Forms, std::size_t... Index>
std::array<schedule_slot, sizeof...(Index)> literal_schedules(std::index_sequence<Index...>)
{
    return {&literal_via_point_schedule<Forms, Index>...};
}

template<const auto &Forms>
std::array<schedule_slot, Forms.size()> literal_schedules()
{
    return literal_schedules<Forms>(std::make_index_sequence<Forms.size()>{});
}

}

#endif
