#include "praxis/evaluation/tolerance.h"
#include "praxis/evaluation/generation.h"

#include <array>
#include <cmath>
#include <numbers>

namespace praxis::evaluation {

namespace {

constexpr double half_turn    = std::numbers::pi_v<double>;
constexpr double quarter_turn = 0.5 * std::numbers::pi_v<double>;

// Metres.
constexpr double position_extent = 2.0;

// Metres per radian.
constexpr double pitch_extent = 2.25;

// Six orderings name three distinct axes and six repeat the first, which is the whole set.
constexpr int rotation_orders = 12;

// The ends of the axis-count range, both reachable.
constexpr int fewest_axes = 1;
constexpr int most_axes   = 8;

// A standard-library hash is not required to be stable across implementations or runs, so the
// mixing below is FNV-1a written out. It starts from the seed rather than from the published offset
// basis, so two seeds separate every name and two names separate every seed.
std::uint64_t folded(std::uint64_t hash, std::uint64_t byte)
{
    return (hash ^ byte) * 0x100000001b3ULL;
}

std::uint64_t mixed(std::uint64_t seed, std::string_view name, spread drawn_from, std::size_t index)
{
    std::uint64_t hash = 0xcbf29ce484222325ULL ^ seed;

    for(char letter : name)
        hash = folded(hash, static_cast<unsigned char>(letter));

    hash = folded(hash, static_cast<std::uint64_t>(drawn_from));

    for(int shift = 0; shift < 64; shift += 8)
        hash = folded(hash, (static_cast<std::uint64_t>(index) >> shift) & 0xffULL);

    return hash;
}

// A standard-library distribution, like a standard-library hash, is not required to draw the same
// values on every implementation, while the engine's own output sequence is specified exactly. The
// draws below are therefore written out over raw engine output, so a recorded seed names one corpus
// everywhere rather than one corpus per standard library.

// The engine's top 53 bits scaled into [0, 1): uniformly spaced, and never reaching one.
double unit_interval(std::mt19937_64 &engine)
{
    return static_cast<double>(engine() >> 11) * 0x1.0p-53;
}

double drawn_between(std::mt19937_64 &engine, double lower, double upper)
{
    return lower + (upper - lower) * unit_interval(engine);
}

// The remainder's bias is one part in 2^60 at the widest count drawn here, beneath anything a
// consumer of these draws could observe.
int drawn_below(std::mt19937_64 &engine, int count)
{
    return static_cast<int>(engine() % static_cast<std::uint64_t>(count));
}

// Box-Muller from two uniform draws, the first taken from (0, 1] so its logarithm is finite.
double drawn_normal(std::mt19937_64 &engine)
{
    const double radius = std::sqrt(-2.0 * std::log(1.0 - unit_interval(engine)));
    const double angle  = 2.0 * half_turn * unit_interval(engine);

    return radius * std::cos(angle);
}

}

case_source::case_source(std::uint64_t seed, spread drawn_from)
        : m_spread(drawn_from)
        , m_seed(seed)
        , m_engine(seed)
{
}

case_source case_source::for_slot(std::uint64_t seed, std::string_view slot, spread drawn_from)
{
    return at_case(seed, slot, drawn_from, 0);
}

case_source case_source::at_case(std::uint64_t seed, std::string_view slot, spread drawn_from, std::size_t index)
{
    return case_source(mixed(seed, slot, drawn_from, index), drawn_from);
}

std::uint64_t case_source::seed() const
{
    return m_seed;
}

spread case_source::drawn_from() const
{
    return m_spread;
}

bool case_source::half_the_time()
{
    return drawn_below(m_engine, 2) == 1;
}

double case_source::standard_normal()
{
    return drawn_normal(m_engine);
}

// The decades between the comparison tolerance and unity are where a comparison stops behaving as it
// does in the bulk, so the displacement is log-uniform over them rather than uniform.
double case_source::offset_from_singular()
{
    const double decades  = drawn_between(m_engine, std::log10(default_tolerance), 0.0);
    const double distance = std::pow(10.0, decades);

    return half_the_time() ? distance : -distance;
}

double case_source::angle_radians()
{
    if(m_spread != spread::near_singular)
        return drawn_between(m_engine, -half_turn, half_turn);

    constexpr std::array<double, 3> singular{0.0, half_turn, -half_turn};
    const auto which = static_cast<std::size_t>(drawn_below(m_engine, 3));

    return singular[which] + offset_from_singular();
}

double case_source::pitch()
{
    if(m_spread == spread::near_singular)
        return offset_from_singular();

    return drawn_between(m_engine, -pitch_extent, pitch_extent);
}

Eigen::Vector3d case_source::normal_triple()
{
    const double x = standard_normal();
    const double y = standard_normal();
    const double z = standard_normal();

    return Eigen::Vector3d(x, y, z);
}

Eigen::Vector3d case_source::unit_direction()
{
    Eigen::Vector3d drawn = normal_triple();
    while(drawn.norm() < default_tolerance)
        drawn = normal_triple();

    return drawn.normalized();
}

Eigen::Vector3d case_source::position_metres()
{
    const double x = drawn_between(m_engine, -position_extent, position_extent);
    const double y = drawn_between(m_engine, -position_extent, position_extent);
    const double z = drawn_between(m_engine, -position_extent, position_extent);

    return Eigen::Vector3d(x, y, z);
}

Eigen::Vector3d case_source::euler_triple_radians()
{
    constexpr std::array<double, 4> singular{0.0, quarter_turn, -quarter_turn, half_turn};

    const double first = drawn_between(m_engine, -half_turn, half_turn);
    const double middle =
            m_spread == spread::near_singular ? singular[static_cast<std::size_t>(drawn_below(m_engine, 4))] + offset_from_singular() : drawn_between(m_engine, -half_turn, half_turn);
    const double last = drawn_between(m_engine, -half_turn, half_turn);

    return Eigen::Vector3d(first, middle, last);
}

Eigen::Vector3d case_source::angular_part()
{
    if(half_the_time())
        return Eigen::Vector3d::Zero();

    return normal_triple();
}

Eigen::Vector3d case_source::linear_part()
{
    return normal_triple();
}

std::uint8_t case_source::axis_order_index()
{
    return static_cast<std::uint8_t>(drawn_below(m_engine, rotation_orders));
}

std::size_t case_source::axis_count()
{
    return static_cast<std::size_t>(fewest_axes + drawn_below(m_engine, most_axes - fewest_axes + 1));
}

}
