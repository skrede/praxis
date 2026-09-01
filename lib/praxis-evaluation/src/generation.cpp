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
    return std::uniform_int_distribution<int>(0, 1)(m_engine) == 1;
}

double case_source::standard_normal()
{
    return std::normal_distribution<double>(0.0, 1.0)(m_engine);
}

// The decades between the comparison tolerance and unity are where a comparison stops behaving as it
// does in the bulk, so the displacement is log-uniform over them rather than uniform.
double case_source::offset_from_singular()
{
    const double decades  = std::uniform_real_distribution<double>(std::log10(default_tolerance), 0.0)(m_engine);
    const double distance = std::pow(10.0, decades);

    return half_the_time() ? distance : -distance;
}

double case_source::angle_radians()
{
    if(m_spread != spread::near_singular)
        return std::uniform_real_distribution<double>(-half_turn, half_turn)(m_engine);

    constexpr std::array<double, 3> singular{0.0, half_turn, -half_turn};
    const auto which = static_cast<std::size_t>(std::uniform_int_distribution<int>(0, 2)(m_engine));

    return singular[which] + offset_from_singular();
}

double case_source::pitch()
{
    if(m_spread == spread::near_singular)
        return offset_from_singular();

    return std::uniform_real_distribution<double>(-pitch_extent, pitch_extent)(m_engine);
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
    std::uniform_real_distribution<double> over_the_range(-position_extent, position_extent);

    const double x = over_the_range(m_engine);
    const double y = over_the_range(m_engine);
    const double z = over_the_range(m_engine);

    return Eigen::Vector3d(x, y, z);
}

Eigen::Vector3d case_source::euler_triple_radians()
{
    constexpr std::array<double, 4> singular{0.0, quarter_turn, -quarter_turn, half_turn};
    std::uniform_real_distribution<double> over_a_turn(-half_turn, half_turn);

    const double first = over_a_turn(m_engine);
    const double middle =
            m_spread == spread::near_singular ? singular[static_cast<std::size_t>(std::uniform_int_distribution<int>(0, 3)(m_engine))] + offset_from_singular() : over_a_turn(m_engine);
    const double last = over_a_turn(m_engine);

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
    return static_cast<std::uint8_t>(std::uniform_int_distribution<int>(0, rotation_orders - 1)(m_engine));
}

std::size_t case_source::axis_count()
{
    return static_cast<std::size_t>(std::uniform_int_distribution<int>(fewest_axes, most_axes)(m_engine));
}

}
