#include "trajectory_executor.h"

#include "praxis/manipulator/csv.h"

#include <spdlog/spdlog.h>

#include <chrono>
#include <memory>
#include <vector>
#include <cstdint>
#include <utility>
#include <algorithm>
#include <filesystem>

namespace praxis::manipulator {

// The recording is written once at the end rather than row by row, and the end is the motion's own or
// the conclusion a retiring composition asks for, so what a reader finds after an interruption is
// every row taken up to it. An inactive one gathers nothing and writes nothing. Each row is stamped
// with the time played since the motion was loaded.
class motion_recording
{
public:
    explicit motion_recording(recording_parameters parameters)
            : m_parameters(std::move(parameters))
            , m_positions()
            , m_timestamps()
    {
    }

    void record(scheduler::seconds at, const joint_vector &position)
    {
        if(!m_parameters.active)
            return;
        m_positions.push_back(position);
        m_timestamps.push_back(static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(at).count()));
    }

    std::filesystem::path where() const
    {
        return m_parameters.active ? m_parameters.directory : std::filesystem::path();
    }

    bool write() const
    {
        if(!m_parameters.active)
            return true;

        const bool stamped = std_vector_to_csv_file(m_timestamps, m_parameters.directory / date_time_stamped_filename("timestamp.csv"));
        const bool played  = eigen_vector_to_csv_file(m_positions, m_parameters.directory / date_time_stamped_filename("trajectory.csv"));
        if(!stamped || !played)
            spdlog::error("praxis: the trajectory recording in '{}' could not be written, so the {} rows it gathered are lost", m_parameters.directory.string(), m_positions.size());

        return stamped && played;
    }

private:
    recording_parameters m_parameters;
    std::vector<joint_vector> m_positions;
    std::vector<std::uint64_t> m_timestamps;
};

trajectory_executor::trajectory_executor(scene_robot &driven)
        : m_started(false)
        , m_played(0.0)
        , m_robot(driven)
        , m_elapsed(scheduler::seconds::zero())
        , m_parameters()
        , m_recording()
        , m_motion()
{
}

trajectory_executor::~trajectory_executor() = default;

recording_parameters trajectory_executor::recording() const
{
    return m_parameters;
}

void trajectory_executor::set_recording(recording_parameters parameters)
{
    m_parameters = std::move(parameters);
}

bool trajectory_executor::pending() const
{
    return m_motion != nullptr && !m_started;
}

bool trajectory_executor::executing() const
{
    return m_motion != nullptr;
}

expected<std::filesystem::path, refusal> trajectory_executor::stop()
{
    if(m_motion == nullptr)
        return std::filesystem::path();

    const std::filesystem::path written = m_recording->where();
    const expected<void, refusal> ended = conclude();
    if(!ended)
        return unexpected(ended.error());

    return written;
}

// The motion held is what rejects a second one: it is the whole of the executor's state, so there is
// nothing else a caller could observe it through.
bool trajectory_executor::execute(std::unique_ptr<trajectory::trajectory_generator> motion)
{
    if(motion == nullptr || m_motion != nullptr)
        return false;

    m_started   = false;
    m_played    = 0.0;
    m_elapsed   = scheduler::seconds::zero();
    m_recording = std::make_unique<motion_recording>(m_parameters);
    m_motion    = std::move(motion);

    return true;
}

expected<void, refusal> trajectory_executor::advance(scheduler::step_delta step, double factor)
{
    if(m_motion == nullptr)
        return {};

    m_started = true;
    m_elapsed += step.value;
    m_played += factor * step.value.count();

    const double span                                              = m_motion->duration();
    const expected<trajectory::trajectory_sample, refusal> sampled = m_motion->sample(std::min(m_played, span));
    if(!sampled)
    {
        static_cast<void>(conclude());
        return unexpected(sampled.error());
    }

    m_robot.set_joint_positions(sampled->position);
    m_recording->record(m_elapsed, sampled->position);
    if(m_played >= span)
        static_cast<void>(conclude());

    return {};
}

// Both files are written where the last sample was observed, so whoever services this strand waits
// out the two writes. The motion is cleared whether or not they landed.
expected<void, refusal> trajectory_executor::conclude()
{
    const bool written = m_recording->write();
    m_recording.reset();
    m_motion.reset();
    if(!written)
        return unexpected(refusal::unsupported_input);

    return {};
}

}
