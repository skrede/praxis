#include "fixtures.h"

#include "praxis/manipulator/arm_state.h"
#include "praxis/manipulator/robot_controller.h"
#include "praxis/manipulator/trajectory_recording_window.h"

#include "praxis/scheduler/scheduler.h"

#include <catch2/catch_test_macros.hpp>

#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <utility>
#include <algorithm>
#include <filesystem>

using namespace praxis::fixture;
using namespace praxis::scheduler;
using namespace praxis::manipulator;

namespace {

time_point dictated{};

time_point reading()
{
    return dictated;
}

clock_source dictating()
{
    dictated = time_point{};
    return clock_source{&reading};
}

// One service covers every period the raised reading spans, so the advance may be coarser than the period the playback is registered at.
constexpr seconds serviced{0.1};
constexpr std::uint32_t most_services = 20000;

struct arm_pipe
{
    strand work;
    arm_reader seen;
    std::shared_ptr<owned_arm> owned;
};

arm_pipe pipe(scheduler &loop)
{
    const strand work    = *loop.make_strand();
    const auto driven    = std::make_shared<scene_robot>(two_joint_arm(robot_ops{}));
    const auto published = std::make_shared<arm_publisher>();
    const auto control   = std::make_shared<robot_controller>(*driven, composing_motion(), composing_path(), task_trajectory_ops{}, composing_time_scaling(),
                                                              praxis::trajectory::trajectory_ops{}, praxis::rigid_motion::screw_ops{});

    return arm_pipe{work, published->reader(), std::make_shared<owned_arm>(work, work, driven, control, published)};
}

trajectory_recording_window::settings into(const std::filesystem::path &folder)
{
    trajectory_recording_window::settings requested;
    requested.active    = true;
    requested.directory = folder;

    return requested;
}

void drive_to(const arm_pipe &arm, double x, double y, double factor)
{
    praxis::transform pose = praxis::transform::Identity();
    pose(0, 3)             = x;
    pose(1, 3)             = y;

    command(arm.owned,
            [pose, factor](robot_controller &control, scene_robot &)
            {
                control.set_velocity_factor(factor);
                control.task_space_ptp(pose);
            });
}

bool played_out(scheduler &loop, const arm_reader &seen)
{
    for(std::uint32_t taken = 0; taken < most_services && seen.read()->executing; ++taken)
    {
        dictated += std::chrono::duration_cast<time_point::duration>(serviced);
        if(!loop.drain().has_value())
            return false;
    }

    return !seen.read()->executing;
}

// The composition frees a preset's arm at its strand's retirement acknowledgment and not at the teardown before it, so the acknowledgment is
// what a case drives: the last share of the arm goes with it, on the strand that owned it.
void retire_carrying_the_arm(scheduler &loop, arm_pipe &arm)
{
    REQUIRE(loop.retire_strand(arm.work, [held = std::move(arm.owned)]() mutable { held.reset(); }).has_value());
    REQUIRE(loop.drain().has_value());
}

std::vector<std::filesystem::path> written_into(const std::filesystem::path &folder, const std::string &tail)
{
    std::vector<std::filesystem::path> found;
    for(const std::filesystem::directory_entry &entry : std::filesystem::directory_iterator(folder))
        if(entry.path().filename().string().ends_with(tail))
            found.push_back(entry.path());

    return found;
}

struct sample_table
{
    bool whole;
    std::size_t rows;
    std::size_t columns;
};

// A row the writer did not finish is short of its columns or of its terminator, and a line read reports the same end either way, so the last byte answers the second.
sample_table shape_of(const std::filesystem::path &file)
{
    std::ifstream rows(file);
    std::string line;
    sample_table read{true, 0u, 0u};

    while(std::getline(rows, line))
    {
        const std::size_t columns = 1u + static_cast<std::size_t>(std::count(line.begin(), line.end(), ','));
        read.whole                = read.whole && (read.rows == 0u || columns == read.columns);
        read.columns              = columns;
        ++read.rows;
    }

    std::ifstream tail(file, std::ios::binary | std::ios::ate);
    tail.seekg(-1, std::ios::end);
    read.whole = read.whole && read.rows != 0u && tail.get() == '\n';

    return read;
}

}

TEST_CASE("a motion played out under an active recording leaves a trajectory beside its timestamps")
{
    const scratch_tree scratch("trajectory_recording");
    const std::filesystem::path folder = scratch.root() / "recordings";

    scheduler loop(inline_workers, dictating());
    const arm_pipe arm = pipe(loop);

    const trajectory_recording_window window("Recording", arm.seen, arm.owned, into(folder));
    REQUIRE(loop.drain().has_value());
    REQUIRE(arm.seen.read()->recording.active);

    drive_to(arm, 0.4, -0.3, 8.0);
    REQUIRE(loop.drain().has_value());
    REQUIRE(arm.seen.read()->executing);
    REQUIRE(played_out(loop, arm.seen));

    const sample_table played = shape_of(written_into(folder, "_trajectory.csv").front());
    const sample_table when   = shape_of(written_into(folder, "_timestamp.csv").front());

    CHECK(written_into(folder, ".csv").size() == 2u);
    CHECK(played.whole);
    CHECK(played.columns == 2u);
    CHECK(played.rows > 1u);
    CHECK(when.whole);
    CHECK(when.columns == 1u);
    CHECK(when.rows == played.rows);
}

// The files are written from the conclusion and from nowhere else: an arm freed without one leaves
// no file rather than a partial one, because a destructor writes nothing.
TEST_CASE("a recording whose arm is freed without a conclusion writes nothing from a destructor")
{
    const scratch_tree scratch("trajectory_recording");
    const std::filesystem::path folder = scratch.root() / "interrupted";

    scheduler loop(inline_workers, dictating());
    arm_pipe arm = pipe(loop);

    const trajectory_recording_window window("Recording", arm.seen, arm.owned, into(folder));
    REQUIRE(loop.drain().has_value());

    drive_to(arm, 0.4, -0.3, 0.05);
    REQUIRE(loop.drain().has_value());

    dictated += std::chrono::duration_cast<time_point::duration>(serviced);
    REQUIRE(loop.drain().has_value());
    REQUIRE(arm.seen.read()->executing);

    retire_carrying_the_arm(loop, arm);

    CHECK(written_into(folder, ".csv").empty());
}
