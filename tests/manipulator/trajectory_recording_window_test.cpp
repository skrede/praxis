#include "fixtures.h"

#include "held_home.h"
#include "captured_log.h"
#include "two_link_arm.h"

#include "praxis/manipulator/arm_state.h"
#include "praxis/manipulator/compose_arm.h"
#include "praxis/manipulator/capabilities.h"
#include "praxis/manipulator/robot_controller.h"
#include "praxis/manipulator/trajectory_recording_window.h"

#include "praxis/scene/preset.h"
#include "praxis/scene/log_buffer.h"
#include "praxis/scene/composition.h"
#include "praxis/scene/preset_site.h"

#include "praxis/scheduler/scheduler.h"

#include "praxis/trajectory/capabilities.h"

#include "praxis/rigid_motion/capabilities.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <meios/model.h>

#include <threepp/scenes/Scene.hpp>

#include <spdlog/spdlog.h>

#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <optional>
#include <utility>
#include <filesystem>

using namespace praxis::tests;
using namespace praxis::fixture;
using namespace praxis::manipulator;
using namespace praxis::scheduler;

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

struct window_outcome
{
    recording_parameters applied;
    trajectory_recording_window::settings shown;
    std::string diagnosis;
};

struct arm_pipe
{
    std::shared_ptr<owned_arm> owned;
    arm_reader seen;
};

std::shared_ptr<robot_controller> controlling(scene_robot &driven, const std::filesystem::path &root)
{
    return std::make_shared<robot_controller>(driven, composing_motion(), composing_path(), task_trajectory_ops{}, composing_time_scaling(), praxis::trajectory::trajectory_ops{},
                                              praxis::rigid_motion::screw_ops{}, std::function<void()>{}, root);
}

arm_pipe pipe(scheduler &loop, const std::filesystem::path &root)
{
    const strand work    = *loop.make_strand();
    const auto driven    = std::make_shared<scene_robot>(two_joint_arm(robot_ops{}));
    const auto control   = controlling(*driven, root);
    const auto published = std::make_shared<arm_publisher>();

    return arm_pipe{std::make_shared<owned_arm>(work, work, driven, control, published), published->reader()};
}

// The window is built through the form that takes the settings, so the folder under test reaches the
// arm without a windowing context and render is never called. The window owns no preparation of its
// own: what it opens on is whatever the arm's own state accepted, and that arrives as a publication.
window_outcome opening_on(const std::filesystem::path &folder, const std::filesystem::path &root)
{
    scheduler loop(inline_workers, dictating());
    const arm_pipe arm = pipe(loop, root);

    trajectory_recording_window::settings requested;
    requested.active    = true;
    requested.directory = folder;

    captured_log captured;

    const trajectory_recording_window window("Recording", arm.seen, arm.owned, requested);
    REQUIRE(loop.drain().has_value());

    return window_outcome{arm.seen.read()->recording, window.state(), captured.text()};
}

std::shared_ptr<praxis::scene::log_buffer> ring_on_the_current_logger()
{
    auto messages = std::make_shared<praxis::scene::log_buffer>(praxis::scene::default_log_capacity);
    praxis::scene::install_log_sink(messages);

    return messages;
}

std::vector<std::string> texts_at(const std::vector<praxis::scene::log_entry> &drained, praxis::scene::severity level)
{
    std::vector<std::string> out;
    for(const praxis::scene::log_entry &entry : drained)
        if(entry.level == level)
            out.push_back(entry.text);

    return out;
}

// The ring goes onto the capture's logger and leaves with it, so what is drained is what the posted
// operation reported while it ran, and the level is the one an operator who changed nothing has.
std::vector<praxis::scene::log_entry> reported_while_opening_on(const std::filesystem::path &folder, const std::filesystem::path &root)
{
    scheduler loop(inline_workers, dictating());
    const arm_pipe arm = pipe(loop, root);

    trajectory_recording_window::settings requested;
    requested.active    = true;
    requested.directory = folder;

    const captured_log terminal;
    const std::shared_ptr<praxis::scene::log_buffer> messages = ring_on_the_current_logger();
    praxis::scene::set_reporting_level(praxis::scene::severity::info);

    const trajectory_recording_window window("Recording", arm.seen, arm.owned, requested);
    REQUIRE(loop.drain().has_value());

    return messages->drain();
}

// One service covers every period the raised reading spans, so the advance may be coarser than the
// period the playback is registered at.
constexpr seconds serviced{0.05};
constexpr std::uint32_t most_services = 20000;

// The working directory is where an unresolved relative folder lands, so a case asserting that it did
// not land there is only meaningful over a directory nothing else writes into.
class started_in
{
public:
    explicit started_in(const std::filesystem::path &where)
            : m_before(std::filesystem::current_path())
    {
        std::filesystem::current_path(where);
    }

    started_in(const started_in &)            = delete;
    started_in &operator=(const started_in &) = delete;

    ~started_in()
    {
        std::error_code ignored;
        std::filesystem::current_path(m_before, ignored);
    }

private:
    std::filesystem::path m_before;
};

// A scene is created headlessly and a renderer robot needs no graphics context, so no display is
// involved. The site is what carries the root the recording resolves against.
struct stage
{
    stage(scheduler &loop, const std::filesystem::path &root)
            : scene(threepp::Scene::create())
            , site{*scene,
                   loop.main_strand(),
                   *loop.make_strand(),
                   [] {},
                   [](const std::shared_ptr<praxis::scene::imgui_window> &) {},
                   [](const std::shared_ptr<praxis::scene::imgui_window> &) {},
                   root}
    {
    }

    std::shared_ptr<threepp::Scene> scene;
    praxis::scene::preset_site site;
};

struct composed_arm
{
    arm_reader seen;
    std::weak_ptr<owned_arm> arm;
    std::shared_ptr<praxis::scene::preset> preset;
};

joint_vector one_joint(double at)
{
    joint_vector q(1);
    q << at;

    return q;
}

trajectory_recording_window::settings into(const std::filesystem::path &folder)
{
    trajectory_recording_window::settings requested;
    requested.active    = true;
    requested.directory = folder;

    return requested;
}

// The window is the composer's one contribution, so the folder under test reaches the arm over the
// same route the demonstration uses and the preset that owns the retirement is a real one.
composed_arm composing(stage &built, const trajectory_recording_window::settings &requested)
{
    std::optional<arm_reader> caught;
    std::weak_ptr<owned_arm> reached;
    const arm_window_composer recording_window = [&caught, &reached, &requested](const arm_window_inputs &offered)
    {
        caught.emplace(offered.seen);
        reached = offered.arm;

        return std::vector<std::shared_ptr<praxis::scene::imgui_window>>{std::make_shared<trajectory_recording_window>("Recording", offered.seen, offered.arm, requested)};
    };

    std::shared_ptr<praxis::scene::preset> composed = compose_arm(well_formed_arm(), built.site, attached_models{}, baseline(), praxis::trajectory::baseline(),
                                                                  praxis::rigid_motion::baseline(), one_joint(0.0), recording_window);
    REQUIRE(composed != nullptr);
    REQUIRE(caught.has_value());
    REQUIRE(composed->initialize().has_value());

    return composed_arm{*caught, reached, std::move(composed)};
}

praxis::scene::window_route ignoring()
{
    return [](const std::shared_ptr<praxis::scene::imgui_window> &) {};
}

// The composer's captures are filled while the composition builds the preset, so a case loading
// through a real composition reaches the same arm the hand-made site above reaches.
praxis::scene::preset_registry::factory recording_preset(std::optional<arm_reader> &caught, std::weak_ptr<owned_arm> &reached, const trajectory_recording_window::settings &requested)
{
    return [&caught, &reached, &requested](const praxis::scene::preset_site &site)
    {
        const arm_window_composer recording_window = [&caught, &reached, &requested](const arm_window_inputs &offered)
        {
            caught.emplace(offered.seen);
            reached = offered.arm;

            return std::vector<std::shared_ptr<praxis::scene::imgui_window>>{std::make_shared<trajectory_recording_window>("Recording", offered.seen, offered.arm, requested)};
        };

        return compose_arm(well_formed_arm(), site, attached_models{}, baseline(), praxis::trajectory::baseline(), praxis::rigid_motion::baseline(), one_joint(0.0), recording_window);
    };
}

void advance(scheduler &loop, seconds by)
{
    dictated += std::chrono::duration_cast<time_point::duration>(by);
    REQUIRE(loop.drain().has_value());
}

void drive(scheduler &loop, const std::weak_ptr<owned_arm> &arm, double to, double factor)
{
    const std::vector<joint_vector> waypoints{one_joint(to)};
    command(arm,
            [waypoints, factor](robot_controller &control, scene_robot &)
            {
                control.set_velocity_factor(factor);
                control.joint_space_trajectory(waypoints);
            });
    REQUIRE(loop.drain().has_value());
}

bool played_out(scheduler &loop, const arm_reader &seen)
{
    for(std::uint32_t taken = 0; taken < most_services && seen.read()->executing; ++taken)
        advance(loop, serviced);

    return !seen.read()->executing;
}

// The acknowledgment of the strand's retirement is what a composition frees its arm at, so that is
// what a case interrupting a recording drives rather than the teardown before it.
void retire(scheduler &loop, const std::shared_ptr<praxis::scene::preset> &composed)
{
    composed->tear_down();
    REQUIRE(loop.retire_strand(composed->work, std::move(composed->release_cb)).has_value());
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

std::vector<std::uint64_t> stamps_in(const std::filesystem::path &file)
{
    std::vector<std::uint64_t> read;
    std::ifstream rows(file);
    std::uint64_t stamp = 0u;

    while(rows >> stamp)
        read.push_back(stamp);

    return read;
}

std::size_t rows_in(const std::filesystem::path &file)
{
    std::ifstream rows(file);
    std::string line;
    std::size_t counted = 0u;

    while(std::getline(rows, line))
        ++counted;

    return counted;
}

// Every sample is stamped one playback period after the one before it, so a stamp column that is the
// first stamp at an even step is a column carrying every row taken up to the interruption, with none
// of them lost between the first and the last.
bool evenly_stepped(const std::vector<std::uint64_t> &stamps)
{
    if(stamps.empty() || stamps.front() == 0u)
        return false;

    for(std::size_t at = 0u; at < stamps.size(); ++at)
        if(stamps[at] != (at + 1u) * stamps.front())
            return false;

    return true;
}

}

TEST_CASE("the folder a window opens on is prepared by the arm it hands it to")
{
    const scratch_tree scratch("recording_window");
    const std::filesystem::path folder = scratch.root() / "recordings" / "run";

    const window_outcome outcome = opening_on(folder, scratch.root());

    CHECK(outcome.applied.active);
    CHECK(outcome.applied.directory == std::filesystem::weakly_canonical(folder));
    CHECK(outcome.shown.active);
    CHECK(std::filesystem::is_directory(folder));
}

TEST_CASE("a window opened on a folder that cannot be prepared configures no recording")
{
    const scratch_tree scratch("recording_window");
    const std::filesystem::path occupied = scratch.root() / "occupied";
    std::ofstream(occupied) << "a regular file, not a folder";
    const std::filesystem::path folder = occupied / "beneath";

    const window_outcome outcome = opening_on(folder, scratch.root());

    CHECK_FALSE(outcome.applied.active);
    CHECK(outcome.applied.directory.empty());
    CHECK_THAT(outcome.diagnosis, Catch::Matchers::ContainsSubstring(folder.string()));
    CHECK_FALSE(std::filesystem::exists(folder));
}

// Nothing can answer a posted command with the call, so a refusal never reaches back into the
// control that sent it: the window keeps what was asked for, and the state a reader believes is the
// published one the checkbox adopts at the top of the next frame.
TEST_CASE("a refused folder leaves the window's own state untouched and the publication switched off")
{
    const scratch_tree scratch("recording_window");
    const std::filesystem::path occupied = scratch.root() / "occupied";
    std::ofstream(occupied) << "a regular file, not a folder";

    const window_outcome outcome = opening_on(occupied / "beneath", scratch.root());

    CHECK(outcome.shown.active);
    CHECK_FALSE(outcome.applied.active);
}

// The composition path: a window built without settings reads them off the publication, which names
// no folder until one is given. That is what happens on the demonstration's first frame, and it has
// nothing to report.
TEST_CASE("a window built from an arm that names no folder reports nothing")
{
    scheduler loop(inline_workers, dictating());
    const arm_pipe arm = pipe(loop, std::filesystem::path());

    captured_log captured;

    const trajectory_recording_window window("Recording", arm.seen, arm.owned);
    REQUIRE(loop.drain().has_value());

    CHECK(captured.text().empty());
    CHECK(arm.seen.read()->recording.directory.empty());
}

// What was typed and where the files land are different claims, and only the resolved form answers
// the second one.
TEST_CASE("an applied folder is reported as it resolved and not as it was typed")
{
    const scratch_tree scratch("recording_window");
    const std::filesystem::path typed = scratch.root() / "recordings" / ".." / "recordings" / "run";

    const std::vector<praxis::scene::log_entry> drained = reported_while_opening_on(typed, scratch.root());
    const std::filesystem::path landed                  = std::filesystem::weakly_canonical(scratch.root() / "recordings" / "run");
    const std::vector<std::string> applied              = texts_at(drained, praxis::scene::severity::info);

    REQUIRE(landed != typed);
    REQUIRE(applied.size() == 1);
    CHECK_THAT(applied.front(), Catch::Matchers::ContainsSubstring(landed.string()));
    CHECK_THAT(applied.front(), !Catch::Matchers::ContainsSubstring(typed.string()));
    CHECK(texts_at(drained, praxis::scene::severity::error).empty());
    CHECK(std::filesystem::is_directory(landed));
}

// A folder beneath an existing regular file cannot be created on any platform the library builds
// for, so the refusal is the filesystem's and not a contrivance of the case.
TEST_CASE("a folder that cannot be prepared is reported as a refusal and nothing claims it was applied")
{
    const scratch_tree scratch("recording_window");
    const std::filesystem::path occupied = scratch.root() / "occupied";
    std::ofstream(occupied) << "a regular file, not a folder";
    const std::filesystem::path folder = occupied / "beneath";

    const std::vector<praxis::scene::log_entry> drained = reported_while_opening_on(folder, scratch.root());
    const std::vector<std::string> refused              = texts_at(drained, praxis::scene::severity::error);

    CHECK(texts_at(drained, praxis::scene::severity::info).empty());
    REQUIRE(refused.size() == 1);
    CHECK_THAT(refused.front(), Catch::Matchers::ContainsSubstring(folder.string()));
    CHECK_FALSE(std::filesystem::exists(folder));
}

// The ring and the terminal hang off one logger, which consults its level once before it reaches any
// sink, so a level that drops a message drops it from both surfaces rather than one.
TEST_CASE("one reporting level governs the terminal and the message ring together")
{
    const captured_log terminal;
    const std::shared_ptr<praxis::scene::log_buffer> messages = ring_on_the_current_logger();

    SECTION("warnings and above drop an informational message from both surfaces")
    {
        praxis::scene::set_reporting_level(praxis::scene::severity::warning);
        spdlog::info("praxis: a folder was applied");
        spdlog::error("praxis: a folder was refused");

        const std::vector<praxis::scene::log_entry> drained = messages->drain();

        REQUIRE(drained.size() == 1);
        CHECK(drained.front().level == praxis::scene::severity::error);
        CHECK_THAT(terminal.text(), !Catch::Matchers::ContainsSubstring("was applied"));
        CHECK_THAT(terminal.text(), Catch::Matchers::ContainsSubstring("was refused"));
    }

    SECTION("the lowest level carries an informational message to both surfaces")
    {
        praxis::scene::set_reporting_level(praxis::scene::severity::debug);
        spdlog::info("praxis: a folder was applied");

        const std::vector<praxis::scene::log_entry> drained = messages->drain();

        REQUIRE(drained.size() == 1);
        CHECK(drained.front().level == praxis::scene::severity::info);
        CHECK_THAT(terminal.text(), Catch::Matchers::ContainsSubstring("was applied"));
    }
}

// A folder that is not absolute is resolved against the root the application chose, which is what
// keeps it off whatever directory the process happened to be started in.
TEST_CASE("a relative recording folder is created under the chosen root and not where the process is running")
{
    const scratch_tree chosen("recording_window");
    const scratch_tree elsewhere("recording_window_working_directory");
    const started_in from(elsewhere.root());
    const std::filesystem::path landed = chosen.root() / "recordings" / "run";

    const std::vector<std::string> applied = texts_at(reported_while_opening_on("recordings/run", chosen.root()), praxis::scene::severity::info);

    REQUIRE(applied.size() == 1);
    CHECK_THAT(applied.front(), Catch::Matchers::ContainsSubstring(landed.string()));
    CHECK(std::filesystem::is_directory(landed));
    CHECK(std::filesystem::is_empty(elsewhere.root()));
}

// A leading tilde names the home directory. Taken as an ordinary component it would name a folder
// spelled with that character, and the emptiness of the chosen root is what says it was not.
TEST_CASE("a recording folder written with a leading tilde is expanded against the home directory")
{
    const scratch_tree chosen("recording_window");
    const scratch_tree dwelling("recording_window_home");
    const scratch_tree elsewhere("recording_window_working_directory");
    const started_in from(elsewhere.root());
    const held_home during{dwelling.root().string()};
    const std::filesystem::path landed = dwelling.root() / "recordings";

    const std::vector<std::string> applied = texts_at(reported_while_opening_on("~/recordings", chosen.root()), praxis::scene::severity::info);

    REQUIRE(applied.size() == 1);
    CHECK_THAT(applied.front(), Catch::Matchers::ContainsSubstring(landed.string()));
    CHECK(std::filesystem::is_directory(landed));
    CHECK(std::filesystem::is_empty(chosen.root()));
    CHECK(std::filesystem::is_empty(elsewhere.root()));
}

TEST_CASE("a recording that names no folder writes under the chosen root and is reported as doing so")
{
    const scratch_tree chosen("recording_window");

    scheduler loop(inline_workers, dictating());
    stage built(loop, chosen.root());

    const captured_log terminal;
    const std::shared_ptr<praxis::scene::log_buffer> messages = ring_on_the_current_logger();
    praxis::scene::set_reporting_level(praxis::scene::severity::info);

    const composed_arm composed = composing(built, into(std::filesystem::path()));
    REQUIRE(loop.drain().has_value());

    const std::vector<std::string> applied = texts_at(messages->drain(), praxis::scene::severity::info);
    REQUIRE(applied.size() == 1);
    CHECK_THAT(applied.front(), Catch::Matchers::ContainsSubstring(chosen.root().string()));
    CHECK_THAT(applied.front(), !Catch::Matchers::ContainsSubstring("where the application was started"));
    CHECK(composed.seen.read()->recording.directory == std::filesystem::weakly_canonical(chosen.root()));

    drive(loop, composed.arm, 0.6, 4.0);
    REQUIRE(played_out(loop, composed.seen));

    CHECK(written_into(chosen.root(), ".csv").size() == 2u);
}

// The samples are gathered in memory and written from the conclusion, so what a retirement leaves on
// disk is the whole question: either the rows the recording had, or nothing at all.
TEST_CASE("a recording interrupted by the composition's retirement holds the rows it had taken")
{
    const scratch_tree chosen("recording_window");
    const std::filesystem::path folder = chosen.root() / "interrupted";

    scheduler loop(inline_workers, dictating());
    stage built(loop, chosen.root());

    const captured_log terminal;
    const composed_arm composed = composing(built, into("interrupted"));
    REQUIRE(loop.drain().has_value());
    REQUIRE(composed.seen.read()->recording.directory == std::filesystem::weakly_canonical(folder));

    drive(loop, composed.arm, 0.6, 0.01);
    advance(loop, serviced);
    REQUIRE(composed.seen.read()->executing);

    retire(loop, composed.preset);

    REQUIRE(written_into(folder, ".csv").size() == 2u);

    const std::vector<std::uint64_t> stamps = stamps_in(written_into(folder, "_timestamp.csv").front());

    REQUIRE(stamps.size() > 1u);
    CHECK(evenly_stepped(stamps));
    CHECK(rows_in(written_into(folder, "_trajectory.csv").front()) == stamps.size());
}

// What interrupts the recording here is the run ending rather than a retirement: the loop is stopped
// while the composition still holds the preset, so the retirement it asks for is refused and no
// acknowledgment is ever run.
TEST_CASE("a recording interrupted by the run ending holds the rows it had taken")
{
    const scratch_tree chosen("recording_window");
    const std::filesystem::path folder = chosen.root() / "ended";

    scheduler loop(inline_workers, dictating());
    const std::shared_ptr<threepp::Scene> target          = threepp::Scene::create();
    const trajectory_recording_window::settings requested = into("ended");

    const captured_log terminal;
    std::optional<arm_reader> seen;
    std::weak_ptr<owned_arm> arm;

    {
        praxis::scene::composition held(*target, loop, chosen.root());
        held.windows_through(ignoring(), ignoring());

        REQUIRE(held.load(recording_preset(seen, arm, requested)).has_value());
        REQUIRE(loop.drain().has_value());
        REQUIRE(seen.has_value());
        REQUIRE(seen->read()->recording.directory == std::filesystem::weakly_canonical(folder));

        drive(loop, arm, 0.6, 0.01);
        advance(loop, serviced);
        REQUIRE(seen->read()->executing);

        loop.stop();
    }

    REQUIRE(written_into(folder, ".csv").size() == 2u);

    const std::vector<std::uint64_t> stamps = stamps_in(written_into(folder, "_timestamp.csv").front());

    REQUIRE(stamps.size() > 1u);
    CHECK(evenly_stepped(stamps));
    CHECK(rows_in(written_into(folder, "_trajectory.csv").front()) == stamps.size());
}

// The release is taken while the loop is still servicing, so the conclusion is queued on the
// preset's own strand rather than performed by whoever asked for it, and the arm's ownership ends
// there too. What tells this path from the one above is the drain: here the rows need one to appear
// at all, there they are on disk without one, because nothing will ever service one. It does not
// reach the interleaving inside the acknowledgment, which no case can observe in a callable it did
// not author.
TEST_CASE("a release taken while the loop runs concludes the recording at the acknowledgment and not before")
{
    const scratch_tree chosen("recording_window");
    const std::filesystem::path folder = chosen.root() / "ordered";

    scheduler loop(inline_workers, dictating());
    const std::shared_ptr<threepp::Scene> target          = threepp::Scene::create();
    const trajectory_recording_window::settings requested = into("ordered");

    const captured_log terminal;
    std::optional<arm_reader> seen;
    std::weak_ptr<owned_arm> arm;

    praxis::scene::composition held(*target, loop, chosen.root());
    held.windows_through(ignoring(), ignoring());

    REQUIRE(held.load(recording_preset(seen, arm, requested)).has_value());
    REQUIRE(loop.drain().has_value());
    REQUIRE(seen.has_value());
    REQUIRE(seen->read()->recording.directory == std::filesystem::weakly_canonical(folder));

    drive(loop, arm, 0.6, 0.01);
    advance(loop, serviced);
    REQUIRE(seen->read()->executing);

    held.release();

    REQUIRE(written_into(folder, ".csv").empty());
    REQUIRE_FALSE(arm.expired());

    REQUIRE(loop.drain().has_value());

    REQUIRE(written_into(folder, ".csv").size() == 2u);
    REQUIRE(arm.expired());
}

// Nothing services the preset's strand between the release the composition's own end takes and the
// scheduler's death, so the acknowledgment queued there is discarded rather than run. The rows reach
// disk at that discarding, which is the only end this path has, and the middle assertion is what
// tells it from a conclusion taken eagerly at the release.
TEST_CASE("a recording abandoned with the loop never serviced again holds the rows it had taken")
{
    const scratch_tree chosen("recording_window");
    const std::filesystem::path folder = chosen.root() / "abandoned";

    const captured_log terminal;
    std::optional<arm_reader> seen;
    std::weak_ptr<owned_arm> arm;

    {
        scheduler loop(inline_workers, dictating());
        const std::shared_ptr<threepp::Scene> target          = threepp::Scene::create();
        const trajectory_recording_window::settings requested = into("abandoned");

        {
            praxis::scene::composition held(*target, loop, chosen.root());
            held.windows_through(ignoring(), ignoring());

            REQUIRE(held.load(recording_preset(seen, arm, requested)).has_value());
            REQUIRE(loop.drain().has_value());
            REQUIRE(seen.has_value());
            REQUIRE(seen->read()->recording.directory == std::filesystem::weakly_canonical(folder));

            drive(loop, arm, 0.6, 0.01);
            advance(loop, serviced);
            REQUIRE(seen->read()->executing);
        }

        REQUIRE(written_into(folder, ".csv").empty());
    }

    REQUIRE(written_into(folder, ".csv").size() == 2u);

    const std::vector<std::uint64_t> stamps = stamps_in(written_into(folder, "_timestamp.csv").front());

    REQUIRE(stamps.size() > 1u);
    CHECK(evenly_stepped(stamps));
    CHECK(rows_in(written_into(folder, "_trajectory.csv").front()) == stamps.size());
    CHECK(arm.expired());
}
