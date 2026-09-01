#include "demo_offered.h"
#include "demo_documents.h"
#include "demo_write_back.h"
#include "demo_configuration.h"

#include "praxis/scene/log_buffer.h"
#include "praxis/scene/visualizer.h"
#include "praxis/scene/preset_registry.h"

#include "praxis/config/store.h"
#include "praxis/config/binding.h"
#include "praxis/config/document.h"

#include "praxis/scheduler/scheduler.h"

#include <spdlog/spdlog.h>

#include <format>
#include <string>
#include <memory>
#include <vector>
#include <optional>
#include <exception>
#include <filesystem>
#include <system_error>

#ifndef PRAXIS_DEMO_CONFIGURATION
    #define PRAXIS_DEMO_CONFIGURATION "demonstration.xml"
#endif

#ifndef PRAXIS_PERSISTENT_DIRECTORY
    #define PRAXIS_PERSISTENT_DIRECTORY "."
#endif

namespace {

// Below the 128 a shell adds to a signal number, so a refusal does not read as a signalled end.
constexpr int renderer_unavailable = 3;

// Where the application keeps what it writes for itself. It is created here because a save against a
// directory that is not there fails, and nothing else in the run creates it.
std::filesystem::path persistent_directory(const std::filesystem::path &beside)
{
    const std::filesystem::path kept = praxis::config::resolve(PRAXIS_PERSISTENT_DIRECTORY, beside).resolved;

    std::error_code failed;
    std::filesystem::create_directories(kept, failed);
    if(failed)
        spdlog::error(std::format("The directory {} could not be created: {}", kept.string(), failed.message()));

    return kept;
}

praxis::config::binding preferences_at(const std::filesystem::path &beside)
{
    return praxis::config::binding{praxis::demo::preferences_keyspace(), praxis::config::resolve(persistent_directory(beside) / "praxis-preferences.xml", beside),
                                   praxis::config::expectation::partial};
}

// The documents the application ships with sit beside its own, and what it writes goes under the
// directory it keeps its state in.
praxis::demo::documents own_documents(const praxis::config::binding &bound, const std::filesystem::path &beside)
{
    return praxis::demo::documents(bound.at.resolved.parent_path(), persistent_directory(beside));
}

praxis::config::binding demonstration_at(const std::filesystem::path &beside)
{
    return praxis::config::binding{praxis::demo::demonstration_keyspace(), praxis::config::resolve(PRAXIS_DEMO_CONFIGURATION, beside), praxis::config::expectation::complete};
}

// Answers where the window was left, which is the one thing the run leaves behind.
praxis::scene::visualizer::geometry run_until_closed(const std::shared_ptr<praxis::scene::preset_registry> &registry, const std::shared_ptr<praxis::demo::write_back> &writing,
                                                     const std::shared_ptr<praxis::scene::log_buffer> &messages, const std::vector<std::string> &machines,
                                                     const praxis::scene::visualizer::geometry &window, const std::filesystem::path &root)
{
    praxis::scheduler::scheduler loop(praxis::scheduler::default_workers());
    praxis::scene::visualizer view(registry, loop, {.view = praxis::scene::visualizer::projection::perspective, .messages = messages, .window = window, .root = root});
    praxis::demo::install_write_back(view, writing);
    if(!machines.empty())
        view.load_preset(machines.front());

    const praxis::scheduler::task_handle frame = view.executor().every(praxis::scheduler::every_step, praxis::scheduler::overrun::drop,
                                                                       [&loop, &view, closing = false](praxis::scheduler::step_delta) mutable
                                                                       {
                                                                           if(closing || view.render_once())
                                                                               return;

                                                                           closing = true;
                                                                           view.release_preset([&loop] { loop.stop(); });
                                                                       });

    loop.run();

    return view.window_geometry();
}

// The renderer throws out of the canvas construction the visualizer's constructor reaches.
std::optional<praxis::scene::visualizer::geometry> run_and_report(const std::shared_ptr<praxis::scene::preset_registry> &registry,
                                                                  const std::shared_ptr<praxis::demo::write_back> &writing, const std::shared_ptr<praxis::scene::log_buffer> &messages,
                                                                  const std::vector<std::string> &machines, const praxis::scene::visualizer::geometry &window,
                                                                  const std::filesystem::path &root)
{
    try
    {
        return run_until_closed(registry, writing, messages, machines, window, root);
    }
    catch(const std::exception &failed)
    {
        spdlog::error(std::format("The renderer could not be started, so there is nothing to show: {}", failed.what()));
    }
    catch(...)
    {
        spdlog::error("The renderer could not be started, so there is nothing to show");
    }

    return std::nullopt;
}

}

int main(int, char **argv)
{
    const std::filesystem::path beside = std::filesystem::weakly_canonical(std::filesystem::path(argv[0])).parent_path();

    const auto messages = std::make_shared<praxis::scene::log_buffer>(praxis::scene::default_log_capacity);
    praxis::scene::install_log_sink(messages);

    const praxis::config::binding prefs     = preferences_at(beside);
    const praxis::config::outcome preferred = praxis::config::load_or_defaults(prefs);
    praxis::scene::set_reporting_level(praxis::demo::preferred_level(preferred.values));

    const praxis::config::binding bound      = demonstration_at(beside);
    const praxis::config::outcome configured = praxis::config::load_or_defaults(bound);

    const praxis::demo::documents mine      = own_documents(bound, beside);
    const auto writing                      = std::make_shared<praxis::demo::write_back>(bound, configured.values, prefs, preferred.values, mine);
    const auto registry                     = std::make_shared<praxis::scene::preset_registry>();
    const std::vector<std::string> machines = praxis::demo::register_offered(registry, mine, configured.values, beside / "urdf", writing);

    const std::optional<praxis::scene::visualizer::geometry> left =
            run_and_report(registry, writing, messages, machines, praxis::demo::preferred_geometry(preferred.values), prefs.at.resolved.parent_path());
    if(!left)
        return renderer_unavailable;

    static_cast<void>(praxis::config::save(prefs, praxis::demo::preferences_edits(*left, praxis::scene::reporting_level())));

    return 0;
}
