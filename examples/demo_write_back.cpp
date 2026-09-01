#include "demo_write_back.h"

#include "praxis/config/error.h"
#include "praxis/config/store.h"

#include <spdlog/spdlog.h>

#include <span>
#include <array>
#include <format>
#include <memory>
#include <string>
#include <vector>
#include <cstddef>
#include <utility>

namespace praxis::demo {

namespace {

constexpr const char *leaving_key = "editing/on_leaving";

// In the enumeration's own order, which is what reading one back as an index and casting relies on.
constexpr std::array<const char *, 3> leaving_choices{"ask", "keep", "discard"};

leaving_choice remembered_leaving(const config::document &values)
{
    const expected<std::string, config::error> read = values.text(leaving_key);
    for(std::size_t option = 0u; read && option < leaving_choices.size(); ++option)
        if(read.value() == leaving_choices[option])
            return static_cast<leaving_choice>(option);

    return leaving_choice::ask;
}

config::edit leaving_edit(leaving_choice chosen)
{
    return config::edit{leaving_key, leaving_choices[static_cast<std::size_t>(chosen)]};
}

// The module's own save names the resolved path it wrote to, verifies the write by reading it back
// and refuses without touching the file when that fails, so nothing here names a path or checks one.
bool written_into(const config::binding &into, std::span<const config::edit> changes)
{
    const expected<void, config::error> written = config::save(into, changes);
    if(!written)
        spdlog::error(std::format("The values were not written: {}", written.error().message));

    return written.has_value();
}

}

void declare_leaving(config::declaration &shape)
{
    shape.group("editing");
    shape.choice(leaving_key, std::vector<std::string>(leaving_choices.begin(), leaving_choices.end()), leaving_choices[0]);
}

write_back::write_back(config::binding bound, config::document carried, config::binding preferences, config::document preferred, documents mine)
        : m_bound(std::move(bound))
        , m_preferences(std::move(preferences))
        , m_carried(std::move(carried))
        , m_remembered(remembered_leaving(preferred))
        , m_mine(std::move(mine))
{
}

void write_back::composing(config::binding bound, config::document carried)
{
    m_bound   = std::move(bound);
    m_carried = std::move(carried);
}

bool write_back::anything_to_decide(std::span<const config::configurable *const> shown)
{
    const bool moved = config::anything_unsaved(shown, m_carried);
    if(m_remembered == leaving_choice::ask)
        return moved;

    if(moved && m_remembered == leaving_choice::keep)
        save(shown);

    return false;
}

void write_back::resolve(scene::leaving_answer chosen, std::span<const config::configurable *const> shown)
{
    if(chosen.keep)
        write(shown_now(shown));

    if(chosen.remember)
        remember(chosen.keep ? leaving_choice::keep : leaving_choice::discard);
}

void write_back::save(std::span<const config::configurable *const> shown)
{
    write(shown_now(shown));
}

void write_back::write(std::vector<config::edit> changes)
{
    if(changes.empty())
    {
        spdlog::info(std::format("Nothing the composition shows is unsaved, so the configuration at {} was left as it is", m_bound.at.resolved.string()));
        return;
    }

    // A document that ships with the repository is read and never written, so a save lands in this
    // application's own copy of it under the same name, made from that document where it was absent.
    const config::binding into{m_bound.shape, m_mine.writing(m_bound.at.given), m_bound.carries};
    if(!written_into(into, changes))
        return;

    // Every comparison after this one is against what the copy now carries.
    expected<config::document, config::error> reread = config::load(into.shape, into.at);
    if(reread)
        m_carried = std::move(reread).value();
}

void write_back::remember(leaving_choice chosen)
{
    const std::array<config::edit, 1> changes{leaving_edit(chosen)};
    if(written_into(m_preferences, changes))
        m_remembered = chosen;
}

std::vector<config::edit> write_back::shown_now(std::span<const config::configurable *const> shown) const
{
    return config::shown_edits(shown, m_carried);
}

void install_write_back(scene::visualizer &view, const std::shared_ptr<write_back> &through)
{
    view.saving_through([&view, through] { through->save(view.configured()); });
    view.asking_before_release([&view, through] { return through->anything_to_decide(view.configured()); },
                               [&view, through](scene::leaving_answer chosen) { through->resolve(chosen, view.configured()); });
}

}
