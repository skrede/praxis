#ifndef HPP_GUARD_PRAXIS_EXAMPLES_DEMO_WRITE_BACK_H
#define HPP_GUARD_PRAXIS_EXAMPLES_DEMO_WRITE_BACK_H

#include "demo_documents.h"

#include "praxis/scene/visualizer.h"

#include "praxis/config/writer.h"
#include "praxis/config/binding.h"
#include "praxis/config/document.h"
#include "praxis/config/declaration.h"
#include "praxis/config/configurable.h"

#include <span>
#include <memory>
#include <vector>
#include <cstdint>

namespace praxis::demo {

// What happens when a scenario whose values have moved is left. `ask` puts the question to a person;
// the other two are an answer a person asked to have stand for every later leaving.
enum class leaving_choice : std::uint8_t
{
    ask,
    keep,
    discard
};

void declare_leaving(config::declaration &shape);

// What is written back and where it goes: an edit made in a composition's window reaches this
// application's own copy of the document that composition was built from, and the remembered leaving
// answer reaches the preferences document, which is what this is constructed with beside it. Every one of these is
// reached only from the task the frames are drawn on, which is where a composer, a control and a
// release all run, so a write can never interleave with an edit of the values it is writing.
class write_back
{
public:
    write_back(config::binding bound, config::document carried, config::binding preferences, config::document preferred, documents mine);

    // The document the composition now on screen was built from, and the binding naming where it
    // came from. Every comparison and every save its windows make is measured against them. The
    // caller says this only once a composition has been answered, so a build that answers nothing
    // leaves it aimed at the document the windows on screen came from.
    void composing(config::binding bound, config::document carried);

    // Whether anything is left for a person to decide. A remembered answer is carried out here and
    // then answers no, so a later leaving neither asks nor loses anything.
    bool anything_to_decide(std::span<const config::configurable *const> shown);

    void resolve(scene::leaving_answer chosen, std::span<const config::configurable *const> shown);

    void save(std::span<const config::configurable *const> shown);

private:
    config::binding m_bound;
    config::binding m_preferences;
    config::document m_carried;
    leaving_choice m_remembered;
    documents m_mine;

    void write(std::vector<config::edit> changes);

    void remember(leaving_choice chosen);

    std::vector<config::edit> shown_now(std::span<const config::configurable *const> shown) const;
};

void install_write_back(scene::visualizer &view, const std::shared_ptr<write_back> &through);

}

#endif
