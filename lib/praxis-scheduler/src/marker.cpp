#include "marker.h"

namespace praxis::scheduler {

namespace {

thread_local running_marker *installed = nullptr;

}

running_marker::running_marker(const pool &owner, strand_id on, std::uint64_t task)
        : m_id(on)
        , m_owner(owner)
        , m_task(task)
        , m_previous(installed)
{
    installed = this;
}

running_marker::~running_marker()
{
    installed = m_previous;
}

bool running_marker::any(const pool &owner)
{
    for(const running_marker *node = installed; node != nullptr; node = node->m_previous)
        if(&node->m_owner == &owner)
            return true;

    return false;
}

bool running_marker::active(const pool &owner, strand_id on)
{
    for(const running_marker *node = installed; node != nullptr; node = node->m_previous)
        if(&node->m_owner == &owner && node->m_id == on)
            return true;

    return false;
}

bool running_marker::running_task(const pool &owner, std::uint64_t task)
{
    for(const running_marker *node = installed; node != nullptr; node = node->m_previous)
        if(&node->m_owner == &owner && node->m_task == task)
            return true;

    return false;
}

}
