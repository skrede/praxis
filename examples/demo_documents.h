#ifndef HPP_GUARD_PRAXIS_EXAMPLES_DEMO_DOCUMENTS_H
#define HPP_GUARD_PRAXIS_EXAMPLES_DEMO_DOCUMENTS_H

#include "praxis/config/store.h"

#include <filesystem>

namespace praxis::demo {

// The two places a named document can be. A seed ships with the repository and is never written to:
// it is read, and it is copied. A copy is the application's own, under the directory it keeps its
// state in and under the same name as the seed it came from, and every save lands there. A read
// answers the copy where there is one and the seed otherwise, so a seed edited in place is still
// what a scenario opens until that document has been saved.
class documents
{
public:
    documents(std::filesystem::path seeds, std::filesystem::path state);

    const std::filesystem::path &seeds() const;

    config::location reading(const std::filesystem::path &named) const;

    // Always the copy, with the directory it lands in made and the seed reproduced into it where
    // that copy is not there yet. A name carrying a directory part keeps it, beneath the state
    // directory. A name no seed carries is answered without a file being made, and the save that
    // follows is what creates it.
    config::location writing(const std::filesystem::path &named) const;

private:
    std::filesystem::path m_seeds;
    std::filesystem::path m_state;
};

}

#endif
