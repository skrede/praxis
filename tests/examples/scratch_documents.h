#ifndef HPP_GUARD_PRAXIS_TESTS_EXAMPLES_SCRATCH_DOCUMENTS_H
#define HPP_GUARD_PRAXIS_TESTS_EXAMPLES_SCRATCH_DOCUMENTS_H

#include <string>
#include <cstddef>
#include <fstream>
#include <iterator>
#include <filesystem>
#include <string_view>
#include <system_error>

namespace scratch_documents {

// A seed carrying a comment, so a case comparing a copy against it also asserts that what the author
// wrote around the values survived the copy.
inline constexpr std::string_view authored = "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
                                             "<!-- the line an author wrote -->\n"
                                             "<machine>\n"
                                             "    <view label=\"as it ships\"/>\n"
                                             "</machine>\n";

// A copy standing where a seed would be reproduced, carrying a value the seed does not.
inline constexpr std::string_view already_there = "<machine>\n    <view label=\"as it was left\"/>\n</machine>\n";

// Each case builds both directories under a root of its own and removes that root when it ends, so
// no case reads what another one wrote.
class scratch
{
public:
    explicit scratch(std::string_view named)
            : m_root(std::filesystem::temp_directory_path() / ("praxis-demo-documents-" + std::string(named)))
    {
        std::filesystem::remove_all(m_root);
        std::filesystem::create_directories(seeds());
    }

    scratch(const scratch &)            = delete;
    scratch &operator=(const scratch &) = delete;

    ~scratch()
    {
        std::error_code ignored;
        std::filesystem::remove_all(m_root, ignored);
    }

    std::filesystem::path seeds() const
    {
        return m_root / "seeds";
    }

    std::filesystem::path state() const
    {
        return m_root / "state";
    }

private:
    std::filesystem::path m_root;
};

inline void author(const std::filesystem::path &at, std::string_view text)
{
    std::filesystem::create_directories(at.parent_path());
    std::ofstream out(at, std::ios::binary);
    out << text;
}

inline std::string bytes_of(const std::filesystem::path &at)
{
    std::ifstream in(at, std::ios::binary);

    return std::string(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
}

inline std::size_t files_in(const std::filesystem::path &directory)
{
    return static_cast<std::size_t>(std::distance(std::filesystem::directory_iterator(directory), std::filesystem::directory_iterator()));
}

}

#endif
