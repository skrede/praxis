#include "praxis/manipulator/csv.h"

#include <catch2/catch_test_macros.hpp>

#include <Eigen/Core>

#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include <filesystem>
#include <system_error>

namespace {

using praxis::manipulator::eigen_vector_to_csv_file;
using praxis::manipulator::std_vector_to_csv_file;

// A device that accepts an open and refuses every write. The failures a recording actually meets — a
// filled disk, an exceeded quota, a mount gone read-only — surface at the flush and nowhere earlier,
// and this is the one target that reproduces that without filling a filesystem.
const std::filesystem::path refusing_every_write{"/dev/full"};

class scratch_tree
{
public:
    explicit scratch_tree(const std::string &named)
            : m_root(std::filesystem::temp_directory_path() / ("praxis_" + named + "_fixture"))
    {
        std::error_code ignored;
        std::filesystem::remove_all(m_root, ignored);
        std::filesystem::create_directories(m_root);
    }

    scratch_tree(const scratch_tree &)            = delete;
    scratch_tree &operator=(const scratch_tree &) = delete;

    ~scratch_tree()
    {
        std::error_code ignored;
        std::filesystem::remove_all(m_root, ignored);
    }

    const std::filesystem::path &root() const
    {
        return m_root;
    }

private:
    std::filesystem::path m_root;
};

// Few enough rows that every byte is still in the stream's buffer when the writer takes its answer,
// which is the case a recording of a short motion produces and the one a premature answer gets wrong.
std::vector<double> gathered_instants()
{
    return std::vector<double>{0.0, 0.1, 0.2};
}

std::vector<Eigen::VectorXd> gathered_positions()
{
    std::vector<Eigen::VectorXd> positions;
    for(int row = 0; row < 3; ++row)
        positions.push_back(Eigen::VectorXd::Constant(2, static_cast<double>(row)));

    return positions;
}

std::string contents(const std::filesystem::path &of)
{
    std::ifstream read(of, std::ios::binary);
    std::ostringstream whole;
    whole << read.rdbuf();

    return whole.str();
}

}

TEST_CASE("rows that reached their file are reported as written", "[manipulator][csv]")
{
    const scratch_tree chosen("csv_written");

    REQUIRE(std_vector_to_csv_file(gathered_instants(), chosen.root() / "timestamp.csv"));
    REQUIRE(eigen_vector_to_csv_file(gathered_positions(), chosen.root() / "trajectory.csv"));
    REQUIRE_FALSE(contents(chosen.root() / "timestamp.csv").empty());
    REQUIRE_FALSE(contents(chosen.root() / "trajectory.csv").empty());
}

TEST_CASE("a target that cannot be opened is not reported as written", "[manipulator][csv]")
{
    const scratch_tree chosen("csv_unopenable");
    const std::filesystem::path absent = chosen.root() / "no_such_folder" / "timestamp.csv";

    REQUIRE_FALSE(std_vector_to_csv_file(gathered_instants(), absent));
    REQUIRE_FALSE(eigen_vector_to_csv_file(gathered_positions(), absent));
}

TEST_CASE("rows a buffered write never delivered are not reported as written", "[manipulator][csv]")
{
    if(!std::filesystem::exists(refusing_every_write))
        SKIP("this system carries no target that accepts an open and refuses every write");

    REQUIRE_FALSE(std_vector_to_csv_file(gathered_instants(), refusing_every_write));
    REQUIRE_FALSE(eigen_vector_to_csv_file(gathered_positions(), refusing_every_write));
}
