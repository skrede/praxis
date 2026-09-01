#ifndef HPP_GUARD_PRAXIS_MANIPULATOR_CSV_H
#define HPP_GUARD_PRAXIS_MANIPULATOR_CSV_H

#include <Eigen/Core>

#include <ctime>
#include <string>
#include <vector>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <concepts>
#include <filesystem>
#include <type_traits>

namespace praxis::manipulator {

inline std::filesystem::path date_time_stamped_filename(const std::string &name)
{
    const std::time_t now = std::time(nullptr);
    const std::tm *local  = std::localtime(&now);

    std::ostringstream stamped;
    if(local)
        stamped << std::put_time(local, "%Y-%m-%d_%H-%M-%S") << "_";
    stamped << name;

    return stamped.str();
}

// Both writers answer whether everything asked of them reached the file: a target that could not be
// opened, a stream that failed part way through and a buffer the closing flush could not write all
// answer false. The stream is closed before the answer is taken, because the rows a small recording
// gathers sit entirely in the stream's buffer until then and a flush that fails at destruction is
// reportable by nothing.
template<std::floating_point F>
bool eigen_vector_to_csv_file(const std::vector<Eigen::Matrix<F, Eigen::Dynamic, 1>> &vectors, const std::filesystem::path &path)
{
    std::ofstream csv_file(path);
    const Eigen::IOFormat csv_format(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ");
    for(const auto &vector : vectors)
        csv_file << vector.transpose().format(csv_format) << '\n';
    csv_file.close();

    return csv_file.good();
}

template<typename T>
    requires std::is_fundamental_v<T>
bool std_vector_to_csv_file(const std::vector<T> &values, const std::filesystem::path &path)
{
    std::ofstream csv_file(path);
    for(const auto &value : values)
        csv_file << value << '\n';
    csv_file.close();

    return csv_file.good();
}

}

#endif
