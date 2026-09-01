#include "praxis/rigid_motion/baseline/frame.h"

#include <Eigen/Core>

#include <iostream>
#include <string_view>

namespace consuming {

std::string_view the_baseline_inverse_undoes_the_motion()
{
    using namespace praxis;
    using namespace praxis::rigid_motion;

    const rotation turned      = rotate_z(0.7);
    const transform placed     = transformation_matrix_from_rotation_position(turned, Eigen::Vector3d(1.0, -2.0, 0.5));
    const transform recomposed = inverse(placed) * placed;

    if((recomposed - transform::Identity()).cwiseAbs().maxCoeff() > 1.0e-12)
        return "the inverse of a baseline rigid motion does not compose with it to the identity";

    return {};
}

}

int main()
{
    const std::string_view failed = consuming::the_baseline_inverse_undoes_the_motion();
    if(failed.empty())
        return 0;

    std::cerr << "the consuming project failed: " << failed << '\n';

    return 1;
}
