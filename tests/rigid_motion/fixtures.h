#ifndef HPP_GUARD_PRAXIS_TESTS_RIGID_MOTION_FIXTURES_H
#define HPP_GUARD_PRAXIS_TESTS_RIGID_MOTION_FIXTURES_H

#include "praxis/rigid_motion/types.h"

#include "praxis/evaluation/tolerance.h"

#include <Eigen/Geometry>

#include <cmath>
#include <numbers>

// The screw motion the recorded mathematics values are written around, carried over digit for
// digit: re-deriving them from the formulae under test would make the suite agree with itself.
namespace praxis::conformance {

inline const Eigen::Vector3d point{0.24, -1.2, 2.5};
inline const Eigen::Vector3d direction = Eigen::Vector3d{1.0, -3.0, 4.0}.normalized();

inline constexpr double pitch = 2.25;
inline constexpr double angle = 0.721;

inline screw_axis reference_screw()
{
    screw_axis s;
    s << 0.196116135138184044528486538184, -0.588348405414552133585459614551, 0.784464540552736178113946152735, 0.97077486893401099266043274838, -1.02176506406993894415791146457,
            1.85918096110998476433451287448;
    return s;
}

inline twist reference_twist()
{
    twist t;
    t << 0.141399733434630692219258207842, -0.424199200303892076657774623527, 0.565598933738522768877032831369, 0.69992868050142198477203692164, -0.73669261119442586149830276554,
            1.34046947296029883567314300308;
    return t;
}

inline rotation reference_rotation()
{
    rotation r;
    r << 0.76071727757306506489953790151, -0.546567326642526496449647765985, -0.350104814375161110806544684237, 0.48913947326006212978910525635, 0.837287748749684257454362068529,
            -0.244319056752752339356504762691, 0.426675285551780303361368851256, 0.014607643222894817203183492893, 0.904286911029225981550894175598;
    return r;
}

inline Eigen::Vector3d reference_position()
{
    return Eigen::Vector3d{0.594958497577254497024057400267, -0.65629873388467019346137476532, 1.42700742667365787497146811802};
}

inline adjoint reference_adjoint()
{
    adjoint a           = adjoint::Zero();
    a.block<3, 3>(0, 0) = reference_rotation();
    a.block<3, 3>(3, 3) = reference_rotation();
    a.block<3, 3>(3, 0) << -0.978032110708863200443374807946, -1.20440281348089195390116401541, -0.244837246312879952547802986373, 0.831695117850496989930775271205,
            -0.788646575761090318223978101742, -1.03761535219228195892071653361, 0.790275472231932063493786699837, 0.139440016577734748892680727295, -0.375133045336455195339908641472;
    return a;
}

inline screw_axis adjoint_probe()
{
    screw_axis s;
    s << 0.429437750933887196769234151361, 0.429437750933887196769234151361, -0.794459839227691366758676849713, -8.26667670547733024477565777488, 2.70545783088348956724189520173,
            -3.00606425653721043289579029079;
    return s;
}

inline twist adjoint_probe_image()
{
    twist t;
    t << 0.370108287831543980583859365652, 0.763719601545402193565337256587, -0.528916085493469245726316785294, -7.45758766946363493843819014728, -0.201042014096709520387662450958,
            -5.50872777461524076869636701304;
    return t;
}

inline rotation axis_angle_rotation(const Eigen::Vector3d &axis, double radians)
{
    return Eigen::AngleAxisd(radians, axis).toRotationMatrix();
}

inline rotation intrinsic_zyx(const Eigen::Vector3d &e)
{
    return axis_angle_rotation(Eigen::Vector3d::UnitZ(), e[0]) * axis_angle_rotation(Eigen::Vector3d::UnitY(), e[1]) * axis_angle_rotation(Eigen::Vector3d::UnitX(), e[2]);
}

inline transform assembled(const rotation &r, const Eigen::Vector3d &p)
{
    transform tf         = transform::Identity();
    tf.block<3, 3>(0, 0) = r;
    tf.block<3, 1>(0, 3) = p;
    return tf;
}

// The columns of the cross-product operator are the images of the basis vectors, which is the
// characterization of the matrix rather than a transcription of its entries.
inline matrix3 cross_operator(const Eigen::Vector3d &v)
{
    matrix3 m;
    m.col(0) = v.cross(Eigen::Vector3d::UnitX());
    m.col(1) = v.cross(Eigen::Vector3d::UnitY());
    m.col(2) = v.cross(Eigen::Vector3d::UnitZ());
    return m;
}

inline bool same_angle(double first, double second)
{
    return is_approx_equal(std::remainder(first - second, 2.0 * std::numbers::pi_v<double>), 0.0);
}

}

#endif
