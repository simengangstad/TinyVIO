#ifndef MATH_UTILS_H
#define MATH_UTILS_H

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
#include <Eigen/Dense>
#pragma GCC diagnostic pop

namespace math {

    /**
     * @brief Computes the skew symmetric matrix of the vector @p w.
     */
    inline __attribute__((always_inline)) Eigen::Matrix3f
    skew_symmetric(const Eigen::Vector3f& w) {
        Eigen::Matrix3f S;

        S(0, 0) = 0;
        S(0, 1) = -w.z();
        S(0, 2) = w.y();

        S(1, 0) = w.z();
        S(1, 1) = 0;
        S(1, 2) = -w.x();

        S(2, 0) = -w.y();
        S(2, 1) = w.x();
        S(2, 2) = 0;

        return S;
    }

    /**
     * @brief Computes the small angle error-quaternion from a given @p dtheta.
     *
     * @param dtheta [in] The angle-axis representation of the small angle
     * error-quaternion.
     */
    inline __attribute__((always_inline)) Eigen::Quaternionf
    small_angle_quaternion(const Eigen::Vector3f& dtheta) {

        Eigen::Vector3f dq = dtheta / 2.0;
        Eigen::Quaternionf q;
        double dq_square_norm = dq.squaredNorm();

        if (dq_square_norm <= 1) {
            q.w() = sqrt(1.0f - dq_square_norm);
            q.x() = dq.x();
            q.y() = dq.y();
            q.z() = dq.z();
        } else {
            q.w() = 1.0f / sqrt(1 + dq_square_norm);
            q.x() = dq.x() / sqrt(1 + dq_square_norm);
            q.y() = dq.y() / sqrt(1 + dq_square_norm);
            q.z() = dq.z() / sqrt(1 + dq_square_norm);
        }

        return q.normalized();
    }
}

#endif
