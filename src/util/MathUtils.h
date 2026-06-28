#pragma once

#include <cmath>
#include <Eigen/Dense>
#include <rbdl/rbdl_math.h>

// 모션 컨트롤러 공용 수학 유틸 (header-only)
namespace mc {

using RigidBodyDynamics::Math::Quaternion;

// 회전행렬 → 쿼터니언 (Shepperd 분기, 수치 안정)
inline Quaternion rotationToQuaternion(const Eigen::Matrix3d& rot) {
    Quaternion q;
    double S;
    double tr = rot(0, 0) + rot(1, 1) + rot(2, 2);
    if (tr > 0) {
        S = std::sqrt(tr + 1.0) * 2.0;            // S = 4*qw
        q.w() = 0.25 * S;
        q.x() = (rot(2, 1) - rot(1, 2)) / S;
        q.y() = (rot(0, 2) - rot(2, 0)) / S;
        q.z() = (rot(1, 0) - rot(0, 1)) / S;
    } else if ((rot(0, 0) > rot(1, 1)) && (rot(0, 0) > rot(2, 2))) {
        S = std::sqrt(1.0 + rot(0, 0) - rot(1, 1) - rot(2, 2)) * 2.0;  // S = 4*qx
        q.w() = (rot(2, 1) - rot(1, 2)) / S;
        q.x() = 0.25 * S;
        q.y() = (rot(0, 1) + rot(1, 0)) / S;
        q.z() = (rot(0, 2) + rot(2, 0)) / S;
    } else if (rot(1, 1) > rot(2, 2)) {
        S = std::sqrt(1.0 + rot(1, 1) - rot(0, 0) - rot(2, 2)) * 2.0;  // S = 4*qy
        q.w() = (rot(0, 2) - rot(2, 0)) / S;
        q.x() = (rot(0, 1) + rot(1, 0)) / S;
        q.y() = 0.25 * S;
        q.z() = (rot(1, 2) + rot(2, 1)) / S;
    } else {
        S = std::sqrt(1.0 + rot(2, 2) - rot(0, 0) - rot(1, 1)) * 2.0;  // S = 4*qz
        q.w() = (rot(1, 0) - rot(0, 1)) / S;
        q.x() = (rot(0, 2) + rot(2, 0)) / S;
        q.y() = (rot(1, 2) + rot(2, 1)) / S;
        q.z() = 0.25 * S;
    }
    return q;
}

// 쿼터니언 → ZYX 오일러각 (roll, pitch, yaw)
inline Eigen::Vector3d quaternionToEuler(const Quaternion& q) {
    Eigen::Vector3d angle;
    angle(0) = std::atan2(2.0 * (q.w() * q.x() + q.y() * q.z()),
                          1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y()));
    double sinp = 2.0 * (q.w() * q.y() - q.z() * q.x());
    angle(1) = (std::fabs(sinp) >= 1.0) ? std::copysign(M_PI / 2, sinp) : std::asin(sinp);
    angle(2) = std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
                          1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
    return angle;
}

// 자세 오차 3-벡터: q_err = q_des * q_cur^{-1} (현재→목표), w<0 이면 최단경로로 부호 정정
inline Eigen::Vector3d orientationError(const Eigen::Matrix3d& R_des,
                                        const Eigen::Matrix3d& R_cur) {
    Quaternion q_des = rotationToQuaternion(R_des);
    Quaternion q_cur = rotationToQuaternion(R_cur);
    Quaternion q_err = q_des * q_cur.conjugate();
    if (q_err.w() < 0) {                  // 이중덮개 보정: 항상 짧은 회전
        q_err.w() = -q_err.w();
        q_err.x() = -q_err.x();
        q_err.y() = -q_err.y();
        q_err.z() = -q_err.z();
    }
    return quaternionToEuler(q_err);
}

// 댐핑 최소자승 의사역행렬: Jpinv = Jᵀ(JJᵀ + λ²I)⁻¹  (rows ≤ cols 인 매니퓰레이터 경우)
// task 속도 해석용 (특이점 강건).
inline Eigen::MatrixXd dampedPinv(const Eigen::MatrixXd& J, double lambda) {
    const int rows = static_cast<int>(J.rows());
    Eigen::MatrixXd A = J * J.transpose() +
                        lambda * lambda * Eigen::MatrixXd::Identity(rows, rows);
    return J.transpose() * A.ldlt().solve(Eigen::MatrixXd::Identity(rows, rows));
}

// SVD 기반 true 의사역행렬 (특이값 절단). null-space 투영자 N 을 크리스프하게 만들 때 사용.
inline Eigen::MatrixXd pinvSvd(const Eigen::MatrixXd& J, double tol = 1e-6) {
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(J, Eigen::ComputeThinU | Eigen::ComputeThinV);
    const Eigen::VectorXd& sv = svd.singularValues();
    Eigen::VectorXd inv(sv.size());
    for (int i = 0; i < sv.size(); ++i) inv(i) = (sv(i) > tol) ? 1.0 / sv(i) : 0.0;
    return svd.matrixV() * inv.asDiagonal() * svd.matrixU().transpose();
}

}  // namespace mc
