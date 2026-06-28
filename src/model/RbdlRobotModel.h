#pragma once

#include <string>
#include <rbdl/rbdl.h>
#include "RobotModel.h"

// RBDL 백엔드 — URDF 로 모델 구성. (현재 Panda URDF 부재로 구조만, 후속 검증)
// Jacobian/회전은 RobotModel 컨벤션(LINEAR/ANGULAR, body→world)으로 정규화.
class RbdlRobotModel : public RobotModel {
public:
    // URDF 경로로 생성 (실패 시 std::runtime_error). floating_base=false 가정.
    explicit RbdlRobotModel(const std::string& urdf_path, int arm_dof = 7);

    int nq() const override { return static_cast<int>(model_.q_size); }
    int nv() const override { return static_cast<int>(model_.qdot_size); }
    int armDof() const override { return arm_dof_; }

    void setState(const Eigen::VectorXd& q, const Eigen::VectorXd& v) override;
    Eigen::VectorXd qpos() const override { return Q_; }
    Eigen::VectorXd qvel() const override { return QDot_; }

    int frameId(const std::string& name) const override;
    Eigen::Vector3d framePos(int frame_id) const override;
    Eigen::Matrix3d frameRot(int frame_id) const override;
    Eigen::MatrixXd frameJacobian(int frame_id) const override;

    Eigen::MatrixXd massMatrix() const override;
    Eigen::VectorXd biasForces() const override;
    Eigen::VectorXd gravity() const override;

private:
    mutable RigidBodyDynamics::Model model_;
    RigidBodyDynamics::Math::VectorNd Q_;     // 현재 위치
    RigidBodyDynamics::Math::VectorNd QDot_;  // 현재 속도
    int arm_dof_ = 7;
};
