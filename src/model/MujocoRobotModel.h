#pragma once

#include <mujoco/mujoco.h>
#include "RobotModel.h"

class MujocoEnv;  // 전방선언

// MuJoCo 백엔드 (raw mjModel/mjData 기반, 비소유). frame = body id.
class MujocoRobotModel : public RobotModel {
public:
    MujocoRobotModel(mjModel* m, mjData* d, int arm_dof = 7);
    explicit MujocoRobotModel(MujocoEnv& env, int arm_dof = 7);

    int nq() const override { return m_->nq; }
    int nv() const override { return m_->nv; }
    int armDof() const override { return arm_dof_; }

    void setState(const Eigen::VectorXd& q, const Eigen::VectorXd& v) override;
    Eigen::VectorXd qpos() const override;
    Eigen::VectorXd qvel() const override;

    int frameId(const std::string& name) const override;     // body id
    Eigen::Vector3d framePos(int frame_id) const override;
    Eigen::Matrix3d frameRot(int frame_id) const override;
    Eigen::MatrixXd frameJacobian(int frame_id) const override;

    Eigen::MatrixXd massMatrix() const override;
    Eigen::VectorXd biasForces() const override;
    Eigen::VectorXd gravity() const override;

private:
    mjModel* m_ = nullptr;  // 비소유
    mjData*  d_ = nullptr;  // 비소유
    int arm_dof_ = 7;
};
