#pragma once

#include <vector>
#include "Task.h"

// joint space 작업: 선택한 관절 인덱스의 목표각 추종(posture/regularization).
// Jacobian 은 운동학 없이 선택행렬 S (dim x armDof).
class JointSpaceTask : public Task {
public:
    // idx: 제어할 팔 관절 인덱스(0..armDof-1). q_des: idx 와 같은 길이의 목표각.
    JointSpaceTask(const std::string& name, std::unique_ptr<ControlLaw> law,
                   std::vector<int> idx, Eigen::VectorXd q_des);

    void setTarget(const Eigen::VectorXd& q_des) { q_des_ = q_des; }  // 상위 연결 seam

    int dimension() const override { return static_cast<int>(idx_.size()); }
    Eigen::VectorXd error(const RobotModel& m) const override;
    Eigen::MatrixXd jacobian(const RobotModel& m) const override;

private:
    std::vector<int> idx_;
    Eigen::VectorXd q_des_;
};
