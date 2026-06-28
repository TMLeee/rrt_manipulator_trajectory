#include "JointSpaceTask.h"

JointSpaceTask::JointSpaceTask(const std::string& name, std::unique_ptr<ControlLaw> law,
                               std::vector<int> idx, Eigen::VectorXd q_des)
    : Task(name, std::move(law)), idx_(std::move(idx)), q_des_(std::move(q_des)) {}

Eigen::VectorXd JointSpaceTask::error(const RobotModel& m) const {
    Eigen::VectorXd q_arm = m.qpos().head(m.armDof());
    Eigen::VectorXd e(idx_.size());
    for (size_t r = 0; r < idx_.size(); ++r) e(r) = q_des_(r) - q_arm(idx_[r]);
    return e;
}

Eigen::MatrixXd JointSpaceTask::jacobian(const RobotModel& m) const {
    // 선택행렬 S: 행 r 은 관절 idx_[r] 에 1
    Eigen::MatrixXd S = Eigen::MatrixXd::Zero(idx_.size(), m.armDof());
    for (size_t r = 0; r < idx_.size(); ++r) S(r, idx_[r]) = 1.0;
    return S;
}
