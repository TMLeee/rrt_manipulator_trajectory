#include "FrameTask.h"
#include "MathUtils.h"

FrameTask::FrameTask(const std::string& name, int frame_id, std::unique_ptr<ControlLaw> law,
                     std::array<bool, 6> mask)
    : Task(name, std::move(law)), frame_id_(frame_id), mask_(mask) {
    for (int i = 0; i < 6; ++i)
        if (mask_[i]) rows_.push_back(i);
    dim_ = static_cast<int>(rows_.size());
}

Eigen::VectorXd FrameTask::error(const RobotModel& m) const {
    // 전체 6D 오차: [위치오차; 자세오차(쿼터니언)]
    Eigen::VectorXd e6(6);
    e6.head(3) = p_des_ - m.framePos(frame_id_);
    e6.tail(3) = mc::orientationError(R_des_, m.frameRot(frame_id_));
    // 선택 마스크 적용
    Eigen::VectorXd e(dim_);
    for (int r = 0; r < dim_; ++r) e(r) = e6(rows_[r]);
    return e;
}

Eigen::MatrixXd FrameTask::jacobian(const RobotModel& m) const {
    Eigen::MatrixXd J6 = m.frameJacobian(frame_id_).leftCols(m.armDof());  // 6 x armDof
    Eigen::MatrixXd J(dim_, m.armDof());
    for (int r = 0; r < dim_; ++r) J.row(r) = J6.row(rows_[r]);
    return J;
}
