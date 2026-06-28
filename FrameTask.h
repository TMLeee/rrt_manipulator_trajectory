#pragma once

#include <array>
#include "Task.h"

// task space 작업: 한 프레임(EE)의 위치+자세 추종. 6-bit 선택마스크로 위치/자세/부분 선택.
// 행 순서는 RobotModel 컨벤션 [LINEAR(0-2); ANGULAR(3-5)] 과 동일.
class FrameTask : public Task {
public:
    // mask: 길이 6 (1=사용). 기본 위치전용 {1,1,1,0,0,0}.
    FrameTask(const std::string& name, int frame_id, std::unique_ptr<ControlLaw> law,
              std::array<bool, 6> mask = {true, true, true, false, false, false});

    // 상위(플래너/트래젝토리) 연결 seam — 목표 갱신
    void setTargetPos(const Eigen::Vector3d& p) { p_des_ = p; }
    void setTargetRot(const Eigen::Matrix3d& R) { R_des_ = R; }

    int dimension() const override { return dim_; }
    Eigen::VectorXd error(const RobotModel& m) const override;
    Eigen::MatrixXd jacobian(const RobotModel& m) const override;

private:
    int frame_id_;
    Eigen::Vector3d p_des_ = Eigen::Vector3d::Zero();
    Eigen::Matrix3d R_des_ = Eigen::Matrix3d::Identity();
    std::array<bool, 6> mask_;
    std::vector<int> rows_;  // 선택된 행 인덱스 (0..5)
    int dim_;
};
