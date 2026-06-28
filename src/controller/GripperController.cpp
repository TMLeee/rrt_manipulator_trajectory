#include "GripperController.h"
#include "MujocoEnv.h"

#include <algorithm>
#include <cmath>

GripperController::GripperController(mjModel* m, mjData* d, const std::string& a)
    : m_(m), d_(d) { init(a); }

GripperController::GripperController(MujocoEnv& env, const std::string& a)
    : m_(env.model()), d_(env.data()) { init(a); }

void GripperController::init(const std::string& a) {
    idx_ = mj_name2id(m_, mjOBJ_ACTUATOR, a.c_str());
    if (idx_ < 0) return;
    if (m_->actuator_ctrllimited[idx_]) {
        lo_ = m_->actuator_ctrlrange[2 * idx_];
        hi_ = m_->actuator_ctrlrange[2 * idx_ + 1];
    }
    // 현재 ctrl 값에서 시작 (명령 점프 방지)
    double c = d_->ctrl[idx_];
    cmd_ = (hi_ > lo_) ? std::clamp((c - lo_) / (hi_ - lo_), 0.0, 1.0) : 0.0;
    target_ = cmd_;
}

void GripperController::setTarget(double open_frac, double vel_limit, double current_limit) {
    target_ = std::clamp(open_frac, 0.0, 1.0);
    vel_    = std::max(0.0, vel_limit);
    ilim_   = std::max(0.0, current_limit);
}

void GripperController::update(double dt) {
    if (idx_ < 0) return;
    double f = d_->actuator_force[idx_];
    bool closing = target_ < cmd_;  // 개방률 감소 = 닫힘

    // 닫힘 방향에서 힘(전류) 한계 도달 시 정지(grasp). 열림(릴리스)은 항상 허용.
    if (!(closing && std::fabs(f) >= ilim_)) {
        double step = vel_ * dt;
        if (std::fabs(target_ - cmd_) <= step) cmd_ = target_;
        else cmd_ += (closing ? -step : step);
    }
    cmd_ = std::clamp(cmd_, 0.0, 1.0);
    d_->ctrl[idx_] = lo_ + cmd_ * (hi_ - lo_);  // 그리퍼 ctrl 만 갱신
}

double GripperController::force() const {
    return (idx_ >= 0) ? d_->actuator_force[idx_] : 0.0;
}
