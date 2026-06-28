#include "Controller.h"

ClikController::ClikController(std::shared_ptr<RobotModel> model, double lambda)
    : model_(std::move(model)), solver_(lambda) {
    q_des_ = model_->qpos().head(model_->armDof());
}

void ClikController::start() {
    for (auto& task : tasks_) task->controlLaw()->reset();   // 적분기 초기화
    q_des_ = model_->qpos().head(model_->armDof());          // 현재 자세에서 출발
}

Eigen::VectorXd ClikController::update(double dt) {
    // 현재 상태로 모델 동기화 (MuJoCo: 라이브 d_ 값 + forward / RBDL: UpdateKinematics)
    Eigen::VectorXd q = model_->qpos();
    Eigen::VectorXd v = model_->qvel();
    model_->setState(q, v);

    Eigen::VectorXd v_arm = v.head(model_->armDof());
    Eigen::VectorXd dq = solver_.solve(*model_, tasks_, v_arm, dt);  // null-space DLS
    q_des_ += dq * dt;                                               // 목표 관절각 적분
    return q_des_;
}
