#pragma once

#include <vector>
#include <memory>
#include <Eigen/Dense>
#include "RobotModel.h"
#include "Task.h"

// null-space 우선순위 DLS 솔버. 등록순=우선순위(0이 최상위).
// model 은 호출 전에 setState 로 동기화되어 있어야 함.
class HierarchicalSolver {
public:
    explicit HierarchicalSolver(double lambda = 0.03) : lambda_(lambda) {}

    // 우선순위 순서 task 스택 → 관절속도 dq(armDof) 합성
    Eigen::VectorXd solve(const RobotModel& model,
                          const std::vector<std::shared_ptr<Task>>& tasks,
                          const Eigen::VectorXd& v_arm, double dt) const;

    void setLambda(double l) { lambda_ = l; }

private:
    double lambda_;
};
