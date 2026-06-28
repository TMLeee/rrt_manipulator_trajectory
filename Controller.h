#pragma once

#include <vector>
#include <memory>
#include <Eigen/Dense>
#include "RobotModel.h"
#include "Task.h"
#include "HierarchicalSolver.h"

// 모션 컨트롤러 추상 베이스 (req1). RobotModel 참조 + Task 스택 소유.
// 상위 플래너는 Task target 갱신으로만 연결.
class Controller {
public:
    virtual ~Controller() = default;

    // 등록순 = 우선순위 (먼저 등록한 task 가 높은 우선순위)
    void addTask(std::shared_ptr<Task> task) { tasks_.push_back(std::move(task)); }

    virtual void start() = 0;                      // 트리거 시 초기화
    virtual Eigen::VectorXd update(double dt) = 0; // 한 틱 → 팔 관절 명령(armDof)

protected:
    std::vector<std::shared_ptr<Task>> tasks_;
};

// DLS CLIK 구현 (req3). 속도 합성 → 목표 관절각 적분 → 위치 액추에이터 명령.
class ClikController : public Controller {
public:
    ClikController(std::shared_ptr<RobotModel> model, double lambda = 0.03);

    void start() override;                     // 각 ControlLaw reset + q_des = 현재 팔 관절각
    Eigen::VectorXd update(double dt) override;

private:
    std::shared_ptr<RobotModel> model_;
    HierarchicalSolver solver_;
    Eigen::VectorXd q_des_;  // 적분된 목표 팔 관절각 (armDof)
};
