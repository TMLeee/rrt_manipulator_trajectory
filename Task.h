#pragma once

#include <string>
#include <memory>
#include <Eigen/Dense>
#include "RobotModel.h"
#include "ControlLaw.h"

// 작업(Task) 추상 베이스 (req6). target + ControlLaw + 프레임/DOF 선택을 묶고,
// 자신의 오차/Jacobian/기준속도를 산출. 솔버는 이 인터페이스만 사용.
class Task {
public:
    Task(std::string name, std::unique_ptr<ControlLaw> law)
        : name_(std::move(name)), law_(std::move(law)) {}
    virtual ~Task() = default;

    virtual int dimension() const = 0;                              // 이 task 의 행 수
    virtual Eigen::VectorXd error(const RobotModel& m) const = 0;   // (dim)
    virtual Eigen::MatrixXd jacobian(const RobotModel& m) const = 0;// (dim x armDof), 이미 leftCols 슬라이스

    // 기준 task 속도 = ControlLaw(error, edot). edot = -(J·v_arm) (목표 task 속도 0 가정)
    Eigen::VectorXd referenceVelocity(const RobotModel& m,
                                      const Eigen::VectorXd& v_arm, double dt) {
        Eigen::VectorXd e = error(m);
        Eigen::VectorXd edot = -(jacobian(m) * v_arm);
        return law_->compute(e, edot, dt);
    }

    ControlLaw* controlLaw() { return law_.get(); }
    const std::string& name() const { return name_; }

protected:
    std::string name_;
    std::unique_ptr<ControlLaw> law_;
};
