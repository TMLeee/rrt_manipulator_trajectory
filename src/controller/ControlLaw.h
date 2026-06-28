#pragma once

#include <Eigen/Dense>

// task별 제어법칙 (req4 "각각에 대한 제어기"). 오차 e, 오차미분 edot → 기준 task 속도.
class ControlLaw {
public:
    virtual ~ControlLaw() = default;
    virtual Eigen::VectorXd compute(const Eigen::VectorXd& e,
                                    const Eigen::VectorXd& edot, double dt) = 0;
    virtual void reset() = 0;
};

// PID: u = Kp⊙e + Kd⊙edot + Ki⊙∫e. 게인은 스칼라(브로드캐스트) 또는 성분별 벡터.
class PIDControlLaw : public ControlLaw {
public:
    PIDControlLaw(double kp, double kd = 0.0, double ki = 0.0)
        : kp_s_(kp), kd_s_(kd), ki_s_(ki) {}
    PIDControlLaw(const Eigen::VectorXd& kp, const Eigen::VectorXd& kd, const Eigen::VectorXd& ki)
        : kp_v_(kp), kd_v_(kd), ki_v_(ki) {}

    void setIntegralLimit(double lim) { i_limit_ = lim; }

    Eigen::VectorXd compute(const Eigen::VectorXd& e,
                            const Eigen::VectorXd& edot, double dt) override {
        if (e_int_.size() != e.size()) e_int_ = Eigen::VectorXd::Zero(e.size());
        e_int_ += e * dt;
        // anti-windup: 성분별 클램프
        e_int_ = e_int_.cwiseMax(-i_limit_).cwiseMin(i_limit_);
        return apply(kp_s_, kp_v_, e) + apply(kd_s_, kd_v_, edot) + apply(ki_s_, ki_v_, e_int_);
    }
    void reset() override { e_int_.setZero(); }

private:
    // 벡터게인이 차원과 맞으면 성분별, 아니면 스칼라 브로드캐스트
    static Eigen::VectorXd apply(double gs, const Eigen::VectorXd& gv, const Eigen::VectorXd& x) {
        if (gv.size() == x.size()) return gv.cwiseProduct(x);
        return gs * x;
    }
    double kp_s_ = 0, kd_s_ = 0, ki_s_ = 0;
    Eigen::VectorXd kp_v_, kd_v_, ki_v_;
    Eigen::VectorXd e_int_;
    double i_limit_ = 1e9;
};

// P 전용 (Kd=Ki=0)
class PControlLaw : public PIDControlLaw {
public:
    explicit PControlLaw(double kp) : PIDControlLaw(kp, 0.0, 0.0) {}
    explicit PControlLaw(const Eigen::VectorXd& kp)
        : PIDControlLaw(kp, Eigen::VectorXd::Zero(kp.size()), Eigen::VectorXd::Zero(kp.size())) {}
};
