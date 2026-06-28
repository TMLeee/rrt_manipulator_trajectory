#include "HierarchicalSolver.h"
#include "MathUtils.h"

Eigen::VectorXd HierarchicalSolver::solve(const RobotModel& model,
                                          const std::vector<std::shared_ptr<Task>>& tasks,
                                          const Eigen::VectorXd& v_arm, double dt) const {
    const int n = model.armDof();
    Eigen::VectorXd dq = Eigen::VectorXd::Zero(n);
    Eigen::MatrixXd N  = Eigen::MatrixXd::Identity(n, n);  // null-space 투영자

    for (const auto& task : tasks) {
        Eigen::MatrixXd J  = task->jacobian(model);                       // dim x n
        Eigen::VectorXd xr = task->referenceVelocity(model, v_arm, dt);   // dim
        Eigen::MatrixXd JN = J * N;                                       // 잔여 null-space 로 투영
        Eigen::MatrixXd JNpinv = mc::dampedPinv(JN, lambda_);            // n x dim (DLS, 속도 해석)
        dq += JNpinv * (xr - J * dq);          // 상위가 못 채운 잔차만 보정
        N  -= mc::pinvSvd(JN) * JN;            // 크리스프 투영자(SVD) → 상위 task 누설 차단
    }
    return dq;
}
