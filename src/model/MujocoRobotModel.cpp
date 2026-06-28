#include "MujocoRobotModel.h"
#include "MujocoEnv.h"

MujocoRobotModel::MujocoRobotModel(mjModel* m, mjData* d, int arm_dof)
    : m_(m), d_(d), arm_dof_(arm_dof) {}

MujocoRobotModel::MujocoRobotModel(MujocoEnv& env, int arm_dof)
    : m_(env.model()), d_(env.data()), arm_dof_(arm_dof) {}

void MujocoRobotModel::setState(const Eigen::VectorXd& q, const Eigen::VectorXd& v) {
    Eigen::Map<Eigen::VectorXd>(d_->qpos, m_->nq) = q;
    Eigen::Map<Eigen::VectorXd>(d_->qvel, m_->nv) = v;
    mj_forward(m_, d_);  // FK + Jacobian 지원량 + 동역학 파생량 갱신
}

Eigen::VectorXd MujocoRobotModel::qpos() const {
    return Eigen::Map<const Eigen::VectorXd>(d_->qpos, m_->nq);
}
Eigen::VectorXd MujocoRobotModel::qvel() const {
    return Eigen::Map<const Eigen::VectorXd>(d_->qvel, m_->nv);
}

int MujocoRobotModel::frameId(const std::string& name) const {
    return mj_name2id(m_, mjOBJ_BODY, name.c_str());
}

Eigen::Vector3d MujocoRobotModel::framePos(int frame_id) const {
    return Eigen::Map<const Eigen::Vector3d>(d_->xpos + 3 * frame_id);
}

Eigen::Matrix3d MujocoRobotModel::frameRot(int frame_id) const {
    // MuJoCo xmat 은 row-major → RowMajor 로 매핑 (body→world)
    return Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(d_->xmat + 9 * frame_id);
}

Eigen::MatrixXd MujocoRobotModel::frameJacobian(int frame_id) const {
    int nv = m_->nv;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> jacp(3, nv), jacr(3, nv);
    mj_jacBody(m_, d_, jacp.data(), jacr.data(), frame_id);
    Eigen::MatrixXd J(6, nv);
    J.topRows(3)    = jacp;  // LINEAR
    J.bottomRows(3) = jacr;  // ANGULAR
    return J;
}

Eigen::MatrixXd MujocoRobotModel::massMatrix() const {
    int nv = m_->nv;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> M(nv, nv);
    mj_fullM(m_, M.data(), d_->qM);  // 희소 qM → 밀집 행렬
    return M;
}

Eigen::VectorXd MujocoRobotModel::biasForces() const {
    // qfrc_bias = Coriolis + 원심 + 중력
    return Eigen::Map<const Eigen::VectorXd>(d_->qfrc_bias, m_->nv);
}

Eigen::VectorXd MujocoRobotModel::gravity() const {
    // 속도 0 에서의 bias = 중력항. d_ 상태를 일시 변경 후 복원.
    int nv = m_->nv;
    Eigen::VectorXd v_save = Eigen::Map<Eigen::VectorXd>(d_->qvel, nv);
    Eigen::Map<Eigen::VectorXd>(d_->qvel, nv).setZero();
    mj_forward(m_, d_);
    Eigen::VectorXd g = Eigen::Map<const Eigen::VectorXd>(d_->qfrc_bias, nv);
    Eigen::Map<Eigen::VectorXd>(d_->qvel, nv) = v_save;
    mj_forward(m_, d_);
    return g;
}
