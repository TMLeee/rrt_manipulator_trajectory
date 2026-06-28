#include "RbdlRobotModel.h"

#include <stdexcept>
#include <limits>
#include <rbdl/addons/urdfreader/urdfreader.h>

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

RbdlRobotModel::RbdlRobotModel(const std::string& urdf_path, int arm_dof)
    : arm_dof_(arm_dof) {
    if (!Addons::URDFReadFromFile(urdf_path.c_str(), &model_, false))
        throw std::runtime_error("RBDL URDF load failed: " + urdf_path);
    Q_    = VectorNd::Zero(model_.q_size);
    QDot_ = VectorNd::Zero(model_.qdot_size);
    UpdateKinematics(model_, Q_, QDot_, VectorNd::Zero(model_.qdot_size));
}

void RbdlRobotModel::setState(const Eigen::VectorXd& q, const Eigen::VectorXd& v) {
    // 주의(TODO): URDF 관절순서 ≠ MuJoCo qpos 일 수 있음 → 앞 arm_dof 정렬 가정.
    Q_    = q;
    QDot_ = v;
    UpdateKinematics(model_, Q_, QDot_, VectorNd::Zero(model_.qdot_size));
}

int RbdlRobotModel::frameId(const std::string& name) const {
    unsigned int id = model_.GetBodyId(name.c_str());
    return (id == std::numeric_limits<unsigned int>::max()) ? -1 : static_cast<int>(id);
}

Eigen::Vector3d RbdlRobotModel::framePos(int frame_id) const {
    return CalcBodyToBaseCoordinates(model_, Q_, frame_id, Vector3d::Zero(), false);
}

Eigen::Matrix3d RbdlRobotModel::frameRot(int frame_id) const {
    // RBDL 은 world→body 반환 → transpose 해서 body→world 로 정규화
    return CalcBodyWorldOrientation(model_, Q_, frame_id, false).transpose();
}

Eigen::MatrixXd RbdlRobotModel::frameJacobian(int frame_id) const {
    int nv = static_cast<int>(model_.qdot_size);
    MatrixNd G = MatrixNd::Zero(6, nv);
    CalcPointJacobian6D(model_, Q_, frame_id, Vector3d::Zero(), G, false);
    // RBDL: ANGULAR(0-2)/LINEAR(3-5) → 컨벤션 LINEAR/ANGULAR 로 행블록 swap
    Eigen::MatrixXd J(6, nv);
    J.topRows(3)    = G.bottomRows(3);  // LINEAR
    J.bottomRows(3) = G.topRows(3);     // ANGULAR
    return J;
}

Eigen::MatrixXd RbdlRobotModel::massMatrix() const {
    MatrixNd M = MatrixNd::Zero(model_.qdot_size, model_.qdot_size);
    CompositeRigidBodyAlgorithm(model_, Q_, M, false);
    return M;
}

Eigen::VectorXd RbdlRobotModel::biasForces() const {
    VectorNd h = VectorNd::Zero(model_.qdot_size);
    NonlinearEffects(model_, Q_, QDot_, h);  // Coriolis + 중력
    return h;
}

Eigen::VectorXd RbdlRobotModel::gravity() const {
    VectorNd g = VectorNd::Zero(model_.qdot_size);
    NonlinearEffects(model_, Q_, VectorNd::Zero(model_.qdot_size), g);  // 속도 0 → 중력만
    return g;
}
