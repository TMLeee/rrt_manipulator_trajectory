#pragma once

#include <string>
#include <Eigen/Dense>

// 로봇 모델 추상 인터페이스 — MuJoCo / RBDL 백엔드 호환 레이어.
// 모든 Jacobian 은 LINEAR(0-2)/ANGULAR(3-5) 순서, frameRot 은 body→world 로 정규화.
class RobotModel {
public:
    virtual ~RobotModel() = default;

    // ---- 차원 ----
    virtual int nq() const = 0;       // 위치 좌표 수
    virtual int nv() const = 0;       // 자유도(속도 좌표) 수
    virtual int armDof() const = 0;   // 제어 대상 팔 자유도 (Panda=7)

    // ---- 상태 동기화: 이후 FK/Jacobian/동역학 질의가 이 상태를 반영 ----
    virtual void setState(const Eigen::VectorXd& q, const Eigen::VectorXd& v) = 0;
    virtual Eigen::VectorXd qpos() const = 0;
    virtual Eigen::VectorXd qvel() const = 0;

    // ---- 이름 → frame id (없으면 -1) ----
    virtual int frameId(const std::string& name) const = 0;

    // ---- 순기구학 ----
    virtual Eigen::Vector3d framePos(int frame_id) const = 0;  // 월드 위치
    virtual Eigen::Matrix3d frameRot(int frame_id) const = 0;  // body→world 회전

    // ---- 6 x nv frame Jacobian (LINEAR 0-2, ANGULAR 3-5) ----
    virtual Eigen::MatrixXd frameJacobian(int frame_id) const = 0;

    // ---- 동역학 (노출만, 후속 동역학 컨트롤러에서 사용) ----
    virtual Eigen::MatrixXd massMatrix() const = 0;   // M (nv x nv)
    virtual Eigen::VectorXd biasForces() const = 0;   // h = Coriolis + gravity (nv)
    virtual Eigen::VectorXd gravity() const = 0;      // g (nv)
};
