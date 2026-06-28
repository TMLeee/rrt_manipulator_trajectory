#include <iostream>
#include <stdexcept>
#include <rbdl/rbdl.h>
#include <rbdl/addons/urdfreader/urdfreader.h>
#include <Eigen/Dense>

#include "MujocoEnv.h"

using namespace std;
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;


Quaternion RotationMatrxToQuaternion(Eigen::Matrix3d rot) {

	Quaternion q;
	double S;
	double tr = rot(0, 0) + rot(1, 1) + rot(2, 2);

	if (tr > 0) {

		S = std::sqrt(tr + 1.f) * 2.f; // S=4*qw 
		q.w() = 0.25f * S;
		q.x() = (rot(2, 1) - rot(1, 2)) / S;
		q.y() = (rot(0, 2) - rot(2, 0)) / S;
		q.z() = (rot(1, 0) - rot(0, 1)) / S;
	}
	else if ((rot(0, 0) > rot(1, 1)) && (rot(0, 0) > rot(2, 2))) {

		S = std::sqrt(1.f + rot(0, 0) - rot(1, 1) - rot(2, 2)) * 2.f; // S=4*qx 
		q.w() = (rot(2, 1) - rot(1, 2)) / S;
		q.x() = 0.25f * S;
		q.y() = (rot(0, 1) + rot(1, 0)) / S;
		q.z() = (rot(0, 2) + rot(2, 0)) / S;
	}
	else if (rot(1, 1) > rot(2, 2)) {

		S = std::sqrt(1.f + rot(1, 1) - rot(0, 0) - rot(2, 2)) * 2.f; // S=4*qy
		q.w() = (rot(0, 2) - rot(2, 0)) / S;
		q.x() = (rot(0, 1) + rot(1, 0)) / S;
		q.y() = 0.25f * S;
		q.z() = (rot(1, 2) + rot(2, 1)) / S;
	}
	else {

		S = std::sqrt(1.f + rot(2, 2) - rot(0, 0) - rot(1, 1)) * 2.f; // S=4*qz
		q.w() = (rot(1, 0) - rot(0, 1)) / S;
		q.x() = (rot(0, 2) + rot(2, 0)) / S;
		q.y() = (rot(1, 2) + rot(2, 1)) / S;
		q.z() = 0.25f * S;
	}

	return q;
}


Vector3d QuaternionToEulerAngle(Quaternion &q)
{
	Vector3d angle;

	// roll (x-axis rotation)
	//double sinr_cosp = 2.0 * (q.w() * q.x() + q.y() * q.z());
	//double cosr_cosp = 1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
	//angle(0) = std::atan2(sinr_cosp, cosr_cosp);
	angle(0) = std::atan2(2.0 * (q.w() * q.x() + q.y() * q.z()), 1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y()));

	// pitch (y-axis rotation)
	double sinp = 2.0 * (q.w() * q.y() - q.z() * q.x());
	if (fabs(sinp) >= 1)
		angle(1) = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range

	else
		angle(1) = std::asin(sinp);

	// yaw (z-axis rotation)
	//double siny_cosp = 2.0 * (q.w() * q.z() + q.x() * q.y());
	//double cosy_cosp = 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
	//angle(2) = std::atan2(siny_cosp, cosy_cosp);
	angle(2) = std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()), 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));

	return angle;
}


int main(void)
{
    const char* model_path = "/home/user1/mani_ws/model/mujoco_menagerie/franka_emika_panda/scene.xml";

    // MuJoCo 환경 생성
    MujocoEnv env(model_path, 1200, 900, "MuJoCo Viewer (C++ from VS Code)");
    env.setCamera(120.0, -15.0, 3.0, 0.5, 0.0, 0.5);

    std::cout << "timestep = " << env.timestep() << std::endl;
    std::cout << "nq=" << env.nq() << " nv=" << env.nv() << " nu=" << env.nu() << std::endl;

    // 엔드이펙터 body id (없으면 마지막 body 로 대체)
    int ee_id = env.bodyId("hand");
    if (ee_id < 0) ee_id = env.model()->nbody - 1;

    // IK 파라미터 (Damped Least-Squares + task-space PID)
    const int    ARM    = 7;                 // 팔 자유도 (앞 7개)
    const double dt     = env.timestep();    // 적분 간격
    const double lambda = 0.03;              // DLS 댐핑 계수
    const double Kp_p = 3.0, Kp_o = 3.0;     // P: 위치 / 자세 오차 게인
    const double Kd_p = 0.1, Kd_o = 0.05;    // D: 위치 / 자세 속도 오차 게인
    const double Ki_p = 0.0, Ki_o = 0.0;     // I: 위치 / 자세 오차 적분 게인 (우선 0)

    // ===== 초기 자세: joint4 = -1.57, joint6 = 1.57 로 설정 후 안정화 =====
    {
        Eigen::VectorXd q0 = env.qpos();   // 기본 초기 관절각 (nq)
        q0[3] = -1.57;                     // joint4
        q0[5] =  1.57;                     // joint6
        env.setQpos(q0);
        env.forward();
        // 위치 서보가 이 자세를 유지하도록 ctrl 도 초기각으로 설정
        Eigen::VectorXd u0 = env.ctrl();
        u0.head(ARM) = q0.head(ARM);
        env.setCtrl(u0);
    }
    env.step(1000);  // 안정화

    // 목표 자세: 위치는 절대좌표 (0.5, 0, 0.5), 자세는 초기 EE 자세 유지
    Eigen::Vector3d p_des(0.5, 0.0, 0.5);
    Eigen::Matrix3d R_des = env.bodyMat(ee_id);

    // CLIK 목표 관절각 초기화 = 현재 팔 관절각
    Eigen::VectorXd q_des = env.qpos().head(ARM);
    Eigen::VectorXd e_int = Eigen::VectorXd::Zero(6);  // 위치/자세 오차 적분항

    bool control_started = false;   // 키 입력 전엔 초기 자세 유지
    long step_count = 0;
    std::cout << "[준비됨] 무조코 창에서 아무 키나 누르면 목표 위치로 제어를 시작합니다." << std::endl;

    // 메인 루프
    while (!env.shouldClose()) {
        env.stepStart();

        // 키 입력을 받으면 제어 시작 (래치)
        if (!control_started && env.keyPressed()) {
            control_started = true;
            std::cout << "[시작] 목표 위치로 제어 시작." << std::endl;
        }

        if (control_started) {
            // ---- FK: 현재 EE 위치/자세 ----
            Eigen::Vector3d p_cur = env.bodyPos(ee_id);
            Eigen::Matrix3d R_cur = env.bodyMat(ee_id);

            // ---- 로그: 목표 / 현재 위치 (500 스텝마다) ----
            if (step_count % 500 == 0) {
                std::printf("target=(%.4f, %.4f, %.4f)  current=(%.4f, %.4f, %.4f)  err=%.4f\n",
                            p_des.x(), p_des.y(), p_des.z(),
                            p_cur.x(), p_cur.y(), p_cur.z(), (p_des - p_cur).norm());
            }
            ++step_count;

            // ---- 위치 오차 + 자세 오차(쿼터니언 → 오일러각) ----
            Quaternion q_cur = RotationMatrxToQuaternion(R_cur);
            Quaternion q_tar = RotationMatrxToQuaternion(R_des);
            Quaternion q_err = q_tar * q_cur.conjugate();       // 현재→목표 회전
            Vector3d   ori_err = QuaternionToEulerAngle(q_err); // 각도 오차

            Eigen::VectorXd e(6);
            e.head(3) = p_des - p_cur;   // 위치 오차
            e.tail(3) = ori_err;         // 자세 오차

            // ---- 자코비안 + 속도 오차 (목표 EE 속도 0 → ė = -(J·q̇)) ----
            Eigen::MatrixXd J = env.jacobianBody(ee_id).leftCols(ARM);  // 6 x 7
            Eigen::VectorXd e_dot = -(J * env.qvel().head(ARM));        // 6

            // ---- 위치/자세 오차 적분 누적 ----
            e_int += e * dt;

            // ---- task-space PID: u = Kp·e + Kd·ė + Ki·∫e (자세항은 우선 off) ----
            Eigen::VectorXd u(6);
            u.setZero();
            u.head(3) = Kp_p * e.head(3) + Kd_p * e_dot.head(3) + Ki_p * e_int.head(3);
            // u.tail(3) = Kp_o * e.tail(3) + Kd_o * e_dot.tail(3) + Ki_o * e_int.tail(3);

            // ---- DLS: dq = Jᵀ(JJᵀ + λ²I)⁻¹·u → 목표 관절각 적분 ----
            Eigen::MatrixXd A = J * J.transpose() + lambda * lambda * Eigen::MatrixXd::Identity(6, 6);
            Eigen::VectorXd dq = J.transpose() * A.ldlt().solve(u);  // 7
            q_des += dq * dt;

            // ---- 제어 입력 주입 (위치 액추에이터: ctrl = 목표각, 그리퍼는 유지) ----
            Eigen::VectorXd ctrl = env.ctrl();
            ctrl.head(ARM) = q_des;
            env.setCtrl(ctrl);
        }
        // control_started 이전: 별도 명령 없음 → 위치 서보가 초기 자세 유지

        env.stepEnd();
        env.render();
    }

    return 0;
}