#include <iostream>
#include <memory>
#include <vector>
#include <Eigen/Dense>

#include "MujocoEnv.h"
#include "MujocoRobotModel.h"
#include "Controller.h"
#include "FrameTask.h"
#include "JointSpaceTask.h"
#include "ControlLaw.h"
#include "GripperController.h"

int main(void)
{
    const char* model_path = "/home/user1/mani_ws/model/mujoco_menagerie/franka_emika_panda/scene.xml";

    // MuJoCo 환경 생성
    MujocoEnv env(model_path, 1200, 900, "MuJoCo Viewer (C++ from VS Code)");
    env.setCamera(120.0, -15.0, 3.0, 0.5, 0.0, 0.5);

    std::cout << "timestep = " << env.timestep() << std::endl;
    std::cout << "nq=" << env.nq() << " nv=" << env.nv() << " nu=" << env.nu() << std::endl;

    const int ARM = 7;

    // 초기 자세: joint4 = -1.57, joint6 = 1.57
    {
        Eigen::VectorXd q0 = env.qpos();
        q0[3] = -1.57;
        q0[5] =  1.57;
        env.setQpos(q0);
        env.forward();
        Eigen::VectorXd u0 = env.ctrl();
        u0.head(ARM) = q0.head(ARM);
        env.setCtrl(u0);
    }
    env.step(1000);  // 안정화

    // ===== 모션 컨트롤러 프레임워크 구성 =====
    auto model = std::make_shared<MujocoRobotModel>(env, ARM);
    int ee = model->frameId("hand");

    // 안정화 후 현재 EE 자세를 목표 자세로 (자세 유지)
    model->setState(env.qpos(), env.qvel());
    Eigen::Matrix3d R_des = model->frameRot(ee);

    ClikController controller(model, /*lambda=*/0.03);

    // 우선순위 0: EE 위치+자세 task (full pose), PID(Ki=0). 자세는 초기 자세 유지(1/2/3 공통)
    auto ee_task = std::make_shared<FrameTask>(
        "ee_pose", ee,
        std::make_unique<PIDControlLaw>(/*Kp=*/3.0, /*Kd=*/0.1, /*Ki=*/0.0),
        std::array<bool, 6>{true, true, true, true, true, true});
    ee_task->setTargetPos(model->framePos(ee));   // 초기 EE 위치에서 시작 (조깅 기준)
    ee_task->setTargetRot(R_des);   // 초기 위치의 자세값 (모든 목표 공통)
    controller.addTask(ee_task);

    // 우선순위 1 (null-space): 3번 축(joint3, index 2) → 0도
    auto j3_task = std::make_shared<JointSpaceTask>(
        "joint3_zero", std::make_unique<PControlLaw>(0.5),
        std::vector<int>{2}, Eigen::VectorXd::Constant(1, 0.0));
    controller.addTask(j3_task);

    // 그리퍼 컨트롤러 (팔과 독립, ctrl[그리퍼]만 제어)
    GripperController gripper(env);
    const double GRIP_VEL = 0.5, GRIP_CURR = 50.0;  // 속도 제한[1/s], 전류(힘) 제한
    gripper.setTarget(1.0, GRIP_VEL, GRIP_CURR);    // 시작: 열림

    // 숫자키 → 목표 위치 매핑
    const Eigen::Vector3d TARGETS[3] = {
        {0.5,  0.0, 0.5},   // 1
        {0.5,  0.3, 0.5},   // 2
        {0.5, -0.3, 0.5},   // 3
    };
    Eigen::Vector3d target = model->framePos(ee);   // 현재 목표 (초기 EE 위치에서 시작)
    const double JOG_SPEED = 0.3;                   // 조깅 속도 [m/s]
    bool control_started = false;
    bool prev1 = false, prev2 = false, prev3 = false;  // 숫자키 엣지 검출
    bool prevR = false, grip_open = true;              // R 토글 / 그리퍼 상태
    long step_count = 0;
    std::cout << "[준비됨] (ESC 종료)\n"
              << "  목표 프리셋: 1=(0.5,0,0.5)  2=(0.5,0.3,0.5)  3=(0.5,-0.3,0.5)\n"
              << "  조깅: W/S=전/후(±x)  A/D=좌/우(±y)  Q/Z=상/하(±z)\n"
              << "  그리퍼: R=열기/닫기 토글" << std::endl;

    // 메인 루프
    while (!env.shouldClose()) {
        env.stepStart();

        // ---- 키 처리 (main 에서 폴링 + 엣지검출) ----
        if (env.keyDown(GLFW_KEY_ESCAPE)) env.requestClose();
        bool k1 = env.keyDown(GLFW_KEY_1) || env.keyDown(GLFW_KEY_KP_1);
        bool k2 = env.keyDown(GLFW_KEY_2) || env.keyDown(GLFW_KEY_KP_2);
        bool k3 = env.keyDown(GLFW_KEY_3) || env.keyDown(GLFW_KEY_KP_3);
        int picked = 0;
        if (k1 && !prev1) picked = 1;
        else if (k2 && !prev2) picked = 2;
        else if (k3 && !prev3) picked = 3;
        prev1 = k1; prev2 = k2; prev3 = k3;

        if (picked) {
            target = TARGETS[picked - 1];
            ee_task->setTargetPos(target);
            if (!control_started) { control_started = true; controller.start(); }
            std::printf("[목표 %d] (%.3f, %.3f, %.3f)\n", picked, target.x(), target.y(), target.z());
        }

        // ---- 조깅: WSAD(전후좌우) + QZ(상하) → 목표를 증분 이동 ----
        Eigen::Vector3d jog = Eigen::Vector3d::Zero();
        if (env.keyDown(GLFW_KEY_W)) jog.x() += 1;   // 전 (+x)
        if (env.keyDown(GLFW_KEY_S)) jog.x() -= 1;   // 후 (-x)
        if (env.keyDown(GLFW_KEY_A)) jog.y() += 1;   // 좌 (+y)
        if (env.keyDown(GLFW_KEY_D)) jog.y() -= 1;   // 우 (-y)
        if (env.keyDown(GLFW_KEY_Q)) jog.z() += 1;   // 상 (+z)
        if (env.keyDown(GLFW_KEY_Z)) jog.z() -= 1;   // 하 (-z)
        if (!jog.isZero()) {
            if (!control_started) { control_started = true; controller.start(); }
            target += jog * JOG_SPEED * env.timestep();
            ee_task->setTargetPos(target);
        }

        // ---- 그리퍼: R 로 열기/닫기 토글 (엣지검출) ----
        bool kr = env.keyDown(GLFW_KEY_R);
        if (kr && !prevR) {
            grip_open = !grip_open;
            gripper.setTarget(grip_open ? 1.0 : 0.0, GRIP_VEL, GRIP_CURR);
            std::printf("[그리퍼] %s\n", grip_open ? "열기" : "닫기");
        }
        prevR = kr;

        if (control_started) {
            Eigen::VectorXd q_arm = controller.update(env.timestep());
            Eigen::VectorXd ctrl = env.ctrl();
            ctrl.head(ARM) = q_arm;
            env.setCtrl(ctrl);

            if (step_count % 500 == 0) {
                Eigen::Vector3d p = model->framePos(ee);
                std::printf("target=(%.4f, %.4f, %.4f)  current=(%.4f, %.4f, %.4f)  err=%.4f  grip=%.2f f=%.1f\n",
                            target.x(), target.y(), target.z(), p.x(), p.y(), p.z(), (target - p).norm(),
                            gripper.position(), gripper.force());
            }
            ++step_count;
        }

        gripper.update(env.timestep());  // 팔과 독립 갱신 (ctrl[그리퍼])

        env.stepEnd();
        env.render();
    }

    return 0;
}
