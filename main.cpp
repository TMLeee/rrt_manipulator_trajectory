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

    // 우선순위 0: EE 위치 task (위치전용 마스크), PID(Ki=0)
    auto ee_task = std::make_shared<FrameTask>(
        "ee_pose", ee,
        std::make_unique<PIDControlLaw>(/*Kp=*/3.0, /*Kd=*/0.1, /*Ki=*/0.0),
        std::array<bool, 6>{true, true, true, false, false, false});
    ee_task->setTargetPos(Eigen::Vector3d(0.5, 0.0, 0.5));
    ee_task->setTargetRot(R_des);
    controller.addTask(ee_task);

    // 우선순위 1 (null-space): 자세(posture) 유지 — 초기 팔 관절각으로 정규화
    Eigen::VectorXd q_arm0 = env.qpos().head(ARM);
    std::vector<int> all_arm = {0, 1, 2, 3, 4, 5, 6};
    auto posture = std::make_shared<JointSpaceTask>(
        "posture", std::make_unique<PControlLaw>(0.5), all_arm, q_arm0);
    controller.addTask(posture);

    bool control_started = false;
    long step_count = 0;
    std::cout << "[준비됨] 무조코 창에서 아무 키나 누르면 목표 위치로 제어를 시작합니다." << std::endl;

    // 메인 루프
    while (!env.shouldClose()) {
        env.stepStart();

        if (!control_started && env.keyPressed()) {
            control_started = true;
            controller.start();
            std::cout << "[시작] 목표 위치로 제어 시작." << std::endl;
        }

        if (control_started) {
            Eigen::VectorXd q_arm = controller.update(env.timestep());
            Eigen::VectorXd ctrl = env.ctrl();
            ctrl.head(ARM) = q_arm;
            env.setCtrl(ctrl);

            if (step_count % 500 == 0) {
                Eigen::Vector3d p = model->framePos(ee);
                Eigen::Vector3d p_des(0.5, 0.0, 0.5);
                std::printf("target=(%.4f, %.4f, %.4f)  current=(%.4f, %.4f, %.4f)  err=%.4f\n",
                            p_des.x(), p_des.y(), p_des.z(), p.x(), p.y(), p.z(), (p_des - p).norm());
            }
            ++step_count;
        }

        env.stepEnd();
        env.render();
    }

    return 0;
}
