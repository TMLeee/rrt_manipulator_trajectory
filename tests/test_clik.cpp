// headless CLIK 수렴 테스트 (GLFW 없음). 프레임워크로 (0.5,0,0.5) 도달 검증.
#include <cstdio>
#include <memory>
#include <vector>
#include <mujoco/mujoco.h>
#include <Eigen/Dense>

#include "MujocoRobotModel.h"
#include "Controller.h"
#include "FrameTask.h"
#include "JointSpaceTask.h"
#include "ControlLaw.h"

static int run(bool with_posture) {
    char err[1024] = {0};
    mjModel* m = mj_loadXML("/home/user1/mani_ws/model/mujoco_menagerie/franka_emika_panda/scene.xml",
                            nullptr, err, sizeof(err));
    if (!m) { std::printf("load fail: %s\n", err); return 1; }
    mjData* d = mj_makeData(m);
    const int ARM = 7;

    // 초기 자세 joint4=-1.57, joint6=1.57 + ctrl hold + 안정화
    d->qpos[3] = -1.57; d->qpos[5] = 1.57; mj_forward(m, d);
    for (int i = 0; i < ARM; ++i) d->ctrl[i] = d->qpos[i];
    for (int i = 0; i < 1000; ++i) { mj_step1(m, d); mj_step2(m, d); }

    auto model = std::make_shared<MujocoRobotModel>(m, d, ARM);
    int ee = model->frameId("hand");
    model->setState(model->qpos(), model->qvel());
    Eigen::Matrix3d R_des = model->frameRot(ee);

    ClikController controller(model, 0.03);
    auto ee_task = std::make_shared<FrameTask>(
        "ee", ee, std::make_unique<PIDControlLaw>(3.0, 0.1, 0.0),
        std::array<bool, 6>{true, true, true, false, false, false});
    Eigen::Vector3d goal(0.5, 0.0, 0.5);
    ee_task->setTargetPos(goal);
    ee_task->setTargetRot(R_des);
    controller.addTask(ee_task);

    if (with_posture) {
        std::vector<int> arm = {0,1,2,3,4,5,6};
        auto posture = std::make_shared<JointSpaceTask>(
            "posture", std::make_unique<PControlLaw>(0.5), arm, model->qpos().head(ARM));
        controller.addTask(posture);
    }
    controller.start();

    for (int k = 0; k < 8000; ++k) {
        mj_step1(m, d);
        Eigen::VectorXd q_arm = controller.update(m->opt.timestep);
        for (int i = 0; i < ARM; ++i) d->ctrl[i] = q_arm[i];
        mj_step2(m, d);
        if (with_posture && k % 2000 == 0)
            std::printf("      k=%5d err=%.5f\n", k, (model->framePos(ee) - goal).norm());
    }
    double e = (model->framePos(ee) - goal).norm();
    Eigen::Vector3d p = model->framePos(ee);
    std::printf("  [%s] final pos=(%.4f,%.4f,%.4f)  err=%.5f  -> %s\n",
                with_posture ? "EE+posture" : "EE only",
                p.x(), p.y(), p.z(), e, (e < 1e-2) ? "PASS" : "FAIL");
    mj_deleteData(d); mj_deleteModel(m);
    return (e < 1e-2) ? 0 : 1;
}

int main() {
    std::printf("=== CLIK headless 수렴 테스트 (목표 0.5,0,0.5) ===\n");
    int rc = 0;
    rc |= run(false);   // EE 위치 task 단독 (회귀 기준)
    rc |= run(true);    // + null-space posture
    std::printf("%s\n", rc == 0 ? "ALL PASS" : "SOME FAIL");
    return rc;
}
