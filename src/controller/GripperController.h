#pragma once

#include <string>
#include <mujoco/mujoco.h>

class MujocoEnv;  // 전방선언

// 1-DOF 그리퍼 컨트롤러 (팔 모션 컨트롤러와 독립).
// 목표 개방률 + 속도 제한 + 전류(힘) 제한 형태. 그리퍼 액추에이터 ctrl 인덱스만 제어.
class GripperController {
public:
    GripperController(mjModel* m, mjData* d, const std::string& actuator = "actuator8");
    explicit GripperController(MujocoEnv& env, const std::string& actuator = "actuator8");

    // open_frac: 0=닫힘, 1=열림 / vel_limit: 개방률/초 / current_limit: 힘(전류 상당) 한계
    void setTarget(double open_frac, double vel_limit, double current_limit);

    // 한 틱: 속도 제한 램프 + 전류(힘) 제한 적용하여 그리퍼 ctrl 갱신
    void update(double dt);

    double position() const { return cmd_; }  // 현재 명령 개방률 [0,1]
    double force() const;                      // 현재 액추에이터 힘
    bool   valid() const { return idx_ >= 0; }

private:
    void init(const std::string& actuator);

    mjModel* m_ = nullptr;  // 비소유
    mjData*  d_ = nullptr;  // 비소유
    int    idx_ = -1;       // 그리퍼 액추에이터(=ctrl) 인덱스
    double lo_ = 0.0, hi_ = 1.0;  // 액추에이터 ctrlrange
    double target_ = 0.0;   // 목표 개방률
    double vel_ = 1.0;      // 속도 제한 (개방률/초)
    double ilim_ = 1e9;     // 전류(힘) 제한
    double cmd_ = 0.0;      // 현재 명령 개방률
};
