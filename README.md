# rrt_manipulator_trajectory

MuJoCo 기반 7-DOF 매니퓰레이터(Franka Emika Panda)의 **모션 컨트롤러 프레임워크**.
DLS(Damped Least-Squares) CLIK과 null-space 우선순위 task를 지원하며, 운동학/동역학 소스를
MuJoCo 또는 RBDL 백엔드로 교체 가능하도록 추상화했습니다.

## 주요 기능
- **MuJoCo 시뮬레이션 + 렌더링** 래퍼 (마우스 카메라 조작 포함)
- **RobotModel 추상화**: FK / 6D Jacobian / 동역학(M, h, g)을 MuJoCo·RBDL 두 백엔드로 호환
- **Task 프레임워크**: task space(EE 위치+자세) / joint space 목표를 등록, 각 task에 제어법칙(P/PID) 주입
- **null-space 우선순위 솔버**: 등록 순서 = 우선순위, hierarchical DLS (크리스프 SVD 투영자)
- **키 입력 목표 전환**: 숫자키로 EE 목표 위치 지정

## 의존성
- [MuJoCo](https://mujoco.org/) 3.x (`/opt/mujoco/mujoco`)
- [RBDL](https://github.com/rbdl/rbdl) (+ urdfreader 애드온, `/usr/local`)
- [Eigen3](https://eigen.tuxfamily.org/) (`/usr/include/eigen3`)
- GLFW, GLEW, OpenGL
- g++ (C++17)

## 프로젝트 구조
```
src/
├── main.cpp                    # 엔트리포인트 (환경 구성, 키 처리, 제어 루프)
├── simulator/                  # MuJoCo 시뮬/렌더 래퍼
│   └── MujocoEnv.{h,cpp}
├── model/                      # RobotModel 추상 + 백엔드 (kinematics/dynamics)
│   ├── RobotModel.h
│   ├── MujocoRobotModel.{h,cpp}
│   └── RbdlRobotModel.{h,cpp}  # 구조 완비, URDF 제공 시 검증 (Panda는 URDF 부재)
├── controller/                 # 컨트롤러 / 솔버 / task / 제어법칙
│   ├── Controller.{h,cpp}      # 최상위 base + ClikController (model+task+solver 집약)
│   ├── HierarchicalSolver.{h,cpp}
│   ├── ControlLaw.h            # P / PID 제어법칙 (task에 플러그인)
│   ├── Task.h, FrameTask.{h,cpp}, JointSpaceTask.{h,cpp}
└── util/
    └── MathUtils.h             # 쿼터니언, orientationError, dampedPinv, pinvSvd
tests/
└── test_clik.cpp               # headless 수렴 검증 (GLFW 불필요)
model/                          # MuJoCo Menagerie 에셋 (.gitignore, 별도 외부 repo)
```

## 빌드
VS Code: `Ctrl+Shift+B` → **build (main.cpp)** (기본), 또는 **build (test_clik)**.

직접 빌드:
```bash
g++ -std=c++17 -g -O0 \
  src/main.cpp src/simulator/MujocoEnv.cpp \
  src/model/MujocoRobotModel.cpp src/model/RbdlRobotModel.cpp \
  src/controller/Controller.cpp src/controller/HierarchicalSolver.cpp \
  src/controller/FrameTask.cpp src/controller/JointSpaceTask.cpp \
  -o main \
  -Isrc/simulator -Isrc/model -Isrc/controller -Isrc/util \
  -I/opt/mujoco/mujoco/include -I/usr/local/include -I/usr/include/eigen3 \
  -L/opt/mujoco/mujoco/lib -L/usr/local/lib \
  -Wl,-rpath,/opt/mujoco/mujoco/lib -Wl,-rpath,/usr/local/lib \
  -lmujoco -lrbdl -lrbdl_urdfreader -lglfw -lGLEW -lGL -ldl -lpthread
```

## 실행 / 조작
```bash
./main
```
- 시작 시 초기 자세(joint4 = −1.57, joint6 = 1.57)로 안정화
- **숫자키 1 / 2 / 3** → EE 목표 위치 지정 (첫 입력 시 제어 시작, 자세는 초기 자세 유지)
  | 키 | 목표 위치 (x, y, z) |
  |----|---------------------|
  | 1  | (0.5,  0.0, 0.5) |
  | 2  | (0.5,  0.3, 0.5) |
  | 3  | (0.5, −0.3, 0.5) |
- null-space에서 **joint3 → 0°** 유지
- **마우스**: 좌드래그 회전 / 우드래그 이동 / 휠 줌
- **ESC**: 종료

## 제어 개요
```
키 입력(목표 위치) ─▶ FrameTask.target 갱신
                       │
ClikController.update():
  RobotModel.setState ─▶ 각 Task: error / Jacobian / referenceVelocity(P·PID)
  HierarchicalSolver: 우선순위 0(EE 6D pose) → null-space → 우선순위 1(joint3→0)
       dq = Σ DLS(JᵢNᵢ₋₁)·(ẋᵢ − Jᵢ·dq),  Nᵢ = Nᵢ₋₁ − pinv(JᵢNᵢ₋₁)·(JᵢNᵢ₋₁)
  q_des += dq·dt  ─▶ MuJoCo 위치 액추에이터(ctrl)
```
- task 속도 해석은 DLS(λ=0.03), null-space 투영자는 SVD true pinv(누설 차단)
- 출력은 속도 → 목표 관절각 적분 → 위치 서보

## 테스트 (headless)
```bash
# build (test_clik) 또는 직접 빌드 후
./test_clik
```
EE 단독 / EE+null-space 모두 목표 위치 수렴(오차 → 0)을 검증합니다.

## 비고
- `model/`(MuJoCo Menagerie, ~2.1GB)은 외부 git repo라 `.gitignore` 처리됨. 별도로 받아야 함.
- RBDL 백엔드는 인터페이스/구현이 완비되어 있으나 Panda URDF가 없어 현재 MuJoCo 백엔드로 검증.
  Jacobian 행순서(MuJoCo: linear/angular, RBDL: angular/linear)와 회전 규약은 내부에서 정규화됨.
- 상위 **글로벌 플래너(RRT)·트래젝토리**는 본 모션 컨트롤러 위에 별도 레이어로 얹는 구조
  (Task target 갱신으로 연결).
