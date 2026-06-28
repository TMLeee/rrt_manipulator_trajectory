#pragma once

#include <string>
#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include <Eigen/Dense>

// MuJoCo 모델 로드 + 시뮬레이션 + GLFW 렌더링을 묶은 환경 클래스
class MujocoEnv {
public:
    // 모델 XML 경로로 환경 생성 (실패 시 std::runtime_error)
    MujocoEnv(const std::string& model_path, int width = 1200, int height = 900,
              const std::string& title = "MuJoCo Viewer");
    ~MujocoEnv();

    MujocoEnv(const MujocoEnv&) = delete;
    MujocoEnv& operator=(const MujocoEnv&) = delete;

    // 창 종료 요청 여부
    bool shouldClose() const;
    // 창 종료 요청 (예: main 에서 ESC 처리 시)
    void requestClose();

    // 키 폴링 — 키 처리 로직은 main 에서 수행 (GLFW 키 코드)
    bool keyDown(int glfw_key) const;

    // 시뮬레이션 한 스텝의 전반부 / 후반부 (사이에 제어 입력 주입)
    void stepStart();
    void stepEnd();
    // 전체 스텝을 n회 진행
    void step(int n = 1);

    // 현재 상태를 화면에 렌더링 (장면 갱신 → 렌더 → 버퍼 스왑 → 이벤트 처리)
    void render();

    // 카메라 위치 설정 (방위각, 고도, 거리, 응시점)
    void setCamera(double azimuth, double elevation, double distance,
                   double look_x, double look_y, double look_z);

    // 내부 핸들 접근자
    mjModel* model() const { return m_; }
    mjData*  data()  const { return d_; }
    double   timestep() const { return m_->opt.timestep; }

    // ---- 차원 ----
    int nq() const { return m_->nq; }  // 위치 좌표 수
    int nv() const { return m_->nv; }  // 자유도(속도 좌표) 수
    int nu() const { return m_->nu; }  // 액추에이터 수

    // ---- 이름 → id ----
    int bodyId(const std::string& name) const;  // 없으면 -1
    int siteId(const std::string& name) const;  // 없으면 -1

    // ---- 현재 상태 (읽기) ----
    Eigen::VectorXd qpos() const;           // 현재 관절각 (nq)
    Eigen::VectorXd qvel() const;           // 현재 관절속도 (nv)
    Eigen::VectorXd actuatorForce() const;  // 현재 액추에이터 힘/토크 (nu)
    Eigen::VectorXd jointTorque() const;    // 관절공간 액추에이터 토크 qfrc_actuator (nv)
    Eigen::VectorXd ctrl() const;           // 현재 제어 입력 (nu)

    // ---- 목표 상태 (쓰기) ----
    void setQpos(const Eigen::VectorXd& q);  // 관절각 직접 설정 (리셋/IK 결과 주입)
    void setQvel(const Eigen::VectorXd& v);  // 관절속도 직접 설정
    void setCtrl(const Eigen::VectorXd& u);  // 목표 제어 입력 (액추에이터 종류에 따라 목표각 또는 목표토크)

    // ---- 순기구학 (FK) ----
    void forward();                          // 파생량(xpos, 자코비안 등) 갱신: mj_forward
    Eigen::Vector3d bodyPos(int body_id) const;   // body 월드 위치
    Eigen::Matrix3d bodyMat(int body_id) const;   // body 월드 회전(3x3)
    Eigen::Vector3d sitePos(int site_id) const;   // site 월드 위치
    Eigen::Matrix3d siteMat(int site_id) const;   // site 월드 회전(3x3)

    // ---- 6자유도 자코비안 (상단 3행: 선형 jacp, 하단 3행: 각 jacr, 6 x nv) ----
    Eigen::MatrixXd jacobianBody(int body_id) const;
    Eigen::MatrixXd jacobianSite(int site_id) const;

private:
    // GLFW 마우스 콜백 (window user pointer 로 인스턴스 접근). 키는 main 에서 폴링.
    static void mouseButtonCb(GLFWwindow* w, int button, int act, int mods);
    static void cursorPosCb(GLFWwindow* w, double xpos, double ypos);
    static void scrollCb(GLFWwindow* w, double xoffset, double yoffset);

    // MuJoCo 데이터
    mjModel* m_ = nullptr;
    mjData*  d_ = nullptr;

    // 시각화 구조체
    mjvCamera  cam_;
    mjvOption  opt_;
    mjvScene   scn_;
    mjrContext con_;
    GLFWwindow* window_ = nullptr;

    // 마우스 카메라 조작 상태
    bool   btn_left_ = false, btn_middle_ = false, btn_right_ = false;
    double last_x_ = 0.0, last_y_ = 0.0;
};
