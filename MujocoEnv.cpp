#include "MujocoEnv.h"

#include <stdexcept>
#include <string>

MujocoEnv::MujocoEnv(const std::string& model_path, int width, int height,
                     const std::string& title) {
    // 모델 로드
    char error[1024] = {0};
    m_ = mj_loadXML(model_path.c_str(), nullptr, error, sizeof(error));
    if (!m_)
        throw std::runtime_error(std::string("MuJoCo load error: ") + error);
    d_ = mj_makeData(m_);

    // GLFW / OpenGL 컨텍스트 생성
    if (!glfwInit())
        throw std::runtime_error("Could not initialize GLFW");
    glfwWindowHint(GLFW_DOUBLEBUFFER, GLFW_TRUE);
    window_ = glfwCreateWindow(width, height, title.c_str(), nullptr, nullptr);
    if (!window_) {
        glfwTerminate();
        throw std::runtime_error("Could not create GLFW window");
    }
    glfwMakeContextCurrent(window_);
    glfwSwapInterval(1);  // vsync

    // 콜백 등록 + 인스턴스 포인터 연결
    glfwSetWindowUserPointer(window_, this);
    glfwSetKeyCallback(window_, keyboardCb);
    glfwSetMouseButtonCallback(window_, mouseButtonCb);
    glfwSetCursorPosCallback(window_, cursorPosCb);
    glfwSetScrollCallback(window_, scrollCb);

    // 시각화 초기화
    mjv_defaultCamera(&cam_);
    mjv_defaultOption(&opt_);
    mjv_defaultScene(&scn_);
    mjr_defaultContext(&con_);
    mjv_makeScene(m_, &scn_, 2000);
    mjr_makeContext(m_, &con_, mjFONTSCALE_150);
}

MujocoEnv::~MujocoEnv() {
    // 생성 역순으로 정리
    if (d_) mj_deleteData(d_);
    if (m_) mj_deleteModel(m_);
    mjv_freeScene(&scn_);
    mjr_freeContext(&con_);
    if (window_) glfwDestroyWindow(window_);
    glfwTerminate();
}

bool MujocoEnv::shouldClose() const {
    return glfwWindowShouldClose(window_);
}

void MujocoEnv::stepStart() { mj_step1(m_, d_); }
void MujocoEnv::stepEnd()   { mj_step2(m_, d_); }

void MujocoEnv::step(int n) {
    for (int i = 0; i < n; ++i) {
        mj_step1(m_, d_);
        mj_step2(m_, d_);
    }
}

void MujocoEnv::render() {
    // 프레임버퍼 크기에 맞춰 뷰포트 설정
    int width, height;
    glfwGetFramebufferSize(window_, &width, &height);
    mjrRect viewport = {0, 0, width, height};

    // 장면 갱신 → 렌더 → 스왑 → 이벤트
    mjv_updateScene(m_, d_, &opt_, nullptr, &cam_, mjCAT_ALL, &scn_);
    mjr_render(viewport, &scn_, &con_);
    glfwSwapBuffers(window_);
    glfwPollEvents();
}

void MujocoEnv::setCamera(double azimuth, double elevation, double distance,
                          double look_x, double look_y, double look_z) {
    cam_.azimuth = azimuth;
    cam_.elevation = elevation;
    cam_.distance = distance;
    cam_.lookat[0] = look_x;
    cam_.lookat[1] = look_y;
    cam_.lookat[2] = look_z;
}

int MujocoEnv::bodyId(const std::string& name) const {
    return mj_name2id(m_, mjOBJ_BODY, name.c_str());
}
int MujocoEnv::siteId(const std::string& name) const {
    return mj_name2id(m_, mjOBJ_SITE, name.c_str());
}

Eigen::VectorXd MujocoEnv::qpos() const {
    return Eigen::Map<const Eigen::VectorXd>(d_->qpos, m_->nq);
}
Eigen::VectorXd MujocoEnv::qvel() const {
    return Eigen::Map<const Eigen::VectorXd>(d_->qvel, m_->nv);
}
Eigen::VectorXd MujocoEnv::actuatorForce() const {
    return Eigen::Map<const Eigen::VectorXd>(d_->actuator_force, m_->nu);
}
Eigen::VectorXd MujocoEnv::jointTorque() const {
    return Eigen::Map<const Eigen::VectorXd>(d_->qfrc_actuator, m_->nv);
}
Eigen::VectorXd MujocoEnv::ctrl() const {
    return Eigen::Map<const Eigen::VectorXd>(d_->ctrl, m_->nu);
}

void MujocoEnv::setQpos(const Eigen::VectorXd& q) {
    Eigen::Map<Eigen::VectorXd>(d_->qpos, m_->nq) = q;
}
void MujocoEnv::setQvel(const Eigen::VectorXd& v) {
    Eigen::Map<Eigen::VectorXd>(d_->qvel, m_->nv) = v;
}
void MujocoEnv::setCtrl(const Eigen::VectorXd& u) {
    Eigen::Map<Eigen::VectorXd>(d_->ctrl, m_->nu) = u;
}

void MujocoEnv::forward() { mj_forward(m_, d_); }

Eigen::Vector3d MujocoEnv::bodyPos(int body_id) const {
    return Eigen::Map<const Eigen::Vector3d>(d_->xpos + 3 * body_id);
}
Eigen::Matrix3d MujocoEnv::bodyMat(int body_id) const {
    // MuJoCo xmat 은 row-major 9개 → Eigen(기본 col-major) 로 전치 매핑
    return Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(d_->xmat + 9 * body_id);
}
Eigen::Vector3d MujocoEnv::sitePos(int site_id) const {
    return Eigen::Map<const Eigen::Vector3d>(d_->site_xpos + 3 * site_id);
}
Eigen::Matrix3d MujocoEnv::siteMat(int site_id) const {
    return Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(d_->site_xmat + 9 * site_id);
}

Eigen::MatrixXd MujocoEnv::jacobianBody(int body_id) const {
    // jacp(3 x nv) 선형 + jacr(3 x nv) 각 → 6 x nv 결합 (MuJoCo 는 row-major 3 x nv)
    int nv = m_->nv;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> jacp(3, nv), jacr(3, nv);
    mj_jacBody(m_, d_, jacp.data(), jacr.data(), body_id);
    Eigen::MatrixXd J(6, nv);
    J.topRows(3)    = jacp;
    J.bottomRows(3) = jacr;
    return J;
}
Eigen::MatrixXd MujocoEnv::jacobianSite(int site_id) const {
    int nv = m_->nv;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> jacp(3, nv), jacr(3, nv);
    mj_jacSite(m_, d_, jacp.data(), jacr.data(), site_id);
    Eigen::MatrixXd J(6, nv);
    J.topRows(3)    = jacp;
    J.bottomRows(3) = jacr;
    return J;
}

void MujocoEnv::keyboardCb(GLFWwindow* w, int key, int /*scancode*/, int act, int /*mods*/) {
    auto* self = static_cast<MujocoEnv*>(glfwGetWindowUserPointer(w));
    if (act != GLFW_PRESS) return;
    if (key == GLFW_KEY_ESCAPE)
        glfwSetWindowShouldClose(w, GLFW_TRUE);  // ESC 로 창 종료
    else
        self->key_pressed_ = true;               // 그 외 키 → 시작 트리거
}

void MujocoEnv::mouseButtonCb(GLFWwindow* w, int /*button*/, int /*act*/, int /*mods*/) {
    auto* self = static_cast<MujocoEnv*>(glfwGetWindowUserPointer(w));
    // 버튼 상태 갱신 + 현재 커서 위치 저장
    self->btn_left_   = glfwGetMouseButton(w, GLFW_MOUSE_BUTTON_LEFT)   == GLFW_PRESS;
    self->btn_middle_ = glfwGetMouseButton(w, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS;
    self->btn_right_  = glfwGetMouseButton(w, GLFW_MOUSE_BUTTON_RIGHT)  == GLFW_PRESS;
    glfwGetCursorPos(w, &self->last_x_, &self->last_y_);
}

void MujocoEnv::cursorPosCb(GLFWwindow* w, double xpos, double ypos) {
    auto* self = static_cast<MujocoEnv*>(glfwGetWindowUserPointer(w));
    // 눌린 버튼 없으면 무시
    if (!self->btn_left_ && !self->btn_middle_ && !self->btn_right_) {
        self->last_x_ = xpos;
        self->last_y_ = ypos;
        return;
    }

    // 이동량 정규화
    int width, height;
    glfwGetWindowSize(w, &width, &height);
    double dx = (xpos - self->last_x_) / height;
    double dy = (ypos - self->last_y_) / height;
    self->last_x_ = xpos;
    self->last_y_ = ypos;

    // 버튼별 카메라 조작 결정 (좌:회전, 우:이동, 중:수직이동)
    bool shift = (glfwGetKey(w, GLFW_KEY_LEFT_SHIFT)  == GLFW_PRESS ||
                  glfwGetKey(w, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);
    mjtMouse action;
    if (self->btn_right_)
        action = shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    else if (self->btn_left_)
        action = shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    else
        action = mjMOUSE_ZOOM;

    mjv_moveCamera(self->m_, action, dx, dy, &self->scn_, &self->cam_);
}

void MujocoEnv::scrollCb(GLFWwindow* w, double /*xoffset*/, double yoffset) {
    auto* self = static_cast<MujocoEnv*>(glfwGetWindowUserPointer(w));
    // 휠로 줌
    mjv_moveCamera(self->m_, mjMOUSE_ZOOM, 0.0, -0.05 * yoffset, &self->scn_, &self->cam_);
}
