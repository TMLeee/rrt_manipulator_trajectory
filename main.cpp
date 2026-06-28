#include <iostream>
#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include <rbdl/rbdl.h>
#include <rbdl/addons/urdfreader/urdfreader.h>
#include <Eigen/Dense>

using namespace std;
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

// 전역 MuJoCo 데이터
mjModel* m = nullptr;
mjData* d = nullptr;

// 시각화용 구조체
mjvCamera cam;
mjvOption opt;
mjvScene scn;
mjrContext con;


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


// 콜백: 키보드 입력 처리
void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods)
{
    if (act == GLFW_PRESS && key == GLFW_KEY_ESCAPE)
    {
        glfwSetWindowShouldClose(window, GLFW_TRUE);
    }
}

// 콜백: 마우스 & 휠
void mouse_button(GLFWwindow* window, int button, int act, int mods)
{
    // 여기에 카메라 제어 로직 추가 가능 (간단 예제라 생략)
}

void scroll(GLFWwindow* window, double xoffset, double yoffset)
{
    // 카메라 줌인/아웃 등 (생략 가능)
}

uint32_t guiEnRot;
int main(void)
{
    // ------------------------
    // 1. MuJoCo 초기화
    // ------------------------
    char error[1024] = {0};

    // 모델 로드 (경로는 실행 위치 기준)
    const char* model_path = "/home/user1/mani_ws/model/mujoco_menagerie/franka_emika_panda/scene.xml";
    m = mj_loadXML(model_path, nullptr, error, sizeof(error));
    if (!m)
    {
        std::printf("MuJoCo load error: %s\n", error);
        return 1;
    }

    d = mj_makeData(m);

    // ------------------------
    // 2. GLFW / OpenGL 초기화
    // ------------------------
    if (!glfwInit())
    {
        std::printf("Could not initialize GLFW\n");
        return 1;
    }

    // OpenGL context 생성
    glfwWindowHint(GLFW_DOUBLEBUFFER, GLFW_TRUE);
    GLFWwindow* window = glfwCreateWindow(1200, 900, "MuJoCo Viewer (C++ from VS Code)", nullptr, nullptr);
    if (!window)
    {
        std::printf("Could not create GLFW window\n");
        glfwTerminate();
        return 1;
    }

    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);  // vsync

    // 콜백 등록
    glfwSetKeyCallback(window, keyboard);
    glfwSetMouseButtonCallback(window, mouse_button);
    glfwSetScrollCallback(window, scroll);

    // ------------------------
    // 3. MuJoCo 시각화 초기화
    // ------------------------
    // 기본 카메라, 옵션
    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);

    // scene 메모리 할당
    mjv_makeScene(m, &scn, 2000);
    // OpenGL context와 연결
    mjr_makeContext(m, &con, mjFONTSCALE_150);

    // 카메라를 모델에 맞게 초기화
    cam.azimuth = 120.0f;
    cam.elevation = -15.0f;
    cam.distance = 3.0f;
    cam.lookat[0] = 0.5f;
    cam.lookat[1] = 0.0f;
    cam.lookat[2] = 0.5f;

    std::cout << "timestep = " << m->opt.timestep << std::endl;

    // Simulation 1
    for(uint32_t i=0; i<1000; ++i) {
        mj_step1(m, d);
        mj_step2(m, d);
    }

    // ------------------------
    // 4. 메인 루프
    // ------------------------
    while (!glfwWindowShouldClose(window)) {   

        // Simulation 1
        mj_step1(m, d);
        
        // Calculate FK //

        // Calculate IK //

        // Simulation 2
        mj_step2(m, d);

        // 윈도우 크기 확인
        int width, height;
        glfwGetFramebufferSize(window, &width, &height);

        // viewport 설정
        mjrRect viewport = {0, 0, width, height};

        // 장면 업데이트 (카메라, 조명, 지오메트리 등)
        mjv_updateScene(m, d, &opt, nullptr, &cam, mjCAT_ALL, &scn);

        // 화면 지우고 렌더링
        mjr_render(viewport, &scn, &con);

        // 버퍼 스왑 & 이벤트 처리
        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    // ------------------------
    // 5. 정리(clean up)
    // ------------------------
    mj_deleteData(d);
    mj_deleteModel(m);
    mjv_freeScene(&scn);
    mjr_freeContext(&con);
    glfwDestroyWindow(window);
    glfwTerminate();

    return 0;
}