// Окно просмотра: сцены движка в реальном времени на OpenGL.
//
//   viewer [--scene N] [--frames N] [--screenshot файл.ppm] [--record префикс]
//
// Управление — на панели в окне и клавишами: 1–9 — сцена, пробел — пауза, → — один шаг,
// R — сцена заново, T — состояние мира в консоль, левая кнопка мыши на теле — взять и тянуть его,
// мимо тел — вращение камеры, правая кнопка — вращение камеры, средняя — сдвиг сцены,
// колесо — приближение, Esc — выход.
// С ключом --frames окно само закрывается после N кадров; --screenshot сохраняет последний,
// --record — каждый (префикс-0001.ppm, …): из них собираются анимации для документации.

#include "GlRenderer.h"
#include "Panel.h"

#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl2.h>

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <string>

using namespace kp;

namespace {

Viewer& viewerOf(GLFWwindow* window) { return *static_cast<Viewer*>(glfwGetWindowUserPointer(window)); }

void onKey(GLFWwindow* window, int key, int, int action, int) {
    if (action == GLFW_RELEASE || ImGui::GetIO().WantCaptureKeyboard) return;
    Viewer& viewer = viewerOf(window);
    if (key >= GLFW_KEY_1 && key < GLFW_KEY_1 + int(scenes().size())) viewer.load(size_t(key - GLFW_KEY_1));
    if (key == GLFW_KEY_R) viewer.load(viewer.scene);
    if (key == GLFW_KEY_SPACE) viewer.paused = !viewer.paused;
    if (key == GLFW_KEY_RIGHT) viewer.stepOnce = true;
    if (key == GLFW_KEY_T) std::fputs(debugText(viewer.world).c_str(), stdout);
    if (key == GLFW_KEY_ESCAPE) glfwSetWindowShouldClose(window, GLFW_TRUE);
}

void onScroll(GLFWwindow* window, double, double amount) {
    if (ImGui::GetIO().WantCaptureMouse) return;
    Camera& camera = viewerOf(window).camera;
    camera.distance = std::clamp(camera.distance * (1 - 0.1 * amount), 0.2, 200.0);
}

// Мышь вне панели. Левая кнопка, нажатая на теле, берёт его: луч от курсора находит точку тела,
// и тело тянется за курсором, пока кнопка зажата. Нажатая мимо тел — вращает камеру вокруг точки,
// на которую та смотрит; правая кнопка вращает камеру всегда.
void pointer(GLFWwindow* window, Viewer& viewer, int width, int height) {
    static double previousX = 0, previousY = 0;
    static bool wasDown = false;
    double x = 0, y = 0;
    glfwGetCursorPos(window, &x, &y);
    const bool down = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS && (wasDown || !ImGui::GetIO().WantCaptureMouse);
    const bool turning = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS && !ImGui::GetIO().WantCaptureMouse;
    const Vector3 direction = viewer.camera.ray(x, y, width, height);
    if (down && !wasDown) viewer.grabAt(direction);
    if (!down) viewer.release();
    viewer.dragTo(direction);
    if (turning || (down && viewer.grab.body < 0)) {
        viewer.camera.yaw -= 0.005 * (x - previousX);
        viewer.camera.pitch = std::clamp(viewer.camera.pitch + 0.005 * (y - previousY), -1.5, 1.5);
    }
    // Средняя кнопка сдвигает точку, на которую смотрит камера, в плоскости экрана: сцена идёт за курсором.
    if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS && !ImGui::GetIO().WantCaptureMouse) {
        const Camera::Axes axes = viewer.camera.axes();
        const double metresPerPixel = 2 * viewer.camera.distance * Camera::kTanHalfFov / height;
        viewer.camera.target = viewer.camera.target + Real(-metresPerPixel * (x - previousX)) * axes.right + Real(metresPerPixel * (y - previousY)) * axes.up;
    }
    previousX = x, previousY = y, wasDown = down;
}

// Встроенный шрифт ImGui знает только латиницу; подписи панели русские, поэтому берётся
// системный шрифт с кириллицей — первый найденный.
void loadFont() {
    for (const char* path : {"/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", "/usr/share/fonts/TTF/DejaVuSans.ttf",
                             "/System/Library/Fonts/Supplemental/Arial.ttf", "C:/Windows/Fonts/segoeui.ttf"}) {
        if (std::FILE* file = std::fopen(path, "rb")) {
            std::fclose(file);
            ImGui::GetIO().Fonts->AddFontFromFileTTF(path, 16);
            return;
        }
    }
    std::fprintf(stderr, "шрифт с кириллицей не найден: подписи панели будут нечитаемы\n");
}

void startPanel(GLFWwindow* window) {
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui::GetIO().IniFilename = nullptr; // не оставлять файл настроек рядом с программой
    ImGui::StyleColorsDark();
    loadFont();
    ImGui_ImplGlfw_InitForOpenGL(window, true); // свои обработчики ввода ImGui ставит поверх наших и вызывает их
    ImGui_ImplOpenGL2_Init();
}

void stopPanel() {
    ImGui_ImplOpenGL2_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
}

} // namespace

int main(int argc, char** argv) {
    size_t firstScene = 0;
    int frameLimit = 0;
    std::string screenshot, record;
    for (int i = 1; i + 1 < argc; i += 2) {
        const std::string key = argv[i];
        if (key == "--scene") firstScene = size_t(std::clamp(std::atoi(argv[i + 1]), 1, int(scenes().size())) - 1);
        if (key == "--frames") frameLimit = std::atoi(argv[i + 1]);
        if (key == "--screenshot") screenshot = argv[i + 1];
        if (key == "--record") record = argv[i + 1];
    }

    if (!glfwInit()) return (std::fprintf(stderr, "не удалось запустить GLFW: нет графического экрана?\n"), 1);
    GLFWwindow* window = glfwCreateWindow(1280, 720, "kinetica-physics", nullptr, nullptr);
    if (!window) return (std::fprintf(stderr, "не удалось открыть окно OpenGL\n"), glfwTerminate(), 1);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1); // кадр окна на кадр экрана: при 60 Гц сцена идёт в реальном времени

    Viewer viewer;
    viewer.load(firstScene);
    glfwSetWindowUserPointer(window, &viewer);
    glfwSetKeyCallback(window, onKey);
    glfwSetScrollCallback(window, onScroll);
    startPanel(window);

    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();
        ImGui_ImplOpenGL2_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        int width = 0, height = 0, pixelsWide = 0, pixelsHigh = 0;
        glfwGetWindowSize(window, &width, &height);
        pointer(window, viewer, width, height);
        if (!viewer.paused || viewer.stepOnce) viewer.step();
        viewer.stepOnce = false;

        glfwGetFramebufferSize(window, &pixelsWide, &pixelsHigh);
        const DebugGeometry all = debugGeometry(viewer.world, viewer.show[int(DebugKind::TreeNode)]);
        render(viewer.visible(all), viewer.camera, pixelsWide, pixelsHigh, viewer.fill ? &viewer.world : nullptr);
        drawPanel(viewer, all, width, height);
        ImGui::Render();
        ImGui_ImplOpenGL2_RenderDrawData(ImGui::GetDrawData());

        if (!record.empty()) {
            char name[16];
            std::snprintf(name, sizeof name, "-%04d.ppm", viewer.frame);
            saveScreenshot(record + name, pixelsWide, pixelsHigh);
        }
        if (frameLimit && viewer.frame >= frameLimit) {
            if (!screenshot.empty()) saveScreenshot(screenshot, pixelsWide, pixelsHigh);
            std::printf("сцена %zu, кадр %d: самый долгий шаг %.2f мс, отрезков в шаге не больше %d\n", viewer.scene + 1,
                        viewer.frame, viewer.slowestStep, viewer.mostSegments);
            break;
        }
        glfwSwapBuffers(window);
    }
    stopPanel();
    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}
