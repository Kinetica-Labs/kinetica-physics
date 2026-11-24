// Окно просмотра: сцены движка в реальном времени на OpenGL.
//
//   viewer [--scene N|rail|floating-box|gear-train|…] [--exact 1] [--frames N] [--screenshot файл.ppm] [--record префикс] [--blast кадр] [--measured 1]
//
// Управление — на панели в окне и клавишами: 1–9 — сцена, пробел — пауза, → — один шаг,
// R — сцена заново, T — состояние мира в консоль, левая кнопка мыши на теле — взять и тянуть его,
// мимо тел — вращение камеры, правая кнопка — вращение камеры, средняя — сдвиг сцены,
// колесо — приближение, Esc — выход.
// С ключом --frames окно само закрывается после N кадров; --screenshot сохраняет последний,
// --record — каждый (префикс-0001.ppm, …): из них собираются анимации для документации.
// --blast ставит взрыв в точку, на которую смотрит камера, и пускает его на заданном кадре: проверка
// источников импульсов без мыши. E — пуск всех источников импульсов с панели.

#include "GlRenderer.h"
#include "HullEventsScene.h"
#include "Panel.h"
#include "PlaybackClock.h"

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

struct WindowInput {
    Viewer& viewer;
    double x = 0, y = 0, previousX = 0, previousY = 0;
    bool orbitWithLeft = false;
};

WindowInput& inputOf(GLFWwindow* window) { return *static_cast<WindowInput*>(glfwGetWindowUserPointer(window)); }

VehicleInput driveInput(GLFWwindow* window) {
    if (ImGui::GetIO().WantCaptureKeyboard || !glfwGetWindowAttrib(window, GLFW_FOCUSED)) return {};
    const auto down = [&](int key) { return int(glfwGetKey(window, key) == GLFW_PRESS); };
    return {4 * Real(down(GLFW_KEY_W) - down(GLFW_KEY_S)), Real(.45) * (down(GLFW_KEY_D) - down(GLFW_KEY_A))};
}

void onKey(GLFWwindow* window, int key, int, int action, int) {
    if (action == GLFW_RELEASE || ImGui::GetIO().WantCaptureKeyboard) return;
    Viewer& viewer = inputOf(window).viewer;
    if (key >= GLFW_KEY_1 && key <= GLFW_KEY_9 && size_t(key - GLFW_KEY_1) < scenes().size()) viewer.load(size_t(key - GLFW_KEY_1));
    if (key == GLFW_KEY_R) viewer.load(viewer.scene);
    if (key == GLFW_KEY_SPACE) viewer.paused = !viewer.paused;
    if (key == GLFW_KEY_RIGHT) viewer.stepOnce = true;
    if (key == GLFW_KEY_E) viewer.fireAll();
    if (key == GLFW_KEY_T) std::fputs(debugText(viewer.world).c_str(), stdout);
    if (key == GLFW_KEY_ESCAPE) glfwSetWindowShouldClose(window, GLFW_TRUE);
}

void onScroll(GLFWwindow* window, double, double amount) {
    if (ImGui::GetIO().WantCaptureMouse) return;
    Camera& camera = inputOf(window).viewer.camera;
    camera.distance = std::clamp(camera.distance * (1 - 0.1 * amount), 0.2, 200.0);
}

// GLFW доставляет движения и нажатия по порядку. Тело выбирается по пикселю нажатия,
// даже если до следующей отрисовки курсор успел уйти с куба.
void onCursor(GLFWwindow* window, double x, double y) {
    auto& input = inputOf(window);
    input.x = x;
    input.y = y;
}

Vector3 cursorRay(const WindowInput& input, int width, int height) {
    return input.viewer.camera.ray(input.x - kPanelWidth, input.y, std::max(1, width - kPanelWidth), height);
}

void onMouseButton(GLFWwindow* window, int button, int action, int) {
    if (button != GLFW_MOUSE_BUTTON_LEFT) return;
    auto& input = inputOf(window);
    Viewer& viewer = input.viewer;
    if (action == GLFW_RELEASE) {
        viewer.release();
        input.orbitWithLeft = false;
        return;
    }
    int width, height;
    glfwGetWindowSize(window, &width, &height);
    if (input.x < kPanelWidth || input.x >= width || input.y < 0 || input.y >= height) return;
    if (hull_events::panelContains(viewer.scene, input.x, input.y) || ImGui::IsPopupOpen(nullptr, ImGuiPopupFlags_AnyPopup)) return;
    const Vector3 direction = cursorRay(input, width, height);
    input.orbitWithLeft = hull_events::replaying(viewer.scene) || (!viewer.placeAt(direction) && !viewer.grabAt(direction));
    input.previousX = input.x;
    input.previousY = input.y;
}

void pointer(GLFWwindow* window, int width, int height) {
    if (height <= 0) return;
    auto& input = inputOf(window);
    Viewer& viewer = input.viewer;
    const double dx = input.x - input.previousX, dy = input.y - input.previousY;
    const bool turning = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS && !ImGui::GetIO().WantCaptureMouse;
    viewer.dragTo(cursorRay(input, width, height));
    if (turning || input.orbitWithLeft) {
        viewer.camera.yaw -= 0.005 * dx;
        viewer.camera.pitch = std::clamp(viewer.camera.pitch + 0.005 * dy, -1.5, 1.5);
    }
    // Средняя кнопка сдвигает точку, на которую смотрит камера, в плоскости экрана: сцена идёт за курсором.
    if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS && !ImGui::GetIO().WantCaptureMouse) {
        const Camera::Axes axes = viewer.camera.axes();
        const double metresPerPixel = 2 * viewer.camera.distance * Camera::kTanHalfFov / height;
        viewer.camera.target = viewer.camera.target + Real(-metresPerPixel * dx) * axes.right + Real(metresPerPixel * dy) * axes.up;
    }
    input.previousX = input.x;
    input.previousY = input.y;
}

void installInput(GLFWwindow* window, WindowInput& input) {
    glfwGetCursorPos(window, &input.x, &input.y);
    input.previousX = input.x;
    input.previousY = input.y;
    glfwSetWindowUserPointer(window, &input);
    glfwSetCursorPosCallback(window, onCursor);
    glfwSetMouseButtonCallback(window, onMouseButton);
    glfwSetKeyCallback(window, onKey);
    glfwSetScrollCallback(window, onScroll);
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
    int frameLimit = 0, blastFrame = -1;
    bool exact = false, measured = false;
    std::string screenshot, record;
    for (int i = 1; i + 1 < argc; i += 2) {
        const std::string key = argv[i];
        if (key == "--scene") {
            firstScene = sceneIndex(argv[i + 1]);
            if (firstScene == scenes().size()) {
                std::fprintf(stderr, "неизвестная сцена %s\n", argv[i + 1]);
                return 2;
            }
        }
        if (key == "--hull-data") hull_events::csvPath = argv[i + 1];
        if (key == "--frames") frameLimit = std::atoi(argv[i + 1]);
        if (key == "--exact") exact = std::atoi(argv[i + 1]) != 0;
        if (key == "--screenshot") screenshot = argv[i + 1];
        if (key == "--record") record = argv[i + 1];
        if (key == "--blast") blastFrame = std::atoi(argv[i + 1]);
        if (key == "--measured") measured = std::atoi(argv[i + 1]) != 0; // настройки прибора с замерами: кнопка «Считать как прибор»
    }

    if (!glfwInit()) return (std::fprintf(stderr, "не удалось запустить GLFW: нет графического экрана?\n"), 1);
    GLFWwindow* window = glfwCreateWindow(1480, 820, "kinetica-physics", nullptr, nullptr);
    if (!window) return (std::fprintf(stderr, "не удалось открыть окно OpenGL\n"), glfwTerminate(), 1);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    Viewer viewer;
    viewer.realtime = !exact;
    viewer.load(firstScene);
    if (measured) viewer.measure();
    if (blastFrame >= 0) viewer.impulses.push_back({}), viewer.impulses.back().center = viewer.camera.target;
    WindowInput input{viewer};
    installInput(window, input);
    startPanel(window);

    PlaybackClock clock(glfwGetTime(), double(Viewer::kFrame));
    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();
        ImGui_ImplOpenGL2_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        int width = 0, height = 0, pixelsWide = 0, pixelsHigh = 0;
        glfwGetWindowSize(window, &width, &height);
        pointer(window, width, height);
        if (scenes()[viewer.scene].vehicle) viewer.drive = driveInput(window);
        const bool recording = frameLimit > 0 || !record.empty();
        const int steps = clock.steps(glfwGetTime(), viewer.paused, viewer.stepOnce, recording);
        for (int i = 0; i < steps; ++i) {
            if (hull_events::replaying(viewer.scene)) hull_events::step(viewer, viewer.stepOnce);
            else {
                if (viewer.frame == blastFrame) viewer.fireAll();
                viewer.step();
            }
        }
        viewer.stepOnce = false;

        glfwGetFramebufferSize(window, &pixelsWide, &pixelsHigh);
        if (hull_events::active(viewer.scene)) hull_events::panel(viewer, kPanelWidth, width, height);
        const DebugGeometry all = viewer.geometry();
        const int panel = width > 0 ? kPanelWidth * pixelsWide / width : 0; // ширина панели в пикселях кадра
        render(viewer.visible(all), viewer.camera, panel, std::max(1, pixelsWide - panel), pixelsHigh,
               viewer.world, viewer.fill, scenes()[viewer.scene], viewer.drive);
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
