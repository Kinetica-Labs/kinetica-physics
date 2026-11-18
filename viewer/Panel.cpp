#include "Panel.h"

#include <imgui.h>

#include <algorithm>
#include <string>

using namespace kp;

namespace {

// Номер каждого тела рядом с его центром.
void drawBodyNumbers(const Viewer& viewer, int width, int height) {
    int index = 0;
    for (const Body& body : viewer.world.bodies()) {
        double x = 0, y = 0;
        if (viewer.camera.project(body.position, width, height, x, y))
            ImGui::GetBackgroundDrawList()->AddText({float(x) + 6, float(y) - 8}, IM_COL32(255, 255, 255, 255), std::to_string(index).c_str());
        ++index;
    }
}

} // namespace

void drawPanel(Viewer& viewer, const DebugGeometry& all, int width, int height) {
    ImGui::SetNextWindowPos({10, 10}, ImGuiCond_FirstUseEver);
    ImGui::Begin("kinetica-physics", nullptr, ImGuiWindowFlags_AlwaysAutoResize);

    ImGui::SeparatorText("Сцена");
    for (size_t k = 0; k < scenes().size(); ++k)
        if (ImGui::RadioButton(scenes()[k].title, viewer.scene == k)) viewer.load(k);
    if (ImGui::Button(viewer.paused ? "Пуск" : "Пауза")) viewer.paused = !viewer.paused;
    ImGui::SameLine();
    if (ImGui::Button("Шаг")) viewer.stepOnce = true;
    ImGui::SameLine();
    if (ImGui::Button("Заново")) viewer.load(viewer.scene);

    ImGui::SeparatorText("Показывать");
    ImGui::Checkbox("заливка и свет", &viewer.fill);
    ImGui::Checkbox("тела", &viewer.show[int(DebugKind::Shape)]);
    ImGui::Checkbox("неподвижные тела", &viewer.show[int(DebugKind::StaticShape)]);
    ImGui::Checkbox("точки контакта", &viewer.show[int(DebugKind::ContactPoint)]);
    ImGui::Checkbox("нормали контакта", &viewer.show[int(DebugKind::ContactNormal)]);
    ImGui::Checkbox("дерево поиска пар", &viewer.show[int(DebugKind::TreeNode)]);
    ImGui::Checkbox("номера тел", &viewer.showBodyNumbers);

    ImGui::SeparatorText("Решатель");
    ImGui::Checkbox("ударная волна (shock propagation)", &viewer.shockPropagation);
    if (ImGui::Checkbox("сон покоящихся тел (сцена начнётся заново)", &viewer.sleeping)) viewer.load(viewer.scene);

    ImGui::SeparatorText("Мир");
    const auto contactPoints = std::count_if(all.points.begin(), all.points.end(),
                                             [](const DebugPoint& point) { return point.kind == DebugKind::ContactPoint; });
    ImGui::Text("время %.2f с, кадр %d", viewer.frame * double(Viewer::kFrame), viewer.frame);
    ImGui::Text("тел %zu, точек контакта %d", viewer.world.bodies().size(), int(contactPoints));
    ImGui::Text("отрезков шага %d, проходов решателя %d", viewer.world.report().segments, viewer.world.report().solverIterations);
    ImGui::Text("счёт шага %.2f мс, самый долгий %.2f мс", viewer.lastStep, viewer.slowestStep);
    ImGui::Text("отрезков в шаге не больше %d", viewer.mostSegments);
    ImGui::Checkbox("шаг с постоянной ценой кадра (иначе точный)", &viewer.realtime);
    ImGui::SliderFloat("трение качения, мм (0 - как на Луне)", &viewer.rollingResistance, 0, 5, "%.1f");
    ImGui::SliderInt("обновлений контакта на кадр", &viewer.contactUpdates, 1, 4);
    ImGui::Checkbox("четыре точки контакта на пару (иначе все)", &viewer.world.soft.fourContactPoints);
    ImGui::SliderInt("подшагов на кадр", &viewer.substeps, 1, 16);
    ImGui::SliderInt("предел отрезков точного шага", &viewer.segmentLimit, 1, 64);
    ImGui::Text("сцена отстала от часов на %.3f с", viewer.lag);

    if (viewer.world.joints().size() > 0) {
        ImGui::SeparatorText("Сочленения");
        ImGui::Text("разрыв %.3f мм, наибольший %.3f мм", 1e3 * double(viewer.world.joints().largestError(viewer.world.bodies())), 1e3 * viewer.worstGap);
    }
    if (const ExactScene* exact = scenes()[viewer.scene].exact) {
        ImGui::SeparatorText("Точное решение");
        const double measured = exact->measured(viewer.world), expected = exact->exact(viewer.frame * double(Viewer::kFrame) - viewer.lag);
        ImGui::Text("%s", exact->quantity);
        ImGui::Text("в мире   %+.5f", measured);
        ImGui::Text("точно    %+.5f", expected);
        ImGui::Text("ошибка   %.2e, наибольшая %.2e", measured - expected, viewer.worstError);
    }

    ImGui::SeparatorText("Управление");
    ImGui::TextUnformatted("левая кнопка на теле — взять и тянуть\nмимо тел или правая кнопка — камера\nсредняя кнопка — сдвинуть сцену\nколесо — ближе и дальше\n1–9 — сцена, пробел — пауза, → — шаг\nR — заново, T — мир в консоль, Esc — выход");
    ImGui::End();

    if (viewer.showBodyNumbers) drawBodyNumbers(viewer, width, height);
}
