#include "Panel.h"

#include <imgui.h>

#include <algorithm>
#include <cstring>
#include <string>

using namespace kp;

namespace {

constexpr float kFieldWidth = 170; // ширина ползунка, пикселей: подпись стоит справа от него
const ImVec4 kTitle{1, 0.85f, 0.4f, 1}, kGood{0.45f, 1, 0.55f, 1}, kBad{1, 0.5f, 0.45f, 1};

// Подсказка под курсором к последнему нарисованному элементу: подписи панели короткие, пояснение — здесь.
void hint(const char* text) { ImGui::SetItemTooltip("%s", text); }

// Номер каждого тела рядом с его центром; сцена нарисована правее панели.
void drawBodyNumbers(const Viewer& viewer, int width, int height) {
    int index = 0;
    for (const Body& body : viewer.world.bodies()) {
        double x = 0, y = 0;
        if (viewer.camera.project(body.position, width - kPanelWidth, height, x, y))
            ImGui::GetBackgroundDrawList()->AddText({float(x) + kPanelWidth + 6, float(y) - 8}, IM_COL32(255, 255, 255, 255), std::to_string(index).c_str());
        ++index;
    }
}

// Шапка: какая сцена идёт, пуск и пауза, режим шага. Режим — двумя кнопками, а не галочкой: его смена
// начинает сцену заново, иначе точный шаг получил бы тела, уже вдавленные друг в друга быстрым.
void drawHeader(Viewer& viewer) {
    ImGui::PushTextWrapPos(0);
    ImGui::TextColored(kTitle, "%zu. %s", viewer.scene + 1, scenes()[viewer.scene].title);
    ImGui::PopTextWrapPos();
    if (ImGui::Button(viewer.paused ? " Пуск " : "Пауза")) viewer.paused = !viewer.paused;
    hint("пробел");
    ImGui::SameLine();
    if (ImGui::Button("Шаг")) viewer.stepOnce = true;
    hint("один кадр; клавиша «стрелка вправо»");
    ImGui::SameLine();
    if (ImGui::Button("Заново")) viewer.load(viewer.scene);
    hint("клавиша R");
    ImGui::AlignTextToFramePadding();
    ImGui::TextUnformatted("шаг:");
    ImGui::SameLine();
    if (ImGui::RadioButton("быстрый", viewer.realtime) && !viewer.realtime) viewer.realtime = true, viewer.load(viewer.scene);
    hint("цена кадра постоянна, тела могут немного войти друг в друга.\nСмена режима начинает сцену заново");
    ImGui::SameLine();
    if (ImGui::RadioButton("точный", !viewer.realtime) && viewer.realtime) viewer.realtime = false, viewer.load(viewer.scene);
    hint("эталон: без перекрытий, цена кадра растёт с числом событий.\nСмена режима начинает сцену заново");
}

// Чем сцена обязана кончиться, знает она сама (StandardScene::verdict): итог виден с момента проверки.
// Прибор считает сцены без воздуха и сна; пока настройки окна другие, итог с ним не сравнить — об этом
// сказано рядом, и кнопка ставит настройки прибора.
void drawVerdict(Viewer& viewer) {
    const StandardScene* standard = scenes()[viewer.scene].standard;
    if (!standard) return;
    ImGui::PushTextWrapPos(0);
    if (!viewer.judged) ImGui::TextDisabled("проверка сцены будет в t = %.1f с", standard->checkAt);
    else if (viewer.verdict.empty()) ImGui::TextColored(kGood, "в момент %.1f с: в допуске", standard->checkAt);
    else ImGui::TextColored(kBad, "в момент %.1f с: %s", standard->checkAt, viewer.verdict.c_str());
    if (!viewer.asMeasured()) {
        ImGui::TextDisabled("в окне включены воздух или сон: у прибора с замерами их нет, итог может отличаться");
        if (ImGui::SmallButton("Считать как прибор")) viewer.measure();
        hint("без воздуха, сна и трения качения, с умолчаниями шага; сцена начнётся заново");
    }
    ImGui::PopTextWrapPos();
}

// Сцена с точным решением: величина, её точное значение и ошибка.
void drawExact(const Viewer& viewer) {
    const ExactScene* exact = scenes()[viewer.scene].exact;
    if (!exact) return;
    const double measured = exact->measured(viewer.world), expected = exact->exact(viewer.frame * double(Viewer::kFrame) - viewer.lag);
    ImGui::PushTextWrapPos(0);
    ImGui::TextUnformatted(exact->quantity);
    ImGui::PopTextWrapPos();
    ImGui::Text("в мире %+.5f, точно %+.5f", measured, expected);
    ImGui::TextColored(kTitle, "ошибка %.2e, наибольшая %.2e", measured - expected, viewer.worstError);
}

// Числа о мире: время, тела, контакты, цена шага.
void drawStatus(Viewer& viewer, const DebugGeometry& all) {
    const auto contactPoints = std::count_if(all.points.begin(), all.points.end(), [](const DebugPoint& point) { return point.kind == DebugKind::ContactPoint; });
    ImGui::Text("время %.2f с, кадр %d", viewer.frame * double(Viewer::kFrame), viewer.frame);
    ImGui::Text("тел %zu, показано точек контакта %d", viewer.world.bodies().size(), int(contactPoints));
    ImGui::Text("счёт шага %.2f мс, самый долгий %.2f", viewer.lastStep, viewer.slowestStep);
    if (viewer.world.joints().size() > 0)
        ImGui::Text("разрыв сочленений %.3f мм, наибольший %.3f", 1e3 * double(viewer.world.joints().largestError(viewer.world.bodies())), 1e3 * viewer.worstGap);
    drawVerdict(viewer);
    drawExact(viewer);
}

// Сцены группами (Scene::group); раскрыта та группа, в которой сцена, открытая при запуске.
void drawScenes(Viewer& viewer) {
    for (size_t first = 0, last = 0; first < scenes().size(); first = last) {
        while (last < scenes().size() && std::strcmp(scenes()[last].group, scenes()[first].group) == 0) ++last;
        ImGui::SetNextItemOpen(viewer.scene >= first && viewer.scene < last, ImGuiCond_Once);
        if (!ImGui::TreeNode(scenes()[first].group, "%s (%zu)", scenes()[first].group, last - first)) continue;
        for (size_t k = first; k < last; ++k) {
            const std::string label = std::to_string(k + 1) + ". " + scenes()[k].title;
            if (ImGui::Selectable(label.c_str(), viewer.scene == k)) viewer.load(k);
            hint(scenes()[k].title);
        }
        ImGui::TreePop();
    }
}

// Один источник импульса: ползунки, пуск, постановка центра мышью. true — источник просят удалить.
bool drawImpulse(Viewer& viewer, size_t index) {
    ImpulseTool& tool = viewer.impulses[index];
    float center[3] = {float(tool.center.x), float(tool.center.y), float(tool.center.z)};
    if (ImGui::DragFloat3("центр, м", center, 0.02f)) tool.center = {Real(center[0]), Real(center[1]), Real(center[2])};
    ImGui::SliderFloat("радиус, м", &tool.radius, 0.1f, 10, "%.2f");
    hint("тело задето, если его ближайшая точка в шаре этого радиуса");
    ImGui::SliderFloat(tool.everyFrame ? "сила, Н" : "импульс, Н·с", &tool.impulse, 1, 20000, "%.0f", ImGuiSliderFlags_Logarithmic);
    ImGui::SliderInt("спад, степень", &tool.falloff, 0, 3);
    hint("доля импульса (1 − расстояние / радиус) в этой степени; 0 — без спада");
    if (!tool.radial) {
        ImGui::SliderAngle("поворот", &tool.yaw, -180, 180);
        ImGui::SliderAngle("подъём", &tool.pitch, -90, 90);
    }
    ImGui::Checkbox("действует постоянно", &tool.everyFrame);
    hint("сила: толкает каждый кадр, пока стоит галочка");
    if (ImGui::Button("Пуск")) tool.touched = tool.fire(viewer.world);
    ImGui::SameLine();
    if (ImGui::Button(viewer.placing == int(index) ? "щёлкните по сцене" : "Поставить мышью")) viewer.placing = int(index);
    hint("следующий щелчок по телу сцены ставит туда центр");
    ImGui::SameLine();
    const bool remove = ImGui::Button("Удалить");
    ImGui::Text("последний пуск задел тел: %d", tool.touched);
    return remove;
}

// Источники импульсов раскрывающимся списком: взрывы и толчки в направлении.
void drawImpulses(Viewer& viewer) {
    for (bool radial : {true, false}) {
        if (ImGui::Button(radial ? "+ взрыв" : "+ толчок в направлении")) {
            viewer.impulses.push_back({});
            viewer.impulses.back().radial = radial, viewer.impulses.back().center = viewer.camera.target;
        }
        hint(radial ? "импульс всем телам в шаре, вдоль луча от центра" : "импульс всем телам в шаре, в одном направлении");
        ImGui::SameLine();
    }
    if (ImGui::Button("Пуск всех")) viewer.fireAll();
    hint("клавиша E");
    if (viewer.impulses.empty()) ImGui::TextDisabled("источников нет: добавьте взрыв или толчок");
    for (size_t k = 0; k < viewer.impulses.size(); ++k) {
        ImGui::PushID(int(k));
        const bool open = ImGui::TreeNodeEx("source", ImGuiTreeNodeFlags_DefaultOpen, "%zu. %s", k + 1, viewer.impulses[k].radial ? "взрыв" : "толчок в направлении");
        const bool remove = open && drawImpulse(viewer, k);
        if (open) ImGui::TreePop();
        ImGui::PopID();
        if (!remove) continue;
        viewer.impulses.erase(viewer.impulses.begin() + std::ptrdiff_t(k));
        viewer.placing = -1;
        break;
    }
}

void drawShown(Viewer& viewer) {
    ImGui::Checkbox("заливка и свет", &viewer.fill);
    ImGui::Checkbox("тела", &viewer.show[int(DebugKind::Shape)]);
    ImGui::Checkbox("неподвижные тела", &viewer.show[int(DebugKind::StaticShape)]);
    ImGui::Checkbox("точки контакта", &viewer.show[int(DebugKind::ContactPoint)]);
    ImGui::Checkbox("нормали контакта", &viewer.show[int(DebugKind::ContactNormal)]);
    ImGui::Checkbox("пересчёт отметок в покое", &viewer.rawContactMarkers);
    hint("показывает исходные геометрические контакты даже при изменении позы на величину округления");
    ImGui::Checkbox("дерево поиска пар", &viewer.show[int(DebugKind::TreeNode)]);
    ImGui::Checkbox("номера тел", &viewer.showBodyNumbers);
}

void drawSettings(Viewer& viewer) {
    ImGui::SliderFloat("воздух, 1/с", &viewer.damping, 0, 2, "%.2f");
    hint("сопротивление среды; 0 — как на Луне. У сцен с точным ответом выключено");
    if (ImGui::Checkbox("сон покоящихся тел", &viewer.sleeping)) viewer.load(viewer.scene);
    hint("сцена начнётся заново");
    ImGui::Checkbox("проход по опорам (shock propagation)", &viewer.shockPropagation);
    hint("уменьшает лишнее сближение в опорах; действует со следующего шага");
    ImGui::SeparatorText("быстрый шаг");
    const Scene& scene = scenes()[viewer.scene];
    if (scene.singleSubstep) {
        ImGui::Text("%d шагов за кадр, по одному поиску контактов", scene.updatesPerFrame);
    } else {
        ImGui::SliderInt("подшагов на кадр", &viewer.substeps, 1, 16);
        ImGui::SliderInt("поисков контакта", &viewer.contactUpdates, 1, 4);
        hint("сколько раз за кадр заново ищутся пары и точки контакта");
    }
    ImGui::SliderFloat("трение качения, мм", &viewer.rollingResistance, 0, 5, "%.1f");
    ImGui::Checkbox("четыре точки на пару", &viewer.world.soft.fourContactPoints);
    hint("иначе в решатель идут все точки площадки");
    ImGui::Checkbox("совместное трение двух контактов", &viewer.world.soft.coupledFriction);
    hint("для быстрого шага: совместное решение касательных импульсов в подходящем острове");
    ImGui::SeparatorText("точный шаг");
    ImGui::SliderInt("предел отрезков", &viewer.segmentLimit, 1, 64);
    hint("не больше отрезков на кадр: шаг, которому их не хватило, отстаёт от часов, зато окно не замирает");
    ImGui::Text("отрезков шага %d, не больше %d", viewer.world.report().segments, viewer.mostSegments);
    ImGui::Text("проходов решателя %d", viewer.world.report().solverIterations);
    ImGui::Text("сцена отстала от часов на %.3f с", viewer.lag);
}

void drawHelp() {
    ImGui::TextUnformatted("левая кнопка на теле — взять и тянуть\nмимо тел или правая кнопка — камера\nсредняя кнопка — сдвинуть сцену\nколесо — ближе и дальше\n"
                           "1–9 — первые девять сцен\nпробел — пауза, стрелка вправо — шаг\nR — заново, E — пуск всех импульсов\nT — мир текстом в консоль, Esc — выход");
}

} // namespace

void drawPanel(Viewer& viewer, const DebugGeometry& all, int width, int height) {
    ImGui::SetNextWindowPos({0, 0});
    ImGui::SetNextWindowSize({float(kPanelWidth), float(height)});
    ImGui::Begin("панель", nullptr, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoSavedSettings);
    ImGui::PushItemWidth(kFieldWidth);
    drawHeader(viewer);
    if (scenes()[viewer.scene].vehicle)
        ImGui::TextWrapped("W / S — вперёд и назад. A / D — руль. Отпустите W и S для торможения.");
    ImGui::Separator();
    drawStatus(viewer, all);
    ImGui::Spacing();
    if (ImGui::CollapsingHeader("Сцены", ImGuiTreeNodeFlags_DefaultOpen)) drawScenes(viewer);
    ImGui::SetNextItemOpen(!viewer.impulses.empty(), ImGuiCond_Once);
    if (ImGui::CollapsingHeader(("Импульсы: взрывы и толчки (" + std::to_string(viewer.impulses.size()) + ")###impulses").c_str())) drawImpulses(viewer);
    if (ImGui::CollapsingHeader("Что показывать")) drawShown(viewer);
    if (ImGui::CollapsingHeader("Настройки шага")) drawSettings(viewer);
    if (ImGui::CollapsingHeader("Мышь и клавиши")) drawHelp();
    ImGui::PopItemWidth();
    ImGui::End();
    if (viewer.showBodyNumbers) drawBodyNumbers(viewer, width, height);
}
