#pragma once
// Воспроизведение поз из hull_event_sweep --animation. Геометрия и контакт запрашиваются у SDK.
#include "Viewer.h"
#include "../examples/scenes/HullSweepShapes.h"
#include <body/MassFrame.h>
#include <imgui.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <sstream>
#include <string>

namespace hull_events {
using namespace kp;
inline std::string csvPath = KP_HULL_EVENTS_FILE;
inline constexpr int samples = 25, pairs = 9, ticksPerSample = 6;
inline int releasedAt = -1; // кадр записи, из которого пользователь отпустил детали

struct Sample {
    Transform a, b;
    Vector3 witnessA, witnessB, normal;
    double time = 0, gap = 0;
};
struct Recording {
    std::array<std::array<Sample, samples>, pairs> path{};
    std::array<std::string, pairs> names{};
    std::string error;
};

inline const Recording& recording() {
    static const Recording data = [] {
        Recording result;
        std::ifstream in(csvPath);
        std::string line;
        if (!std::getline(in, line) || line.compare(0, 19, "type_a,id_a,type_b,") != 0) {
            result.error = "Укажите файл animation.csv через --hull-data"; return result;
        }
        for (int i = 0; i < pairs * samples; ++i) {
            if (!std::getline(in, line)) { result.error = "В animation.csv отсутствуют кадры"; return result; }
            std::replace(line.begin(), line.end(), ',', ' ');
            std::istringstream row(line);
            std::string a, b;
            int idA, idB, frame;
            Sample& s = result.path[i / samples][i % samples];
            const auto vec = [&](Vector3& v) { row >> v.x >> v.y >> v.z; };
            const auto pose = [&](Transform& p) {
                vec(p.position); row >> p.orientation.w >> p.orientation.x >> p.orientation.y >> p.orientation.z;
            };
            row >> a >> idA >> b >> idB >> frame >> s.time >> s.gap;
            pose(s.a); pose(s.b); vec(s.witnessA); vec(s.witnessB); vec(s.normal);
            const auto label = a + " / " + b;
            auto& name = result.names[i / samples];
            if (i % samples == 0) name = label;
            if (!row || frame != i % samples || idA != 0 || idB != 0 || label != name ||
                !std::isfinite(s.time) || (frame > 0 && s.time <= result.path[i / samples][frame - 1].time)) {
                result.error = "Нарушен порядок кадров в animation.csv"; return result;
            }
        }
        return result;
    }();
    return data;
}

// CSV содержит позу исходной сетки. Тело хранит позу центра масс в главных осях.
// T_тела = T_сетки * T_инерции; обратное преобразование размещает исходную форму на теле.
struct Part {
    Body body;
    Transform inertia;
    explicit Part(const hull_sweep::Mesh& mesh) {
        const auto hull = std::make_shared<ConvexHull>(mesh.vertices, mesh.faces);
        const MassFrame m = MassFrame::of(*hull, 1000); // одинаковая учебная плотность, кг/м³
        inertia = m.frame;
        body.mass = m.mass;
        body.shape = hull;
        body.placement = inertia.relative(Transform{});
        body.placed = true;
    }
};
inline const std::array<Part, 2>& parts() {
    static const std::array<Part, 2> value{Part(hull_sweep::shapeA()), Part(hull_sweep::shapeB())};
    return value;
}
inline int pairAt(int frame) { return frame / (samples * ticksPerSample) % pairs; }
inline int sampleAt(int frame) { return frame / ticksPerSample % samples; }
inline Transform centrePose(const Sample& sample, int i) {
    const auto& model = i == 0 ? sample.a : sample.b;
    const Transform& inertia = parts()[i].inertia;
    return {model.point(inertia.position), model.orientation * inertia.orientation};
}
inline void show(World& world, int frame) {
    if (!recording().error.empty()) return;
    const auto& s = recording().path[pairAt(frame)][sampleAt(frame)];
    for (int i = 0; i < 2; ++i) {
        const Transform pose = centrePose(s, i);
        Body& body = world.body(BodyId(i));
        body.position = pose.position;
        body.orientation = pose.orientation;
    }
}
inline World build() {
    releasedAt = -1;
    World world;
    world.gravity = {};
    if (recording().error.empty()) {
        for (const auto& p : parts()) world.addBody(p.body);
        show(world, 0);
    }
    return world;
}
inline bool active(size_t scene) { return scenes()[scene].build == build; }
inline bool replaying(size_t scene) { return active(scene) && releasedAt < 0; }
inline ImVec2 panelOrigin{}, panelSize{};
inline bool panelContains(size_t scene, double x, double y) {
    return active(scene) && x >= panelOrigin.x && x < panelOrigin.x + panelSize.x &&
           y >= panelOrigin.y && y < panelOrigin.y + panelSize.y;
}

// Односторонняя разность в момент отпускания: от текущего замера к следующему,
// а в конце траектории — последний интервал этой же пары. Время берём из CSV.
inline void releaseBodies(Viewer& viewer) {
    releasedAt = viewer.frame;
    const auto& path = recording().path[pairAt(releasedAt)];
    const int from = std::min(sampleAt(releasedAt), samples - 2);
    const Real dt = path[from + 1].time - path[from].time;
    for (int i = 0; i < 2; ++i) {
        Body& body = viewer.world.body(BodyId(i));
        const Transform a = centrePose(path[from], i), b = centrePose(path[from + 1], i);
        body.linearMomentum = (1 / (dt * body.mass.inverseMass)) * (b.position - a.position);
        const Quaternion dq = (b.orientation * a.orientation.conjugate()).unit();
        const Vector3 axis = (dq.w < 0 ? -1 : 1) * Vector3{dq.x, dq.y, dq.z};
        const Real sine = axis.length();
        const Real rate = sine > 0 ? 2 * std::atan2(sine, std::abs(dq.w)) / (sine * dt) : 0;
        const Vector3 spin = body.orientation.rotateInverse(rate * axis);
        const Vector3 invI = body.mass.inverseInertia;
        body.angularMomentum = body.orientation.rotate({spin.x / invI.x, spin.y / invI.y, spin.z / invI.z});
    }
    viewer.frame = 0;
    viewer.paused = false;
}

inline void restoreRecording(Viewer& viewer) {
    const int savedFrame = releasedAt;
    const Camera camera = viewer.camera;
    viewer.load(viewer.scene); // также удаляет привязь мыши и накопленные контакты решателя
    viewer.camera = camera;
    viewer.frame = savedFrame;
    viewer.paused = true;
    show(viewer.world, savedFrame);
}

inline void step(Viewer& viewer, bool manual) {
    viewer.frame = manual ? (viewer.frame / ticksPerSample + 1) * ticksPerSample : viewer.frame + 1;
    show(viewer.world, viewer.frame);
}

// Метки живут в мировых координатах и проецируются той же камерой, что и тела.
inline void panel(Viewer& viewer, int left, int width, int height) {
    ImGui::SetNextWindowPos(ImVec2(float(left + 12), 12), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowBgAlpha(.9f);
    ImGui::Begin("Траектории двух деталей", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
    panelOrigin = ImGui::GetWindowPos();
    panelSize = ImGui::GetWindowSize();
    const Recording& data = recording();
    if (!data.error.empty()) { ImGui::TextUnformatted(data.error.c_str()); ImGui::End(); return; }
    if (releasedAt < 0) {
        ImGui::TextUnformatted("Запись опыта");
        int pair = pairAt(viewer.frame), frame = sampleAt(viewer.frame);
        if (ImGui::BeginCombo("Пара", data.names[pair].c_str())) {
            for (int i = 0; i < pairs; ++i)
                if (ImGui::Selectable(data.names[i].c_str(), i == pair)) pair = i;
            ImGui::EndCombo();
        }
        ImGui::SliderInt("Кадр опыта", &frame, 0, samples - 1);
        if (pair != pairAt(viewer.frame) || frame != sampleAt(viewer.frame)) {
            viewer.frame = (pair * samples + frame) * ticksPerSample;
            viewer.paused = true;
            show(viewer.world, viewer.frame);
        }
        const auto& sample = data.path[pair][frame];
        ImGui::Text("t опыта %.3f с    зазор %+.3f мм", sample.time, 1000 * sample.gap);
        ImGui::TextUnformatted("Пробел — пауза, → — один замер, ПКМ — камера");
        if (ImGui::Button("Отпустить тела")) releaseBodies(viewer);
    } else {
        ImGui::TextUnformatted("Свободная динамика движка");
        ImGui::TextUnformatted("Без тяжести, плотность обеих деталей 1000 кг/м³");
        ImGui::TextUnformatted("Тяните тело ЛКМ, поворачивайте камеру ПКМ");
        if (ImGui::Button("Вернуть запись")) restoreRecording(viewer);
    }
    static bool axes = true, oracle = true;
    ImGui::Checkbox("Оси центров масс XYZ", &axes);
    if (releasedAt < 0) ImGui::Checkbox("Фиолетовая нормаль оракула", &oracle);
    ImGui::End();

    auto* draw = ImGui::GetBackgroundDrawList();
    const auto pixel = [&](Vector3 v, ImVec2& p) {
        double x, y;
        if (!viewer.camera.project(v, width - left, height, x, y)) return false;
        p = {float(x + left), float(y)}; return true;
    };
    const auto line = [&](Vector3 start, Vector3 end, ImU32 colour) {
        ImVec2 a, b;
        if (pixel(start, a) && pixel(end, b)) draw->AddLine(a, b, colour, 2);
    };
    draw->PushClipRect({float(left), 0}, {float(width), float(height)}, true);
    if (axes) {
        for (const Body& body : viewer.world.bodies()) {
            if (!body.shape) continue;
            const auto p = body.position;
            line(p, p + body.orientation.rotate({.14, 0, 0}), IM_COL32(250, 90, 75, 255));
            line(p, p + body.orientation.rotate({0, .14, 0}), IM_COL32(95, 235, 105, 255));
            line(p, p + body.orientation.rotate({0, 0, .14}), IM_COL32(85, 155, 255, 255));
        }
    }
    if (releasedAt < 0 && oracle) {
        const auto& sample = data.path[pairAt(viewer.frame)][sampleAt(viewer.frame)];
        line(sample.witnessA, sample.witnessA + .15 * sample.normal, IM_COL32(224, 100, 255, 255));
    }
    draw->PopClipRect();
}
} // namespace hull_events
