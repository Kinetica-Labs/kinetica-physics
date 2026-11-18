// Одни и те же сцены для разных движков. Программа движка только строит мир сцены и шагает;
// варианты сцены, подсчёт итогов и строка JSON для журнала табло — здесь.
//
// Статистика: стопка хаотична — сдвиг куба на доли миллиметра меняет итог, поэтому каждая сцена
// идёт в kVariants вариантах. Вариант 0 — сцена как описана; остальные — малые возмущения,
// одинаковые для всех движков (сравнение идёт парами по номеру варианта):
// плотность каждого тела ±1 %; в сценах без сочленений ещё слой тел (тела на одной высоте)
// сдвинут вбок до ±0,5 мм, приподнят над нижним до 0,5 мм, а одиночное в слое тело повёрнуто
// вокруг вертикали до ±0,2°. Начальных перекрытий возмущения не создают.
#pragma once
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <map>
#include <string>
#include <vector>

struct V3 { double x = 0, y = 0, z = 0; };
inline V3 operator+(V3 a, V3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
inline V3 operator-(V3 a, V3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
inline V3 operator*(double s, V3 a) { return {s * a.x, s * a.y, s * a.z}; }
inline double dot(V3 a, V3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline V3 cross(V3 a, V3 b) { return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x}; }
inline double len(V3 a) { return std::sqrt(dot(a, a)); }
struct Q { double w = 1, x = 0, y = 0, z = 0; };
inline V3 rotate(Q q, V3 v) { const V3 u{q.x, q.y, q.z}; return v + 2 * cross(u, cross(u, v) + q.w * v); }
inline V3 unrotate(Q q, V3 v) { return rotate({q.w, -q.x, -q.y, -q.z}, v); }
inline Q aboutZ(double a) { return {std::cos(a / 2), 0, 0, std::sin(a / 2)}; }
inline Q aboutY(double a) { return {std::cos(a / 2), 0, std::sin(a / 2), 0}; }

struct BoxSpec { V3 half, at; double density = 1000; Q turn; V3 velocity; }; // density 0 — неподвижное
struct JointSpec { int a, b; V3 anchor; bool weld; };             // шарнир — вокруг оси z мира
// settle — с какой секунды сцена обязана стоять: с неё меряется дрожь.
struct SceneSpec { const char* key; const char* name; std::vector<BoxSpec> boxes; std::vector<JointSpec> joints; double seconds; bool pendulum = false; double settle = 1; };
struct State { V3 at; Q turn; V3 v, w; bool asleep = false; };

constexpr int kVariants = 16;
constexpr double kGravity = 9.81, kWallBudget = 20; // м/с²; с настоящего времени на прогон

inline double massOf(const BoxSpec& b) { return 8 * b.density * b.half.x * b.half.y * b.half.z; }
inline double energy(const SceneSpec& s, const std::vector<State>& st) {
    double e = 0;
    for (size_t i = 0; i < s.boxes.size(); ++i) {
        const BoxSpec& b = s.boxes[i];
        if (b.density == 0) continue;
        const double m = massOf(b);
        const V3 h = b.half, I{m * (h.y * h.y + h.z * h.z) / 3, m * (h.x * h.x + h.z * h.z) / 3, m * (h.x * h.x + h.y * h.y) / 3}, w = unrotate(st[i].turn, st[i].w);
        e += 0.5 * m * dot(st[i].v, st[i].v) + 0.5 * (I.x * w.x * w.x + I.y * w.y * w.y + I.z * w.z * w.z) + m * kGravity * st[i].at.y;
    }
    return e;
}
inline double jointGap(const SceneSpec& s, const std::vector<State>& st) {
    double gap = 0;
    for (const JointSpec& j : s.joints) {
        const BoxSpec &a = s.boxes[size_t(j.a)], &b = s.boxes[size_t(j.b)];
        const V3 la = unrotate(a.turn, j.anchor - a.at), lb = unrotate(b.turn, j.anchor - b.at);
        gap = std::max(gap, len(st[size_t(j.a)].at + rotate(st[size_t(j.a)].turn, la) - st[size_t(j.b)].at - rotate(st[size_t(j.b)].turn, lb)));
    }
    return gap;
}

// Глубина перекрытия двух брусков по разделяющим осям (SAT): наименьшее перекрытие проекций по 15
// осям — 3 + 3 оси граней и 9 произведений рёбер; 0, если хоть одна ось разделяет. Для брусков это
// наименьший сдвиг, разводящий их; почти параллельные рёбра дают ось с шумом, но перекрытие по
// любой оси не меньше настоящего, так что минимум не врёт вниз.
inline double penetration(const BoxSpec& a, const State& sa, const BoxSpec& b, const State& sb) {
    const V3 t = sb.at - sa.at;
    if (len(t) > len(a.half) + len(b.half)) return 0;
    const V3 unit[3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    const double ha[3] = {a.half.x, a.half.y, a.half.z}, hb[3] = {b.half.x, b.half.y, b.half.z};
    V3 ea[3], eb[3];
    for (int i = 0; i < 3; ++i) ea[i] = rotate(sa.turn, unit[i]), eb[i] = rotate(sb.turn, unit[i]);
    double depth = 1e300;
    const auto along = [&](V3 axis) {
        if (len(axis) < 1e-6) return; // параллельные рёбра: ось не определена, её заменяет ось грани
        axis = (1 / len(axis)) * axis;
        double overlap = -std::fabs(dot(t, axis));
        for (int i = 0; i < 3; ++i) overlap += ha[i] * std::fabs(dot(ea[i], axis)) + hb[i] * std::fabs(dot(eb[i], axis));
        depth = std::min(depth, overlap);
    };
    for (int i = 0; i < 3; ++i) {
        along(ea[i]), along(eb[i]);
        for (int j = 0; j < 3; ++j) along(cross(ea[i], eb[j]));
    }
    return std::max(depth, 0.0);
}
// Наибольшее перекрытие пар тел сцены. Только в сценах без сочленений: соседние звенья сочленения
// касаются друг друга по замыслу.
inline double deepestOverlap(const SceneSpec& s, const std::vector<State>& st) {
    double deepest = 0;
    for (size_t i = 0; i < s.boxes.size(); ++i)
        for (size_t j = i + 1; j < s.boxes.size(); ++j)
            if (s.boxes[i].density != 0 || s.boxes[j].density != 0) deepest = std::max(deepest, penetration(s.boxes[i], st[i], s.boxes[j], st[j]));
    return deepest;
}

// Брусок с полуразмерами (d, hy) на шарнире в торце: θ'' = −g sin θ / (I/(m d)), I/(m d) = (d² + hy²)/(3d) + d.
// Рунге — Кутта с шагом 10 мкс: ошибка много меньше 1e-6 рад.
inline double pendulumAngle(double release, double d, double hy, double time) {
    const double k = kGravity / ((d * d + hy * hy) / (3 * d) + d), dt = 1e-5;
    double angle = release, speed = 0;
    for (double at = 0; at < time - dt / 2; at += dt) {
        const double a1 = -k * std::sin(angle), v2 = speed + dt / 2 * a1, a2 = -k * std::sin(angle + dt / 2 * speed);
        const double v3 = speed + dt / 2 * a2, a3 = -k * std::sin(angle + dt / 2 * v2), v4 = speed + dt * a3, a4 = -k * std::sin(angle + dt * v3);
        angle += dt / 6 * (speed + 2 * v2 + 2 * v3 + v4), speed += dt / 6 * (a1 + 2 * a2 + 2 * a3 + a4);
    }
    return angle;
}

inline std::vector<SceneSpec> scenes() {
    const BoxSpec floor{{5, 0.5, 5}, {0, -0.5, 0}, 0};
    const V3 pivot{0, 3, 0}, cube{0.2, 0.2, 0.2}, bar{0.1, 0.03, 0.03};
    std::vector<SceneSpec> all;
    SceneSpec stack{"stack10", "стопка из 10 кубов", {floor}, {}, 5};
    for (int k = 0; k < 10; ++k) stack.boxes.push_back({cube, {0, 0.2 + 0.4 * k, 0}});
    all.push_back(stack);
    SceneSpec pyramid{"pyramid55", "пирамида из 55 кубов", {floor}, {}, 5};
    for (int row = 0; row < 10; ++row)
        for (int k = 0; k < 10 - row; ++k) pyramid.boxes.push_back({cube, {0.4 * (k - (9 - row) / 2.0), 0.2 + 0.4 * row, 0}});
    all.push_back(pyramid);
    // Осыпь: из пирамиды 36 в начале выбит вбок куб нижнего ряда, часть кубов падает; к пятой
    // секунде груда обязана стоять (дрожь и мигание сна — docs/experiments/40-sleep-flicker.md).
    SceneSpec rubble{"rubble36", "осыпь: из пирамиды 36 выбит куб нижнего ряда", {floor}, {}, 8, false, 5};
    for (int row = 0; row < 8; ++row)
        for (int k = 0; k < 8 - row; ++k) rubble.boxes.push_back({cube, {0.4 * k + 0.2 * row - 1.4, 0.2 + 0.4 * row, 0}, 1000, {}, row == 0 && k == 3 ? V3{0, 0, 3} : V3{}});
    all.push_back(rubble);
    // Два куба с зазором 1 мм: больше касания (0,1 мм), меньше пути за кадр (≈ 3 мм) — пара, которая
    // засыпала двумя островами и будила друг друга по кругу (docs/experiments/40-sleep-flicker.md).
    // Правый падает с 5 мм и успокаивается позже левого: цикл начинается, когда соседи засыпают врозь.
    // Тело в теле: куб на 10 см в полу и куб на 10 см в соседнем кубе. Выталкивание обязано
    // развести их без вылета: тело не поднимается выше своего итогового положения (overshoot_mm).
    all.push_back({"overlap", "тело в теле: вдавлены на 10 см", {floor, {cube, {-1, 0.1, 0}}, {cube, {1, 0.2, 0}}, {cube, {1, 0.5, 0}}}, {}, 3, false, 1.5});
    all.push_back({"gap1mm", "два куба с зазором 1 мм", {floor, {cube, {-0.2005, 0.2, 0}}, {cube, {0.2005, 0.205, 0}}}, {}, 3});
    all.push_back({"cube100", "куб 100:1 на лёгком кубе", {floor, {cube, {0, 0.2, 0}}, {cube, {0, 0.6, 0}, 100000}}, {}, 5});
    // Высокий столбик, как в окне (viewer, «столбик из … кубов»): кубы с ребром 20 см висят в 1 см
    // друг над другом и садятся снизу вверх, верхний падает с метра. Опора от пола должна пройти
    // через сто контактов: проходы мягкого шага не доносят её за кадр, кубы проседают друг в друга
    // и вылетают вбок (overlap_mm, drift_mm). Точный шаг держит его с перекрытием 0.
    SceneSpec column{"column100", "столбик из 100 кубов, падают с 1 см", {floor}, {}, 5, false, 2};
    for (int k = 0; k < 100; ++k) column.boxes.push_back({{0.1, 0.1, 0.1}, {0, 0.11 + 0.21 * k, 0}});
    all.push_back(column);
    const double release = 2 * M_PI / 3;
    all.push_back({"pendulum120", "маятник 120°", {{{0.05, 0.05, 0.05}, pivot, 0}, {{0.4, 0.03, 0.03}, pivot + 0.4 * V3{std::sin(release), -std::cos(release), 0}, 1000, aboutZ(release - M_PI / 2)}}, {{0, 1, pivot, false}}, 10, true});
    SceneSpec beam{"beam6", "балка из 6 сваренных брусков", {{{0.05, 0.05, 0.05}, pivot, 0}}, {}, 5}, chain{"chain100", "цепь из 10 шарниров, груз 100:1", {{{0.05, 0.05, 0.05}, pivot, 0}}, {}, 5};
    for (int i = 0; i < 6; ++i) beam.boxes.push_back({bar, pivot + V3{0.1 + 0.2 * i, 0, 0}}), beam.joints.push_back({i, i + 1, pivot + V3{0.2 * i, 0, 0}, true});
    for (int i = 0; i < 10; ++i) chain.boxes.push_back({bar, pivot + V3{0.1 + 0.2 * i, 0, 0}, i == 9 ? 100000.0 : 1000.0}), chain.joints.push_back({i, i + 1, pivot + V3{0.2 * i, 0, 0}, false});
    all.push_back(beam), all.push_back(chain);
    return all;
}

// splitmix64: одинаковая последовательность на любой платформе; uniform() — в [−1, 1).
class Noise {
public:
    explicit Noise(std::uint64_t seed) : state_(seed * 0x9E3779B97F4A7C15ull) {}
    double uniform() {
        std::uint64_t z = (state_ += 0x9E3779B97F4A7C15ull);
        z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull, z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
        return double((z ^ (z >> 31)) >> 11) / double(1ull << 52) - 1;
    }

private:
    std::uint64_t state_;
};

inline SceneSpec variant(SceneSpec s, int index) {
    if (index == 0) return s;
    Noise noise{std::uint64_t(index)};
    for (BoxSpec& b : s.boxes) b.density *= b.density == 0 ? 1 : 1 + 0.01 * noise.uniform();
    if (!s.joints.empty()) return s;
    std::map<long, std::vector<BoxSpec*>> layers; // слой — тела с одной высотой центра, снизу вверх
    for (BoxSpec& b : s.boxes)
        if (b.density != 0) layers[std::lround(b.at.y * 1e6)].push_back(&b);
    double lift = 0;
    for (auto& [height, layer] : layers) {
        const V3 shift{0.5e-3 * noise.uniform(), 0, 0.5e-3 * noise.uniform()};
        lift += 0.25e-3 * (1 + noise.uniform());
        const double yaw = layer.size() == 1 ? 0.2 * M_PI / 180 * noise.uniform() : 0;
        for (BoxSpec* b : layer) b->at = b->at + shift + V3{0, lift, 0}, b->turn = aboutY(yaw);
    }
    return s;
}

using Step = std::function<void(double)>;
using Read = std::function<std::vector<State>()>;
using Play = std::function<void(const Step&, const Read&)>;
// Программа движка: построить мир сцены, вызвать play(step, read), разрушить мир.
using Simulate = std::function<void(const SceneSpec&, const Play&)>;

struct Outcome { std::map<std::string, double> numbers; };

// FNV-1a по битам состояния всех тел каждого кадра: совпадение — побитовое совпадение следа.
inline void absorb(std::uint64_t& hash, const std::vector<State>& st) {
    for (const State& s : st) {
        const double v[13] = {s.at.x, s.at.y, s.at.z, s.turn.w, s.turn.x, s.turn.y, s.turn.z, s.v.x, s.v.y, s.v.z, s.w.x, s.w.y, s.w.z};
        unsigned char bytes[sizeof v];
        std::memcpy(bytes, v, sizeof v);
        for (unsigned char byte : bytes) hash = (hash ^ byte) * 1099511628211ull;
    }
}

// Дрожь, как её видит глаз, — после s.settle: наибольшая скорость точки тела по кадрам
// (|v| + |ω| · полудиагональ), развороты скорости тел (v · v прошлого кадра < 0 при |v| > 0,1 мм/с)
// и пробуждения спящих. Скорость в последнем кадре дрожи не видит: груда может стоять в среднем и
// дёргаться, а мигание сна меняет цвет тел при нулевой скорости.
struct Jitter {
    std::vector<double> peaks; // мм/с
    int reversals = 0, wakes = 0;
    std::vector<State> last;

    void add(const SceneSpec& s, const std::vector<State>& now) {
        double peak = 0;
        for (size_t i = 0; i < s.boxes.size(); ++i) {
            if (s.boxes[i].density == 0) continue;
            peak = std::max(peak, len(now[i].v) + len(now[i].w) * len(s.boxes[i].half));
            if (last.empty()) continue;
            reversals += dot(now[i].v, last[i].v) < 0 && len(now[i].v) > 1e-4;
            wakes += last[i].asleep && !now[i].asleep;
        }
        peaks.push_back(1e3 * peak), last = now;
    }
    double median() const {
        std::vector<double> sorted = peaks;
        std::sort(sorted.begin(), sorted.end());
        return sorted.empty() ? 0 : sorted[sorted.size() / 2];
    }
};

// Кадры по 1/60 с, пока не кончится время опыта или kWallBudget настоящего времени.
inline Outcome play(const SceneSpec& s, const Step& step, const Read& read, std::uint64_t& hash) {
    const std::vector<State> start = read();
    const auto clock = std::chrono::steady_clock::now();
    const auto wall = [&] { return std::chrono::duration<double>(std::chrono::steady_clock::now() - clock).count(); };
    double gap = 0, peak = 0, overlap = 0, overlapPeak = 0; // перекрытие после успокоения и за весь опыт
    int frames = 0;
    Jitter jitter;
    std::vector<double> highest(s.boxes.size(), -1e300);
    for (; frames < int(s.seconds * 60) && wall() < kWallBudget; ++frames) {
        step(1.0 / 60);
        const std::vector<State> now = read();
        absorb(hash, now);
        if (!s.joints.empty()) gap = std::max(gap, jointGap(s, now));
        const bool settled = frames + 1 >= int(s.settle * 60);
        if (settled) jitter.add(s, now);
        if (s.joints.empty()) {
            const double deepest = deepestOverlap(s, now);
            overlapPeak = std::max(overlapPeak, deepest), overlap = settled ? std::max(overlap, deepest) : overlap;
        }
        for (size_t i = 0; i < s.boxes.size(); ++i)
            if (s.boxes[i].density != 0) highest[i] = std::max(highest[i], now[i].at.y), peak = std::max(peak, len(now[i].v) + len(now[i].w) * len(s.boxes[i].half));
    }
    const std::vector<State> end = read();
    double drift = 0, speed = 0, sink = 0, overshoot = 0;
    for (size_t i = 0; i < s.boxes.size(); ++i) {
        if (s.boxes[i].density == 0) continue;
        overshoot = std::max(overshoot, highest[i] - end[i].at.y); // выше итогового положения — вылет
        const V3 d = end[i].at - start[i].at;
        drift = std::max(drift, std::hypot(d.x, d.z)), speed = std::max(speed, len(end[i].v)), sink = std::max(sink, -d.y);
    }
    Outcome o{{{"drift_mm", 1e3 * drift}, {"sink_mm", 1e3 * sink}, {"speed_mm_s", 1e3 * speed}, {"energy_J", energy(s, end) - energy(s, start)},
               {"ms_frame", 1e3 * wall() / std::max(frames, 1)}, {"done_s", frames / 60.0}, {"jitter_mm_s", jitter.median()},
               {"jitter_peak_mm_s", jitter.peaks.empty() ? 0 : *std::max_element(jitter.peaks.begin(), jitter.peaks.end())},
               {"reversals", double(jitter.reversals)}, {"wakes", double(jitter.wakes)}, {"peak_mm_s", 1e3 * peak}, {"overshoot_mm", 1e3 * overshoot}}};
    if (!s.joints.empty()) o.numbers["gap_mm"] = 1e3 * gap;
    else o.numbers["overlap_mm"] = 1e3 * overlap, o.numbers["overlap_peak_mm"] = 1e3 * overlapPeak;
    if (s.pendulum) {
        const V3 r = end[1].at - s.boxes[0].at;
        o.numbers["angle_err_rad"] = std::fabs(std::atan2(r.x, -r.y) - pendulumAngle(2 * M_PI / 3, 0.4, 0.03, frames / 60.0));
    }
    return o;
}

// Строка журнала на сцену: у каждого числа — значения по вариантам, хэш — по всем вариантам.
inline void printRow(const std::string& engine, const SceneSpec& s, const std::vector<Outcome>& runs, std::uint64_t hash) {
    std::printf("{\"engine\":\"%s\",\"scene\":\"%s\",\"title\":\"%s\",\"variants\":%zu,\"numbers\":{", engine.c_str(), s.key, s.name, runs.size());
    const char* comma = "";
    for (const auto& [name, unused] : runs[0].numbers) {
        std::printf("%s\"%s\":[", comma, name.c_str());
        for (size_t k = 0; k < runs.size(); ++k) {
            const double v = runs[k].numbers.at(name); // взорвавшаяся сцена — законный итог: NaN по правилам JSON Python
            std::printf(std::isfinite(v) ? "%s%.9g" : "%sNaN", k ? "," : "", v);
        }
        std::printf("]"), comma = ",";
    }
    std::printf("},\"hash\":\"%016llx\"}\n", static_cast<unsigned long long>(hash));
    std::fflush(stdout);
}

// SCENE=ключ в окружении — прогнать одну сцену (быстрая проверка; табло гоняет все).
inline void compare(const std::string& engine, const Simulate& simulate) {
    const char* only = std::getenv("SCENE");
    for (const SceneSpec& scene : scenes()) {
        if (only && std::strcmp(only, scene.key) != 0) continue;
        std::vector<Outcome> runs;
        std::uint64_t hash = 14695981039346656037ull;
        for (int k = 0; k < kVariants; ++k) {
            const SceneSpec s = variant(scene, k);
            simulate(s, [&](const Step& step, const Read& read) { runs.push_back(play(s, step, read, hash)); });
        }
        printRow(engine, scene, runs, hash);
    }
}
