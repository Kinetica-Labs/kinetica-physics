// Прибор стандартного набора сцен (examples/scenes): каждая сцена считается шагами по 1/60 с, и
// записывается, чем она кончилась и чего стоила.
//   study-standard-scenes [scenes.jsonl [предел времени на сцену, с [имя сцены]]]
// С переменной окружения KP_SLEEP=1 сцены считаются со сном островов (World::sleeping).
// Мерки те же, что в таблице случаев (CaseTable.cpp):
//   перекрытие — наибольшая глубина, на которую тела вошли друг в друга, по всем кадрам;
//   энергия    — наибольший подъём механической энергии над уже достигнутым низшим уровнем, доля
//                от начальной энергии сцены (кинетической плюс m g R по всем телам);
//   шаг встал  — число кадров, в которых шаг остановился раньше времени (World::Report::stalled).
// Сцена, не уложившаяся в предел времени, обрывается: записывается, сколько кадров она успела.
// Рядом с замерами пишется файл <scenes>.shapes.jsonl — очертания тел каждой сцены в начале и в
// момент проверки, сбоку (вдоль оси z) и с торца (вдоль оси x): по ним строится рисунок сцен.

#include <kp/collision/PairFinder.h>
#include "../../examples/scenes/StandardScenes.h"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

using namespace kp;

namespace {

// Спящее тело на время сна неподвижно; его потенциальная энергия считается по настоящей массе.
double energy(const World& world) {
    double total = 0;
    for (size_t i = 0; i < world.bodies().size(); ++i) {
        const Body& b = world.bodies()[i];
        const Real inverseMass = world.mass(BodyId(int(i))).inverseMass;
        if (inverseMass != 0) total += double(b.kineticEnergy()) - double(world.gravity.dot(b.position) / inverseMass);
    }
    return total;
}

double deepestOverlap(const World& world) {
    double deepest = 0;
    for (const ContactPair& pair : PairFinder(world.bodies(), world.gravity).pairs(0))
        if (pair.inContact()) deepest = std::fmax(deepest, double(overlapDepth(world.bodies()[size_t(pair.a)].part(pair.partA), world.bodies()[size_t(pair.b)].part(pair.partB))));
    return deepest;
}

// Очертание выпуклой формы в плоскости (u, v): опорные точки по 24 направлениям в этой плоскости.
// Проекция выпуклого тела выпукла, и её опорная точка — проекция опорной точки самого тела.
void outline(std::FILE* file, const Body& part, Vector3 u, Vector3 v) {
    std::fprintf(file, "[");
    for (int k = 0; k < 24; ++k) {
        const double angle = k * 3.14159265358979323846 / 12;
        const Vector3 d = Real(std::cos(angle)) * u + Real(std::sin(angle)) * v;
        const Vector3 p = part.pose().point(part.shape->support(part.pose().localDirection(d))) + part.shape->radius * d;
        std::fprintf(file, "%s[%.4f,%.4f]", k ? "," : "", double(p.dot(u)), double(p.dot(v)));
    }
    std::fprintf(file, "]");
}

void shapes(std::FILE* file, const char* scene, const char* when, const World& world) {
    if (!file) return;
    for (int view = 0; view < 2; ++view) {
        std::fprintf(file, "{\"scene\":\"%s\",\"when\":\"%s\",\"view\":\"%s\",\"parts\":[", scene, when, view ? "end" : "side");
        bool first = true;
        for (size_t i = 0; i < world.bodies().size(); ++i) {
            const Body& body = world.bodies()[i];
            for (int p = 0; body.shape && p < body.partCount(); ++p) {
                std::fprintf(file, "%s{\"fixed\":%s,\"outline\":", first ? "" : ",", body.isStatic() ? "true" : "false");
                outline(file, body.part(p), view ? Vector3{0, 0, 1} : Vector3{1, 0, 0}, {0, 1, 0});
                std::fprintf(file, "}");
                first = false;
            }
        }
        std::fprintf(file, "]}\n");
    }
    std::fflush(file);
}

} // namespace

int main(int argc, char** argv) {
    std::FILE* rows = argc > 1 ? std::fopen(argv[1], "w") : nullptr;
    const double budget = argc > 2 ? std::atof(argv[2]) : 120;
    std::FILE* drawn = rows ? std::fopen((std::string(argv[1]) + ".shapes.jsonl").c_str(), "w") : nullptr;
    std::printf("%-42s %5s %5s %6s %9s %9s %8s %9s %9s %5s  %s\n", "сцена", "тел", "част.", "кадров", "мс/кадр", "худший мс", "отрезков", "перекр. мм", "энергия", "встал", "итог");
    for (const StandardScene& scene : standardScenes()) {
        if (argc > 3 && std::strcmp(argv[3], scene.key) != 0) continue;
        World world = scene.build();
        world.sleeping = std::getenv("KP_SLEEP") != nullptr;
        world.realtime = std::getenv("KP_REALTIME") != nullptr;
        const World start = world;
        shapes(drawn, scene.key, "start", world);
        int parts = 0;
        double scale = 0;
        for (const Body& b : world.bodies()) {
            parts += b.partCount();
            if (!b.isStatic()) scale += double(b.kineticEnergy()) + double(world.gravity.length()) * double(b.boundingRadius()) / double(b.mass.inverseMass);
        }
        const int frames = int(std::lround(scene.seconds * 60)), checkFrame = int(std::lround(scene.checkAt * 60));
        double lowest = energy(world), gain = 0, deepest = 0, slowest = 0, total = 0;
        long segments = 0;
        int most = 0, stalled = 0, done = 0;
        std::string verdict = "не досчитана до проверки";
        for (int frame = 0; frame < frames && total < budget; ++frame) {
            if (scene.control) scene.control(world, frame / 60.0);
            const auto began = std::chrono::steady_clock::now();
            world.step(Real(1) / 60);
            const double took = std::chrono::duration<double>(std::chrono::steady_clock::now() - began).count();
            total += took, slowest = std::fmax(slowest, took);
            segments += world.report().segments, most = std::max(most, world.report().segments), stalled += world.report().stalled;
            const double now = energy(world);
            gain = std::fmax(gain, (now - lowest) / scale), lowest = std::fmin(lowest, now);
            deepest = std::fmax(deepest, deepestOverlap(world));
            ++done;
            if (done == checkFrame) verdict = scene.verdict(start, world), shapes(drawn, scene.key, "check", world);
        }
        if (verdict.empty() && stalled > 0) verdict = "проверка пройдена, но шаг вставал: мир досчитан не полностью";
        const bool ok = verdict.empty() && done == frames;
        std::printf("%-42s %5zu %5d %6d %9.2f %9.1f %8.1f %9.4f %9.1e %5d  %s\n", scene.title, world.bodies().size(), parts, done, 1000 * total / std::max(done, 1), 1000 * slowest, double(segments) / std::max(done, 1),
                    1000 * deepest, gain, stalled, verdict.empty() ? (done == frames ? "как должно" : "проверка пройдена, сцена не досчитана") : verdict.c_str());
        std::fflush(stdout);
        if (rows) {
            std::fprintf(rows, "{\"scene\":\"%s\",\"title\":\"%s\",\"bodies\":%zu,\"parts\":%d,\"joints\":%zu,\"frames\":%d,\"frames_planned\":%d,\"ms_per_frame\":%.4f,\"slowest_ms\":%.3f,"
                               "\"segments_per_frame\":%.3f,\"most_segments\":%d,\"overlap_m\":%.3e,\"energy_gain\":%.3e,\"stalled\":%d,\"asleep\":%d,\"verdict\":\"%s\",\"ok\":%s}\n",
                         scene.key, scene.title, world.bodies().size(), parts, world.joints().size(), done, frames, 1000 * total / std::max(done, 1), 1000 * slowest, double(segments) / std::max(done, 1), most, deepest,
                         gain, stalled, world.report().asleep, verdict.c_str(), ok ? "true" : "false");
            std::fflush(rows);
        }
    }
    if (rows) std::fclose(rows);
    if (drawn) std::fclose(drawn);
}
