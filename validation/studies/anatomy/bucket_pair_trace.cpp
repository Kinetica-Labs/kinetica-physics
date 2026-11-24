// Полигональные части вёдер. stdout — минимальные входы и их геометрические результаты, stderr — сводка.
// Без аргументов — шаг SDK. --poses stock.csv — та же геометрия при сохранённых позах (17 значащих цифр).
// --replay witness.csv — восстановить только отмеченную пару в каждой строке, без шагов и истории контактов.
// --compare-poses stock.csv — A/B на фиксированных входах. --check-rest — критерии для свободной динамики.
#include "../../../examples/scenes/StandardScenes.h"
#include <collision/PairFinder.h>
#include <collision/narrowphase/HullVsHullAlgorithm.h>
#include <collision/narrowphase/Sat.h>
#include <math/WorkerPool.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <limits>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
using namespace kp;
constexpr int first = 60 * 60, last = 74 * 60;
constexpr double deg = 180 / 3.14159265358979323846;
constexpr double stationary = 256 * double(kEpsilon); // м для сдвига, рад для угла
using Poses = std::array<Transform, 4>;
using Key = PairFinder::Key;
using Pairs = std::map<Key, ContactPair>;
using Cells = std::vector<std::string>;

Cells split(const std::string& line) {
    Cells cells;
    size_t from = 0;
    for (;;) {
        const size_t end = line.find(',', from);
        cells.push_back(line.substr(from, end - from));
        if (end == line.npos) return cells;
        from = end + 1;
    }
}
Transform readPose(const Cells& c, size_t i) {
    const auto n = [&](size_t k) { return Real(std::stod(c.at(i + k))); };
    return {{n(0), n(1), n(2)}, {n(3), n(4), n(5), n(6)}};
}
void setPose(Body& body, Transform t) { body.position = t.position; body.orientation = t.orientation; }
double turn(Quaternion a, Quaternion b) {
    const Quaternion q = a * b.conjugate();
    return 2 * std::atan2(std::hypot(double(q.x), double(q.y), double(q.z)), std::abs(double(q.w)));
}
double angle(Vector3 a, Vector3 b) {
    return deg * std::atan2(double(a.cross(b).length()), double(a.dot(b)));
}
void xyz(Vector3 v) { std::printf(",%.17g,%.17g,%.17g", double(v.x), double(v.y), double(v.z)); }
void pose(Transform t) {
    xyz(t.position);
    std::printf(",%.17g,%.17g,%.17g,%.17g", double(t.orientation.w), double(t.orientation.x),
                double(t.orientation.y), double(t.orientation.z));
}
Body part(const World& world, int id, int index, Transform t) {
    auto body = world.bodies().at(size_t(id));
    if (index < 0 || index >= body.partCount()) throw std::runtime_error("invalid part index");
    setPose(body, t);
    return body.part(index);
}
struct Sample { int frame; Transform a, b; ContactPair pair; };
struct Flip {
    bool found = false;
    Key key{};
    Sample before{}, after{};
    double angleDeg{}, relativeMove{}, relativeTurn{};
};

// Поля SAT относятся к исходной оси до settleAxis. Итоговая ветвь определяется отдельным вызовом clipFaces.
const char* kind(SeparatingAxis::Kind k) {
    if (k == SeparatingAxis::Kind::FaceA) return "face_a";
    if (k == SeparatingAxis::Kind::FaceB) return "face_b";
    return "edge_pair";
}
std::string ids(const ContactManifold& m) {
    std::string s;
    for (int i = 0; i < m.count; ++i) {
        if (i) s += '|';
        s += std::to_string(m.points[i].id);
    }
    return s;
}
bool dump(const World& world, const char* name, const Flip& event, const Sample& sample) {
    const auto [a,b,ia,ib] = event.key;
    const Body pa = part(world, a, ia, sample.a), pb = part(world, b, ib, sample.b);
    const auto* ha = pa.shape->hull();
    const auto* hb = pb.shape->hull();
    if (!ha || !hb) return false;
    const auto& pair = sample.pair;
    const auto& m = pair.manifold;
    const auto repeated = examinePairWithin(pa, pb, pair.reach, kOnBoundary);
    if (!repeated || (repeated->manifold.normal - m.normal).length() != 0 ||
        repeated->distance != pair.distance || ids(repeated->manifold) != ids(m)) return false;
    const Real margin = kContactDistance + pair.reach;
    const auto clip = HullVsHullAlgorithm::clipFaces(*ha, pa.pose(), *hb, pb.pose(), margin, kContactDistance, kOnBoundary);
    const auto gjk = closestPoints(*pa.shape, pa.pose(), *pb.shape, pb.pose(), margin);
    const auto raw = findSeparatingAxis(TransformedHull(*ha, Transform{}),
                                        TransformedHull(*hb, pa.pose().relative(pb.pose())), kOnBoundary);
    const char* path = (m.normal - gjk.normal).length() == 0 ? "fallback_matches_gjk_normal" : "fallback_other_normal";
    if (clip.count > 0) {
        if ((clip.normal - m.normal).length() != 0 || ids(clip) != ids(m)) path = "modified_clip";
        else if (clip.faceOnB) path = "clip_face_b";
        else if ((clip.points[0].id & 0xc0000000u) == 0xc0000000u) path = "clip_edges";
        else path = "clip_face_a";
    }
    std::printf("%s,%d,%d,%d,%d,%d", name, sample.frame, a, b, ia, ib);
    pose(sample.a); pose(sample.b);
    std::printf(",%.17g", double(pair.reach)); xyz(m.normal);
    std::printf(",%d,%.17g,%s,%.17g,%.17g,%.17g,%s,%d,%zu,%zu,%zu,%zu,%zu,%zu,%.17g,%s,%d,%d,%.17g",
                m.count, double(pair.distance), ids(m).c_str(), event.angleDeg, event.relativeMove, event.relativeTurn,
                path, clip.count, ha->vertices().size(), ha->faces().size(), ha->edges().size(),
                hb->vertices().size(), hb->faces().size(), hb->edges().size(), double(m.normal.length()),
                kind(raw.kind), raw.featureA, raw.featureB, double(raw.separation));
    xyz(pa.pose().direction(raw.normal));
    std::printf(",%.17g,%d,%d", double(gjk.distance), int(gjk.coresOverlap), gjk.iterations);
    xyz(gjk.normal);
    Real minimum = m.points[0].separation, maximum = minimum;
    for (const auto& p : m.points) { minimum = std::min(minimum, p.separation); maximum = std::max(maximum, p.separation); }
    std::printf(",%.17g,%.17g", double(minimum), double(maximum));
    xyz(clip.normal); xyz(gjk.onA); xyz(gjk.onB); std::putchar('\n');
    return true;
}

struct Scan {
    Poses previous{}, start{};
    Pairs previousPairs;
    Flip firstTurn{}, firstReverse{}, largest{};
    int frames{}, compared{}, quietPairs{}, tilted{};
    int appeared{}, disappeared{}, maxMatchedCountChange{}, maxCountChange{};
    int minPoints = std::numeric_limits<int>::max(), maxPoints{}, archiveFrames{}, differingFrames{};
    int minArchive = std::numeric_limits<int>::max(), maxArchive{}, minDelta = std::numeric_limits<int>::max();
    int maxDelta = std::numeric_limits<int>::min(), missingSupportFrames{};
    double maxGapResidual{}, maxComMove{}, maxBodyTurn{}, maxPointSpeed{};
    std::uint64_t poseHash = UINT64_C(14695981039346656037);

    // Одинаковый дайджест проверяет совпадение заданных поз всех тел на всех 841 кадрах A/B.
    bool recordBody(const World& world, int i, const Transform& t) {
        for (double x : {double(t.position.x),double(t.position.y),double(t.position.z),
                         double(t.orientation.w),double(t.orientation.x),double(t.orientation.y),double(t.orientation.z)}) {
            if (!std::isfinite(x)) return false;
            std::uint64_t bits;
            static_assert(sizeof(bits) == sizeof(x));
            std::memcpy(&bits, &x, sizeof(bits));
            for (int b = 0; b < 8; ++b) {
                poseHash ^= (bits >> (8*b)) & 255u;
                poseHash *= UINT64_C(1099511628211);
            }
        }
        const auto q = t.orientation;
        if (std::abs(double(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z)-1) > 1e-7) return false;
        if (frames == 0) start[i] = t;
        maxComMove = std::max(maxComMove, double((t.position - start[i].position).length()));
        maxBodyTurn = std::max(maxBodyTurn, turn(t.orientation, start[i].orientation) * deg);
        const auto& body = world.bodies()[size_t(i)];
        const double speed = double(body.linearVelocity().length()) + double(body.boundingRadius()) * double(body.angularVelocity().length());
        if (!std::isfinite(speed)) return false;
        maxPointSpeed = std::max(maxPointSpeed, speed);
        return true;
    }

    bool observe(const World& world, int frame, int expectedPoints = -1, bool strict = true) {
        if (frame != first + frames) return false;
        Poses now{};
        Pairs pairs;
        int points = 0;
        bool support[3]{};
        for (int i = 0; i < 4; ++i) {
            now[i] = frameOf(world.bodies()[size_t(i)]);
            if (!recordBody(world, i, now[i])) return false;
        }
        for (const auto& pair : PairFinder(world.bodies(), world.gravity).pairs(0)) {
            if (!pair.inContact() || !pair.manifold.count) continue;
            if (pair.a >= 0 && pair.a < 3 && pair.b == pair.a + 1) support[pair.a] = true;
            points += pair.manifold.count;
            for (const auto& p : pair.manifold.points) {
                const double residual = std::abs(double(pair.manifold.normal.dot(p.onB-p.onA) - p.separation));
                if (!std::isfinite(residual)) return false;
                maxGapResidual = std::max(maxGapResidual, residual);
            }
            if (!pairs.emplace(Key{pair.a,pair.b,pair.partA,pair.partB}, pair).second) return false;
        }
        missingSupportFrames += !(support[0] && support[1] && support[2]);
        minPoints = std::min(minPoints, points); maxPoints = std::max(maxPoints, points);
        if (expectedPoints >= 0) {
            ++archiveFrames;
            differingFrames += expectedPoints != points;
            minArchive = std::min(minArchive, expectedPoints); maxArchive = std::max(maxArchive, expectedPoints);
            minDelta = std::min(minDelta, points - expectedPoints); maxDelta = std::max(maxDelta, points - expectedPoints);
        }
        if (strict && expectedPoints >= 0 && expectedPoints != points) {
            std::fprintf(stderr, "frame=%d expected_points=%d actual_points=%d\n", frame, expectedPoints, points);
            return false;
        }
        for (const auto& [key, current] : pairs) {
            const auto it = previousPairs.find(key);
            if (it == previousPairs.end()) {
                if (frames) { ++appeared; maxCountChange = std::max(maxCountChange, current.manifold.count); }
                continue;
            }
            ++compared;
            const int change = std::abs(current.manifold.count - it->second.manifold.count);
            maxCountChange = std::max(maxCountChange, change);
            maxMatchedCountChange = std::max(maxMatchedCountChange, change);
            const auto [a,b,ia,ib] = key;
            const Transform oldRel = previous[a].relative(previous[b]), newRel = now[a].relative(now[b]);
            const double dp = double((newRel.position - oldRel.position).length());
            const double dq = turn(oldRel.orientation, newRel.orientation);
            if (dp > stationary || dq > stationary || turn(now[a].orientation, previous[a].orientation) > stationary ||
                turn(now[b].orientation, previous[b].orientation) > stationary) continue;
            ++quietPairs;
            const auto& old = it->second;
            const double rotation = angle(old.manifold.normal, current.manifold.normal);
            if (!std::isfinite(rotation) || std::abs(double(old.manifold.normal.length())-1) > 1e-8 ||
                std::abs(double(current.manifold.normal.length())-1) > 1e-8) return false;
            const bool earlier = rotation >= 5 && !firstTurn.found, reverse = rotation >= 90 && !firstReverse.found;
            const bool higher = !largest.found || rotation > largest.angleDeg;
            tilted += rotation >= 5;
            if (!earlier && !reverse && !higher) continue;
            const Flip e{true, key, {frame-1,previous[a],previous[b],old}, {frame,now[a],now[b],current}, rotation,dp,dq};
            if (earlier) firstTurn = e;
            if (reverse) firstReverse = e;
            if (higher) largest = e;
        }
        for (const auto& [key, old] : previousPairs)
            if (pairs.find(key) == pairs.end()) { ++disappeared; maxCountChange = std::max(maxCountChange, old.manifold.count); }
        previousPairs = std::move(pairs);
        previous = now;
        ++frames;
        return true;
    }
};

bool fromArchive(World& world, const char* path, Scan& scan, bool strict) {
    std::ifstream f(path);
    std::string line;
    if (!std::getline(f,line) || line.find("mode,frame,time_s,body,com_x_m") != 0) return false;
    int body = 1, points = 0;
    while (std::getline(f,line)) {
        const auto c = split(line);
        const int frame = std::stoi(c.at(1));
        if (frame < first) continue;
        if (frame > last) break;
        if (c.at(0) != "stock" || frame != first + scan.frames || std::stoi(c.at(3)) != body ||
            std::abs(std::stod(c.at(2)) - double(frame)/60) > 1e-9) return false;
        setPose(world.body(BodyId(body)), readPose(c, 4));
        const int n = std::stoi(c.at(21));
        if (body == 1) points = n;
        if (n != points) return false;
        if (++body == 4) {
            if (!scan.observe(world, frame, points, strict)) return false;
            body = 1;
        }
    }
    return body == 1 && scan.frames == last - first + 1;
}

bool replay(const World& world, const char* path, bool checkWall) {
    std::ifstream f(path);
    std::string line;
    if (!std::getline(f,line) || line.find("case,frame,body_a,body_b,part_a,part_b,") != 0) return false;
    int rows = 0;
    while (std::getline(f,line)) {
        const auto c = split(line);
        const Body pa = part(world, std::stoi(c.at(2)), std::stoi(c.at(4)),readPose(c, 6));
        const Body pb = part(world, std::stoi(c.at(3)), std::stoi(c.at(5)),readPose(c, 13));
        const Real reach = Real(std::stod(c.at(20)));
        const auto hit = examinePairWithin(pa, pb, reach, kOnBoundary);
        if (checkWall) {
            // Нормаль стенки правильного 16-угольного усечённого конуса (сцена buckets).
            const double slope = ((.20 - .15) / .30) * std::cos(3.14159265358979323846 / 16);
            const double expected = slope / std::hypot(1., slope);
            const auto swapped = examinePairWithin(pb, pa, reach, kOnBoundary);
            if (!hit || !swapped || hit->manifold.count == 0 || swapped->manifold.count == 0) return false;
            std::printf("%s,%s,%.17g,%.17g,%.17g\n", c[0].c_str(), c[1].c_str(), expected,
                        double(hit->manifold.normal.y), double(swapped->manifold.normal.y));
            if (!(std::abs(hit->manifold.normal.y - expected) <= 1e-10 &&
                  std::abs(swapped->manifold.normal.y + expected) <= 1e-10)) return false;
            ++rows;
            continue;
        }
        const Vector3 saved{Real(std::stod(c.at(21))), Real(std::stod(c.at(22))), Real(std::stod(c.at(23)))};
        if (!hit || (hit->manifold.normal - saved).length() != 0 || hit->manifold.count != std::stoi(c.at(24)) ||
            hit->distance != Real(std::stod(c.at(25))) || ids(hit->manifold) != c.at(26)) return false;
        ++rows;
    }
    std::fprintf(stderr, "replayed_rows=%d check_wall=%d\n", rows, int(checkWall));
    return checkWall ? rows == 4 : rows >= 2;
}
int run(World world, int argc, char** argv) {
    if (world.bodies().size() != 4) return 2;
    Scan scan;
    const std::string mode = argc > 1 ? argv[1] : "";
    if (argc == 3 && (mode == "--replay" || mode == "--check-walls"))
        return replay(world, argv[2], mode == "--check-walls") ? 0 : 1;
    if (argc == 3 && (mode == "--poses" || mode == "--compare-poses")) {
        if (!fromArchive(world, argv[2], scan, mode == "--poses")) return 1;
    } else if (argc == 1 || (mode == "--check-rest" && (argc == 2 || (argc == 3 && std::string(argv[2]) == "--shock-off")))) {
        if (argc == 3) world.shockPropagation = false;
        world.realtime = true; // штатные 8 подшагов, 4 обновления, сон и демпфирование выключены
        for (int frame = 1; frame <= last; ++frame) {
            world.step(Real(1)/60);
            if (world.report().stalled || world.report().unfinished != 0) return 1;
            if (frame >= first && !scan.observe(world, frame)) return 1;
        }
    } else return 2;
    std::puts("case,frame,body_a,body_b,part_a,part_b,ax_m,ay_m,az_m,aqw,aqx,aqy,aqz,bx_m,by_m,bz_m,bqw,bqx,bqy,bqz,"
              "reach_m,nx,ny,nz,points,pair_distance_m,point_ids,transition_deg,relative_translation_m,relative_rotation_rad,"
              "manifold_path,clip_count,hull_a_verts,hull_a_faces,hull_a_edges,hull_b_verts,hull_b_faces,hull_b_edges,"
              "normal_length,raw_sat_kind,raw_sat_feature_a,raw_sat_feature_b,raw_sat_gap_m,raw_sat_nx,raw_sat_ny,raw_sat_nz,"
              "gjk_distance_m,gjk_cores_overlap,gjk_iterations,gjk_nx,gjk_ny,gjk_nz,min_point_separation_m,max_point_separation_m,"
              "clip_nx,clip_ny,clip_nz,gjk_ax_m,gjk_ay_m,gjk_az_m,gjk_bx_m,gjk_by_m,gjk_bz_m");
    const auto emit = [&](const char* tag, const Flip& e) {
        return !e.found || (dump(world,tag,e,e.before) && dump(world,tag,e,e.after));
    };
    if (!emit("first_5deg", scan.firstTurn) || !emit("first_90deg", scan.firstReverse) || !emit("largest", scan.largest)) return 1;
    std::fprintf(stderr, "complete=1 frames=%d matched_transitions=%d stationary_transitions=%d at_least_5deg=%d first_90deg=%d max_deg=%.12g\n",
                 scan.frames,scan.compared,scan.quietPairs,scan.tilted,int(scan.firstReverse.found),scan.largest.angleDeg);
    std::fprintf(stderr, "pose_fnv64=%016llx actual_points_min=%d actual_points_max=%d appeared_pairs=%d disappeared_pairs=%d "
                 "max_matched_pair_point_count_change=%d max_pair_count_change_including_absent=%d max_gap_residual_m=%.17g "
                 "missing_support_frames=%d max_from_60s_m=%.12g max_from_60s_deg=%.12g\n",
                 (unsigned long long)scan.poseHash,scan.minPoints,scan.maxPoints,scan.appeared,scan.disappeared,scan.maxMatchedCountChange,
                 scan.maxCountChange,scan.maxGapResidual,scan.missingSupportFrames,scan.maxComMove,scan.maxBodyTurn);
    if (scan.archiveFrames)
        std::fprintf(stderr, "archive_frames=%d different_point_count_frames=%d archive_min=%d archive_max=%d "
                     "actual_minus_archive_min=%d actual_minus_archive_max=%d\n", scan.archiveFrames,scan.differingFrames,
                     scan.minArchive,scan.maxArchive,scan.minDelta,scan.maxDelta);
    if (mode == "--check-rest") {
        const bool pass = scan.frames == last-first+1 && scan.missingSupportFrames == 0 && scan.quietPairs > 0 &&
            scan.tilted == 0 && scan.maxComMove <= 0.001 && scan.maxBodyTurn <= 1 && scan.maxPointSpeed <= 0.02 &&
            scan.maxGapResidual <= 1e-9;
        std::fprintf(stderr,"dynamic_rest_pass=%d max_point_speed_m_s=%.12g\n", int(pass),scan.maxPointSpeed);
        return pass ? 0 : 1;
    }
    return 0; // отсутствие разворота — результат измерения
}
}
int main(int argc, char** argv) {
    const char* threads = std::getenv("KP_THREADS");
    kp::WorkerPool::shared().resize(threads ? std::max(1, std::atoi(threads)) : 1);
    try {
        for (const auto& s : standardScenes()) if (std::string(s.key) == "buckets") return run(s.build(), argc, argv);
    } catch (const std::exception& e) { std::fprintf(stderr, "%s\n", e.what()); }
    return 2;
}
