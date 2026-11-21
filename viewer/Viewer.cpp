#include "Viewer.h"

#include <algorithm>
#include <chrono>
#include <cmath>

using namespace kp;

void Viewer::load(size_t index) {
    scene = index;
    world = scenes()[index].build();
    world.sleeping = sleeping;
    grab = {};
    placing = -1;
    camera.target = scenes()[index].lookAt;
    camera.distance = scenes()[index].distance;
    frame = mostSegments = 0;
    lag = 0;
    lastStep = slowestStep = 0;
    worstError = worstGap = 0;
    judged = false, verdict.clear();
}

void Viewer::step() {
    world.shockPropagation = shockPropagation;
    world.maxSegments = segmentLimit;
    world.realtime = realtime;
    world.soft.rollingResistance = Real(rollingResistance) / 1000;
    // Порог сна у шага с проходами — как у Box3D, 5 см/с: остаточная дрожь мягких контактов ниже
    // него. Точному шагу хватает десятой доли миллиметра в секунду.
    world.sleepSpeed = realtime ? Real(0.05) : Real(1e-4);
    // Время до сна — 2 с, как у Bullet: за полсекунды засыпал куб, поставленный на ребро с наклоном до
    // 0,1° от равновесия; за 2 с он успевает упасть уже с 0,001°.
    world.sleepTime = 2;
    const Real drag = scenes()[scene].exact ? Real(0) : Real(damping);
    for (size_t i = 0; i < world.bodies().size(); ++i) world.body(BodyId(int(i))).linearDamping = world.body(BodyId(int(i))).angularDamping = drag;
    if (scenes()[scene].control) scenes()[scene].control(world, frame * double(kFrame));
    world.soft.contactUpdates = contactUpdates, world.soft.substeps = substeps;
    for (const ImpulseTool& tool : impulses)
        if (tool.everyFrame) tool.fire(world, kFrame); // постоянная сила: импульс «сила × время кадра»
    const auto start = std::chrono::steady_clock::now();
    world.step(kFrame);
    lastStep = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
    slowestStep = std::max(slowestStep, lastStep);
    mostSegments = std::max(mostSegments, world.report().segments);
    lag += double(world.report().unfinished);
    ++frame;
    worstGap = std::max(worstGap, double(world.joints().largestError(world.bodies())));
    if (const StandardScene* standard = scenes()[scene].standard; standard && !judged && frame * double(kFrame) >= standard->checkAt)
        verdict = standard->verdict(standard->build(), world), judged = true;
    if (const ExactScene* exact = scenes()[scene].exact)
        worstError = std::max(worstError, std::abs(exact->measured(world) - exact->exact((frame * double(kFrame)) - lag)));
}

bool Viewer::asMeasured() const {
    const SoftStep standard;
    return damping == 0 && !sleeping && rollingResistance == 0 && !shockPropagation && substeps == standard.substeps && contactUpdates == standard.contactUpdates;
}

void Viewer::measure() {
    const SoftStep standard;
    damping = rollingResistance = 0, sleeping = shockPropagation = false;
    substeps = standard.substeps, contactUpdates = standard.contactUpdates;
    load(scene);
}

bool Viewer::placeAt(Vector3 direction) {
    if (placing < 0) return false;
    if (const RayHit hit = castRay(world, {camera.eye(), direction})) impulses[size_t(placing)].center = hit.point;
    placing = -1;
    return true;
}

void Viewer::fireAll() {
    for (ImpulseTool& tool : impulses)
        if (!tool.everyFrame) tool.touched = tool.fire(world);
}

bool Viewer::grabAt(Vector3 direction) {
    const RayHit hit = castRay(world, {camera.eye(), direction});
    if (!hit) return false;
    world.wake(BodyId(hit.body)); // спящее тело на время сна неподвижно: сначала разбудить
    if (world.bodies()[size_t(hit.body)].isStatic()) return false;
    // Второй конец привязи держит неподвижное тело сцены; если такого нет — тело без формы.
    int hand = 0;
    while (hand < int(world.bodies().size()) && (!world.bodies()[size_t(hand)].isStatic() || world.asleep(BodyId(hand)) || isDriven(world.bodies()[size_t(hand)]))) ++hand;
    if (hand == int(world.bodies().size())) world.addBody(Body{});
    const BodyId held = BodyId(hit.body);
    const double mass = 1 / double(world.body(held).mass.inverseMass), omega = 2 * 3.14159265358979323846 * kGrabFrequency;
    const TetherJoint tether(hand, world.body(BodyId(hand)), hit.body, world.body(held), hit.point, hit.point, Real(mass * omega * omega), Real(2 * mass * omega));
    if (grab.tether) *grab.tether = tether;
    else grab.tether = &addTether(world, BodyId(hand), hit.point, held, hit.point, tether.stiffness, tether.damping);
    grab.body = hit.body;
    grab.depth = double(camera.axes().forward.dot(hit.point - camera.eye()));
    return true;
}

void Viewer::dragTo(Vector3 direction) {
    if (grab.body < 0) return;
    TetherJoint& tether = *grab.tether;
    const Vector3 target = camera.eye() + Real(grab.depth / double(direction.dot(camera.axes().forward))) * direction;
    tether.anchorA = world.body(BodyId(tether.a)).pose().localPoint(target);
}

void Viewer::release() {
    if (grab.body < 0) return;
    grab.tether->stiffness = grab.tether->damping = 0;
    grab.body = -1;
}

DebugGeometry Viewer::visible(const DebugGeometry& all) const {
    DebugGeometry shown;
    for (const DebugLine& line : all.lines)
        if (show[int(line.kind)]) shown.lines.push_back(line);
    for (const DebugPoint& point : all.points)
        if (show[int(point.kind)]) shown.points.push_back(point);
    for (const ImpulseTool& tool : impulses) tool.draw(shown);
    if (grab.body >= 0) {
        const TetherJoint& tether = *grab.tether;
        const Vector3 from = world.bodies()[size_t(tether.b)].pose().point(tether.anchorB), to = world.bodies()[size_t(tether.a)].pose().point(tether.anchorA);
        shown.lines.push_back({from, to, DebugKind::ClosestPoints});
        shown.points.push_back({from, DebugKind::ClosestPoints}), shown.points.push_back({to, DebugKind::ClosestPoints});
    }
    return shown;
}
