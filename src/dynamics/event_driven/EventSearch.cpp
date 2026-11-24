#include <math/StepMemory.h>
#include <dynamics/event_driven/EventSearch.h>

#include <collision/narrowphase/Gjk.h>
#include <collision/narrowphase/TimeOfImpact.h>
#include <collision/shapes/ConvexHull.h>
#include <collision/shapes/Cylinder.h>
#include <joints/SingleFreedomJoint.h>
#include <dynamics/Integrator.h>

#include <algorithm>
#include <limits>

namespace kp {
namespace {

// Сколько шагов делает продвижение к моменту удара, прежде чем уступить поиску с доказанной
// границей (TimeOfImpact.h).
constexpr int kAdvancePatience = 48;

// Когда у касающейся пары коснётся ещё одна точка. Расстояние между такими телами нулевое и о
// новых точках ничего не говорит, поэтому за каждой точкой многообразия, которая ещё не в полосе
// касания, слежение отдельное. Такая точка — вершина одного тела над гранью другого; зазор —
// расстояние от вершины до плоскости грани, и обе движутся вместе со своими телами.
// bodiesAt(t) — оба тела через время t; возвращает момент касания или horizon.
template <class BodiesAt>
Real nextPointContact(const ContactPair& pair, const Body& a, const Body& b, BodiesAt&& bodiesAt, Real closingSpeed, Real horizon) {
    const ContactManifold& manifold = pair.manifold;
    const Vector3 localNormal = (manifold.faceOnB ? b : a).pose().localDirection(manifold.normal);
    for (int i = 0; i < manifold.count; ++i) {
        if (manifold.points[i].separation <= kContactDistance) continue;
        const Vector3 localA = a.pose().localPoint(manifold.points[i].onA), localB = b.pose().localPoint(manifold.points[i].onB);
        const auto gapAt = [&](Real t) {
            const auto [at, bt] = bodiesAt(t);
            const Vector3 normal = (manifold.faceOnB ? bt : at).pose().direction(localNormal);
            return normal.dot(bt.pose().point(localB) - at.pose().point(localA));
        };
        // У круглых форм зазор точки меряется между поверхностями вдоль своей нормали и с этим
        // зазором может расходиться: точка «ещё не касается», а здесь зазор уже в полосе. Следить за
        // такой точкой нечем — слежение каждый раз отвечало бы «касание сейчас», и шаг вставал
        // (две капсулы в куче тел). Перекрытия у неё не допустит поиск по истинному расстоянию.
        if (gapAt(0) <= kContactDistance) continue;
        horizon = std::min(horizon, timeOfImpact(gapAt, closingSpeed, horizon, kContactDistance));
    }
    return horizon;
}

// Границы движения тела на отрезке. Силы на отрезке постоянны, поэтому импульсы линейны по
// времени, а их величины не больше, чем в начале отрезка или в его конце.
struct MotionBounds {
    Vector3 acceleration; // ускорение центра масс, постоянное
    Real speed = 0;    // |v| не больше
    Real spin = 0;     // |ω| не больше, см. spinBound
    Real spinUp = 0;   // |dω/dt| не больше: dω/dt = I⁻¹ (τ − ω × L)
    Real radius = 0;   // охватывающий радиус
};

// Наибольшая угловая скорость тела, если квадрат его момента импульса не больше momentum², а
// удвоенная энергия вращения не больше twiceEnergy. В осях тела ω_k = L_k / I_k; для x_k = L_k²
// это наибольшее значение Σ x_k / I_k² при x >= 0, Σ x_k <= L², Σ x_k / I_k <= 2T — линейная
// задача, её ответ лежит в вершине многогранника условий. Одного момента импульса мало: у
// стержня он дал бы скорость «вокруг длинной оси» в сотню раз больше настоящей, и только энергия
// говорит, что стержень кувыркается, а не вертится вокруг оси.
Real spinBound(Vector3 inverse, Real momentumSquared, Real twiceEnergy) {
    const Real j[3] = {inverse.x, inverse.y, inverse.z};
    Real best = 0;
    for (int i = 0; i < 3; ++i) {
        best = std::max(best, j[i] * j[i] * std::min(momentumSquared, twiceEnergy / j[i]));
        for (int k = i + 1; k < 3; ++k) {
            if (j[i] == j[k]) continue;
            const Real xi = (twiceEnergy - j[k] * momentumSquared) / (j[i] - j[k]), xk = momentumSquared - xi;
            if (xi >= 0 && xk >= 0) best = std::max(best, j[i] * j[i] * xi + j[k] * j[k] * xk);
        }
    }
    return sqrt(best);
}

MotionBounds motionBounds(const Body& body, Vector3 force, Vector3 torque, Real h) {
    MotionBounds bounds;
    bounds.radius = body.boundingRadius();
    if (body.isStatic()) {
        if (isDriven(body)) bounds.speed = body.drivenVelocity.length(), bounds.spin = body.drivenSpin.length(); // скорости постоянны: ускорений нет
        return bounds;
    }
    Body later = body;
    integrateVelocity(later, force, torque, h);
    const Vector3 inverse = body.mass.inverseInertia;
    const Real stiffest = std::max({inverse.x, inverse.y, inverse.z});
    const Real momentum = std::max(body.angularMomentum.length(), later.angularMomentum.length());
    bounds.acceleration = body.mass.inverseMass * force;
    bounds.speed = std::max(body.linearVelocity().length(), later.linearVelocity().length());
    // Энергия вращения растёт не быстрее мощности момента сил: dT/dt = τ·ω <= |τ| |ω|.
    const Real twiceEnergy = body.angularMomentum.dot(body.angularVelocity()) + 2 * torque.length() * stiffest * momentum * h;
    bounds.spin = spinBound(inverse, momentum * momentum, twiceEnergy);
    bounds.spinUp = stiffest * (bounds.spin * momentum + torque.length());
    return bounds;
}

// Верхняя граница второй производной зазора вдоль направления n, неподвижного в осях тела frame.
// В этих осях точки frame стоят, а точка q другого тела движется с ускорением
//   R_fᵀ (b̈ − ẍ_f − α_f × ρ − 2 ω_f × (ḃ − ẋ_f) + ω_f × (ω_f × ρ)),   ρ = b − x_f,
// где b̈ = ẍ_m + α_m × r + ω_m × (ω_m × r). Разность ускорений центров постоянна; если frame не
// вращается, направление n неподвижно, и от неё нужна только составляющая вдоль n, причём только
// раскрывающая зазор: прогиб ниже хорды даёт положительная вторая производная. reach — наибольшее
// расстояние от центра frame до точек другого тела.
Real gapCurvature(const MotionBounds& frame, const MotionBounds& moving, Vector3 n, Vector3 opening, Real reach) {
    const Real centres = frame.spin == 0 && frame.spinUp == 0 ? std::max(Real(0), n.dot(opening)) : opening.length();
    return centres + (moving.spinUp + moving.spin * moving.spin) * moving.radius + (frame.spinUp + frame.spin * frame.spin) * reach +
           2 * frame.spin * (frame.speed + moving.speed + moving.spin * moving.radius);
}

// Расстояние между телами и направление, зазор вдоль которого к нему ближе всего.
//
// Зазор вдоль направления — нижняя граница расстояния, и точна она только вдоль истинной нормали.
// Нормаль из ближайших точек у почти касающихся тел шумит: это направление вектора длиной в
// микроны. У длинного тела шум в тысячные радиана стоит десятых долей миллиметра: опорная
// плоскость упирается в дальний край цилиндра. У острой кромки и ребра нормали нет вовсе —
// годится любое направление между нормалями двух поверхностей, которые в них сходятся. Поэтому
// к найденной нормали примеряются естественные направления самих тел — ось цилиндра и его радиус
// в сторону второго тела, нормали двух граней многогранника, ближайших к найденной нормали, —
// и берётся то, вдоль которого зазор наибольший. Любое направление даёт верную нижнюю границу,
// так что выбор лучшего её только уточняет.
struct Anchor {
    Vector3 normal;
    Real gap;      // зазор вдоль normal: нижняя граница расстояния
    Real distance; // само расстояние со знаком
};

Anchor anchorOf(const Body& a, const Body& b) {
    const Separation separation = separationOf(a, b);
    Anchor best{separation.normal, gapAlong(a, b, separation.normal), separation.distance};
    const auto consider = [&](Vector3 direction) {
        const Real size = direction.length();
        if (size < Real(1e-6)) return;
        const Vector3 unit = (1 / size) * direction;
        const Real gap = gapAlong(a, b, unit);
        if (gap > best.gap) best.normal = unit, best.gap = gap;
    };
    const Vector3 found = separation.normal;
    for (const Body* body : {&a, &b}) {
        const Real outward = body == &a ? Real(1) : Real(-1); // found направлена от a к b
        if (dynamic_cast<const Cylinder*>(body->shape.get())) {
            const Vector3 axis = body->pose().direction({0, 1, 0});
            const Real along = found.dot(axis);
            consider(found - along * axis);                         // по радиусу
            consider((along > 0 ? Real(1) : Real(-1)) * axis);      // по оси
        } else if (const ConvexHull* hull = body->shape->hull()) {
            const Vector3 local = body->pose().localDirection(outward * found);
            const ConvexHull::Face *first = nullptr, *second = nullptr;
            for (const ConvexHull::Face& face : hull->faces()) {
                if (!first || face.normal.dot(local) > first->normal.dot(local)) second = first, first = &face;
                else if (!second || face.normal.dot(local) > second->normal.dot(local)) second = &face;
            }
            for (const ConvexHull::Face* face : {first, second})
                if (face) consider(outward * body->pose().direction(face->normal));
        }
    }
    return best;
}

// Когда касающаяся пара начнёт входить друг в друга. Тела, коснувшись углом или ребром,
// проворачиваются вокруг точки касания и сходятся там, где точек контакта ещё нет. Расстояние
// между ними всё это время близко к нулю, и безопасных шагов «расстояние, делённое на скорость»
// оно не даёт. Но зазор вдоль направления n(t) — наименьшее из гладких функций времени (по одной
// на пару точек), вторая производная которых не больше M. На участке длины Δ такая функция не
// опускается ниже меньшего из своих значений на концах больше чем на M Δ² / 8
// (tests/proofs/sag_bound.smt2), а расстояние со знаком не меньше зазора вдоль любого направления.
// Поэтому участок заверен, если зазор на обоих его концах не ниже порога, а за вычетом прогиба —
// не ниже порога больше чем на допуск kOverlap. Чем дальше тела от порога, тем длиннее участок.
//
// Участок заверяет любая из четырёх проверок:
//   — зазор вдоль направления, неподвижного в осях тела a или тела b: годится, когда тела
//     касаются гранью одного из них;
//   — зазор вдоль направления, поворачивающегося от нормали в начале участка к нормали в его
//     конце: годится и когда нормаль не закреплена ни в одном теле (ребро о ребро, качение); его
//     кривизна включает поворот нормали на угол φ за участок:
//     M = 3,2 (φ/Δ)² D + 2,08 (φ/Δ) V + A;
//   — само расстояние: оно меняется не быстрее наибольшей скорости сближения точек V
//     (tests/proofs/advance_is_safe.smt2), поэтому между концами участка оно не ниже
//     (d₀ + d₁)/2 − V Δ/2. Эта проверка грубая — участки по микросекундам, — но не зависит от
//     того, насколько точна нормаль, и не даёт поиску встать, когда остальные не срабатывают.
// Отрезок проходится такими участками; незаверенный участок делится пополам.
//
// Порог level и допуск allowance задаёт вызывающий. Для касающихся тел порог — минус допуск
// перекрытия: расстояние не должно опуститься ниже него. Если тела уже перекрыты глубже, порог —
// нынешняя глубина плюс допуск: идти в углубление перекрытия нельзя; как только тела разошлись,
// порог возвращается на место. Для тел, которые ещё не коснулись, порог — середина полосы
// касания: поиск останавливается внутри неё. Возвращает последний заверенный момент или horizon.
template <class BodiesAt>
Real crossingStart(const Body& a, const Body& b, BodiesAt&& bodiesAt, const MotionBounds& boundsA, const MotionBounds& boundsB, Real horizon, Real level,
                   Real allowance) {
    const Vector3 opening = boundsB.acceleration - boundsA.acceleration;
    const Real apart = (b.position - a.position).length() + (boundsA.speed + boundsB.speed) * horizon;
    // Наибольшие расстояние, скорость и ускорение точки одного тела относительно точки другого.
    const Real extent = apart + boundsA.radius + boundsB.radius;
    const Real speed = boundsA.speed + boundsA.spin * boundsA.radius + boundsB.speed + boundsB.spin * boundsB.radius;
    const Real acceleration = boundsA.acceleration.length() + (boundsA.spinUp + boundsA.spin * boundsA.spin) * boundsA.radius + boundsB.acceleration.length() +
                              (boundsB.spinUp + boundsB.spin * boundsB.spin) * boundsB.radius;
    // Направление зазора хранится в осях каждого тела: вместе с телом оно и поворачивается.
    const auto curvatures = [&](const Body& at, const Body& bt, Vector3 inA, Vector3 inB) {
        return std::make_pair(gapCurvature(boundsA, boundsB, at.pose().direction(inA), opening, apart + boundsB.radius),
                              gapCurvature(boundsB, boundsA, bt.pose().direction(inB), opening, apart + boundsA.radius));
    };
    // Состояние в начале очередного участка: тела, направления и зазоры вдоль них. exact — в этот
    // момент известны истинная нормаль и расстояние (now).
    Body at = a, bt = b;
    Anchor now = anchorOf(a, b);
    Vector3 inA = a.pose().localDirection(now.normal), inB = b.pose().localDirection(now.normal);
    Real gapA = now.gap, gapB = now.gap;
    bool exact = true;
    Real floor = now.distance < level ? now.distance - allowance : level;
    const auto certified = [&](Real curvature, Real start, Real end, Real span) {
        const Real lower = std::min(start, end);
        return lower >= floor && lower - curvature * span * span / 8 >= floor - allowance;
    };
    // Самый длинный участок, который зазор gap при кривизне curvature ещё может заверить.
    const auto longest = [&](Real curvature, Real gap) {
        if (gap < floor) return Real(0);
        return curvature > 0 ? Real(0.999) * sqrt(8 * (gap - floor + allowance) / curvature) : std::numeric_limits<Real>::infinity();
    };
    const auto moveTo = [&](const Body& bodyA, const Body& bodyB, const Anchor& anchor) {
        at = bodyA, bt = bodyB, now = anchor, exact = true;
        inA = bodyA.pose().localDirection(anchor.normal), inB = bodyB.pose().localDirection(anchor.normal);
        gapA = gapB = anchor.gap;
        floor = std::max(floor, anchor.distance < level ? anchor.distance - allowance : level);
    };
    for (Real t = 0; t < horizon;) {
        const auto [curveA, curveB] = curvatures(at, bt, inA, inB);
        Real span = std::min(horizon - t, std::max({longest(curveA, gapA), longest(curveB, gapB), exact ? longest(acceleration, gapA) : Real(0)}));
        if (span == 0) span = horizon - t; // вдоль прежних направлений заверять нечем: решат истинные расстояния
        for (;; span /= 2) {
            if (t + span == t) return t; // перекрытие начинается сразу: ближе на часах не подойти
            const auto [ae, be] = bodiesAt(t + span);
            const Real endA = gapAlong(ae, be, ae.pose().direction(inA)), endB = gapAlong(ae, be, be.pose().direction(inB));
            if (certified(curveA, gapA, endA, span) || certified(curveB, gapB, endB, span)) {
                gapA = endA, gapB = endB, exact = false;
                at = ae, bt = be, t += span;
                break;
            }
            // Вмороженные направления участок не заверили. Истинные расстояния и нормали на концах.
            if (!exact) moveTo(at, bt, anchorOf(at, bt));
            const Anchor end = anchorOf(ae, be);
            if (end.distance < floor) {
                // Порог впереди пересечён, а тела уже в четверти допуска от него: событие найдено.
                // Подбираться к порогу ближе незачем — это тысячи делений ради долей микрона.
                if (now.distance <= floor + allowance / 4) return t;
                continue;
            }
            // Направление, поворачивающееся от нормали начала к нормали конца. Угол между ними
            // φ не больше 1,05 sin φ, пока он меньше 0,5 рад.
            const Real turn = Real(1.05) * now.normal.cross(end.normal).length();
            const Real curvature = (Real(3.2) * turn * turn * extent + Real(2.08) * turn * speed * span) / (span * span) + acceleration;
            const bool turning = now.normal.dot(end.normal) > Real(0.88) && certified(curvature, now.gap, end.gap, span);
            // Само расстояние: между концами участка оно не ниже (d₀ + d₁)/2 − V Δ/2.
            const bool slow = (now.distance + end.distance) / 2 - speed * span / 2 >= floor - allowance;
            if (turning || slow) {
                t += span;
                moveTo(ae, be, end);
                break;
            }
        }
    }
    return horizon;
}

} // namespace

// Наибольший поворот касающихся тел за время t под действием найденных сил. Момент импульса на
// отрезке линеен по времени, поэтому угловая скорость не больше, чем в его начале или в конце.
Real EventSearch::contactTurn(const std::vector<ContactPair>& pairs, const StepArray<Rate>& rates, Real t) const {
    Real turn = 0;
    const auto consider = [&](int id) {
        Body later = bodies_[size_t(id)];
        integrateVelocity(later, rates[size_t(id)].force, rates[size_t(id)].torque, t);
        turn = std::max(turn, t * std::max(bodies_[size_t(id)].angularVelocity().length(), later.angularVelocity().length()));
    };
    for (const ContactPair& pair : pairs)
        if (pair.inContact()) consider(pair.a), consider(pair.b);
    // Силы сочленения на отрезке тоже постоянны, и для провернувшегося тела тоже теряют смысл.
    for (const Joint& joint : joints_) consider(joint.a), consider(joint.b);
    return turn;
}

// Ближайший момент, когда появится новая точка касания: пара ещё не касающихся тел войдёт в
// полосу касания или у касающейся пары коснётся ещё одна точка.
Real EventSearch::earliestImpact(const std::vector<ContactPair>& pairs, const StepArray<Rate>& rates, Real h) const {
    Real earliest = h;
    for (const ContactPair& pair : pairs) earliest = impactOf(pair, rates[size_t(pair.a)], rates[size_t(pair.b)], h, earliest);
    return earliest;
}

// Время удара одной пары, если оно раньше earliest; иначе earliest.
Real EventSearch::impactOf(const ContactPair& pair, const Rate& ra, const Rate& rb, Real h, Real earliest) const {
    const Body a = bodies_[size_t(pair.a)].part(pair.partA);
    const Body b = bodies_[size_t(pair.b)].part(pair.partB);
    // Импульс на отрезке линеен по времени, поэтому скорость тела не больше, чем в начале
    // отрезка или в его конце.
    const auto fastest = [&](const Body& body, const Rate& rate) {
        Body later = body;
        integrateVelocity(later, rate.force, rate.torque, h);
        return std::max(maxPointSpeed(body), maxPointSpeed(later));
    };
    const Real closing = fastest(a, ra) + fastest(b, rb);
    const auto bodiesAt = [&](Real t) { return std::make_pair(advanced(a, ra.force, ra.torque, t), advanced(b, rb.force, rb.torque, t)); };
    if (pair.inContact()) {
        earliest = nextPointContact(pair, a, b, bodiesAt, closing, earliest);
        return crossingStart(a, b, bodiesAt, motionBounds(a, ra.force, ra.torque, h), motionBounds(b, rb.force, rb.torque, h), earliest, -kOverlap, kOverlap);
    }
    const auto distanceAt = [&](Real t) {
        const auto [at, bt] = bodiesAt(t);
        const ClosestPoints gap = closestPoints(*at.shape, at.pose(), *bt.shape, bt.pose());
        return gap.coresOverlap ? Real(0) : gap.distance;
    };
    // Сначала — продвижение «расстояние, делённое на скорость»: обычно ему хватает десятка
    // шагов. Если тела летят вплотную и быстро вращаются, шаги мельчают до сотых долей
    // радиана поворота; тогда остаток интервала проходит поиск с доказанной границей, которому
    // хватает участка на десятые доли радиана. Он останавливается в середине полосы касания.
    Real reached = -1;
    const Real impact = timeOfImpact(distanceAt, closing, earliest, kContactDistance, kAdvancePatience, &reached);
    if (reached < 0) return std::min(earliest, impact);
    const auto [as, bs] = bodiesAt(reached);
    const auto later = [&](Real t) { return bodiesAt(reached + t); };
    return reached + crossingStart(as, bs, later, motionBounds(a, ra.force, ra.torque, h), motionBounds(b, rb.force, rb.torque, h), earliest - reached, kContactDistance / 2,
                                   kContactDistance / 4);
}

// Когда сочленение дойдёт до предела хода. Это событие того же рода, что удар: отрезок обрывается
// на нём, и следующий начинается с неупругого удара о предел. Зазор до предела g(t) — гладкая
// функция времени: его скорость не больше W, вторая производная не больше M (обе границы — из
// MotionBounds). Участок от t до t + Δ пройден без события, если
//   Δ <= (g(t) − уровень) / W                          — зазор не успеет закрыться, или
//   min(g(t), g(t + Δ)) − M Δ² / 8 >= уровень          — прогиб между концами не достанет уровня
// (tests/proofs/advance_is_safe.smt2, tests/proofs/sag_bound.smt2). Поиск останавливается в середине полосы
// предела; сочленение, которое уже в полосе, держит связь предела, и искать нечего.
//
// Шарнир: g = ±(угол − предел), dg/dt = ±e·(ω_b − ω_a) (tests/proofs/hinge_angle_rate.smt2), а
// d²g/dt² = ±((ω_a × e)·ω_b + e·(α_b − α_a)). Ползун: g = ±(e·d − предел), d — от точки
// сочленения тела a до точки тела b; ось e вращается с телом a.
namespace {

// Границы изменения зазора предела хода на отрезке h: |dg/dt| ≤ speed, |d²g/dt²| ≤ curvature.
struct GapBounds {
    Real speed, curvature;
};
GapBounds limitBounds(const SingleFreedomJoint& joint, const Body& a, const Body& b, const MotionBounds& ma, const MotionBounds& mb, Real h) {
    if (joint.angular()) return {ma.spin + mb.spin, ma.spinUp + mb.spinUp + ma.spin * mb.spin};
    const Real armA = joint.anchorA.length(), armB = joint.anchorB.length();
    const Real parting = ma.speed + mb.speed + ma.spin * armA + mb.spin * armB; // |dd/dt|
    const Real apart = (b.pose().point(joint.anchorB) - a.pose().point(joint.anchorA)).length() + parting * h; // |d|
    return {parting + ma.spin * apart,
            (mb.acceleration - ma.acceleration).length() + (ma.spinUp + ma.spin * ma.spin) * (armA + apart) + (mb.spinUp + mb.spin * mb.spin) * armB + 2 * ma.spin * parting};
}

} // namespace

Real EventSearch::earliestLimit(const StepArray<Rate>& rates, Real h) const {
    constexpr Real level = kLimitBand / 2, allowance = kLimitBand / 4;
    Real earliest = h;
    for (const Joint& any : joints_) {
        const SingleFreedomJoint* single = any.singleFreedom();
        if (!single || !single->limited()) continue;
        const SingleFreedomJoint& joint = *single;
        const Body &a = bodies_[size_t(joint.a)], &b = bodies_[size_t(joint.b)];
        if (joint.limitGap(a, b).gap <= kLimitBand) continue;
        const Rate &ra = rates[size_t(joint.a)], &rb = rates[size_t(joint.b)];
        const MotionBounds ma = motionBounds(a, ra.force, ra.torque, h), mb = motionBounds(b, rb.force, rb.torque, h);
        const GapBounds bound = limitBounds(joint, a, b, ma, mb, h);
        const Real speed = bound.speed, curvature = bound.curvature;
        const auto gapAt = [&](Real t) { return joint.limitGap(advanced(a, ra.force, ra.torque, t), advanced(b, rb.force, rb.torque, t)).gap; };
        Real time = 0;
        while (time < earliest) {
            const Real gap = gapAt(time);
            if (gap <= level + allowance) break;
            const Real room = gap - level;
            const Real sure = speed > 0 ? (room - allowance / 2) / speed : earliest - time; // столько зазор заведомо не закроется
            Real span = earliest - time;
            if (curvature > 0) span = std::min(span, sqrt(4 * room / curvature));
            while (span > sure && std::min(gap, gapAt(time + span)) - curvature * span * span / 8 < level) span /= 2;
            span = std::max(span, std::min(sure, earliest - time));
            if (time + span == time) break;
            time += span;
        }
        earliest = std::min(earliest, time);
    }
    return earliest;
}

} // namespace kp
