// Опыт Л2 (docs/ROADMAP.md, research/papers.md): решатель Augmented Vertex Block Descent на общих
// сценах (common.h). Giles, Diaz, Yuksel, «Augmented Vertex Block Descent», ACM TOG 44(4), 2025.
// Порядок действий и постоянные — по открытой демонстрации авторов (avbd-demo3d, лицензия MIT);
// код написан заново. В сборку движка и в табло не входит: это прибор для решения, годится ли метод
// на место пути на пружинах.
//
// Шаг — неявный Эйлер как минимизация по положениям: x = argmin ‖x − y‖²_M / (2 dt²) + Σ E_j(x),
// y — куда тело пришло бы без контактов. За проход каждое тело по очереди сдвигается одним шагом
// Ньютона: H d = −g, H = M / dt² + Σ k J Jᵀ, g = M (x − y) / dt² + Σ J F по строкам его контактов.
// Жёсткое условие C ≥ 0 — дополненным лагранжианом: F = min(k C + λ, 0); после прохода λ ← F,
// k ← k + β |C|; между кадрами λ ← α γ λ, k ← γ k. Трение — две строки на точку, ‖F_t‖ ≤ μ |F_n|.
// Ошибка положения убирается долей (1 − α) за шаг: C = (1 − α) C₀ + J (x − x₀).
//
// Отличия от демонстрации: площадки ищет наш PairFinder — в них есть точки, которые ещё не
// коснулись, и их зазор C₀ > 0 входит в условие целиком; запаса на перекрытие нет; тензор инерции
// берётся в осях мира.
//
//   compare-avbd [ПРОХОДОВ]        по умолчанию 4; SCENE и BENCH — как у compare-ours
//   AVBD_ALPHA, AVBD_BETA, AVBD_GAMMA — постоянные метода (0,99; 10000 Н/м² на проход; 0,999)
//   AVBD_MASS_SCALE=s — начальная жёсткость точки s · m / dt² (m — приведённая масса пары) вместо 1 Н/м
//   AVBD_PER_KG=1 — наименьшая жёсткость и β берутся на килограмм приведённой массы пары: постоянные
//                   демонстрации подобраны под тела в килограмм, наши кубы весят от 64 кг до тонны
//   AVBD_PHASES=1 — в конце напечатать время поиска контактов и решения на кадр
#include "common.h"
#include <physics.h>
#include <collision/PairFinder.h>
#include <math/StepMemory.h>
#include <math/WorkerPool.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <map>

using namespace kp;

namespace {

struct Settings {
    int iterations = 4;
    Real alpha = Real(0.99), beta = 10000, gamma = Real(0.999);
    Real penaltyMin = 1, penaltyMax = Real(1e10); // Н/м
    Real stick = Real(1e-5);                      // м: касательный сдвиг меньше этого — точка держится на месте
    Real massScale = 0;
    bool perKilogram = false;
};

struct Six {
    Vector3 lin, ang;
};

Real at(const Vector3& v, int k) { return k == 0 ? v.x : k == 1 ? v.y : v.z; }
Real& at(Vector3& v, int k) { return k == 0 ? v.x : k == 1 ? v.y : v.z; }

// Поворот из from в now вектором поворота; для малых углов — удвоенная векторная часть кватерниона.
Vector3 turned(const Quaternion& now, const Quaternion& from) {
    const Quaternion d = now * from.conjugate();
    const Real twice = d.w < 0 ? Real(-2) : Real(2);
    return {twice * d.x, twice * d.y, twice * d.z};
}

// H += k J Jᵀ, нижний треугольник.
void addOuter(Real H[6][6], const Six& J, Real k) {
    const Real v[6] = {J.lin.x, J.lin.y, J.lin.z, J.ang.x, J.ang.y, J.ang.z};
    for (int i = 0; i < 6; ++i)
        for (int j = 0; j <= i; ++j) H[i][j] += k * v[i] * v[j];
}

// H d = −g; H симметричная положительно определённая (нижний треугольник): разложение Холецкого на месте.
Six solve(Real H[6][6], const Six& g) {
    Real b[6] = {-g.lin.x, -g.lin.y, -g.lin.z, -g.ang.x, -g.ang.y, -g.ang.z};
    for (int i = 0; i < 6; ++i)
        for (int j = 0; j <= i; ++j) {
            Real value = H[i][j];
            for (int m = 0; m < j; ++m) value -= H[i][m] * H[j][m];
            H[i][j] = i == j ? std::sqrt(value) : value / H[j][j];
        }
    for (int i = 0; i < 6; ++i) {
        for (int m = 0; m < i; ++m) b[i] -= H[i][m] * b[m];
        b[i] /= H[i][i];
    }
    for (int i = 5; i >= 0; --i) {
        for (int m = i + 1; m < 6; ++m) b[i] -= H[m][i] * b[m];
        b[i] /= H[i][i];
    }
    return {{b[0], b[1], b[2]}, {b[3], b[4], b[5]}};
}

class Avbd {
public:
    Avbd(const Settings& settings, const SceneSpec& spec);
    ~Avbd();
    void step(Real dt);
    std::vector<State> states() const;

private:
    struct Pose {
        Vector3 position;
        Quaternion orientation;
    };
    // Точка контакта: строка нормали и две строки трения.
    struct Contact {
        int a, b;
        ContactId key;
        Vector3 basis[3];        // нормаль от A к B и две касательные, мир
        Vector3 rA, rB;          // точка на каждом теле, в его осях
        Vector3 c0;              // зазор и касательный сдвиг в позах начала шага
        Vector3 lambda, penalty; // λ и k строк
        Real friction, beta;
        bool stick = false;
    };
    struct Kept {
        Vector3 lambda, penalty, rA, rB;
        bool stick;
    };
    // Значения условий и силы строк при нынешних позах; sliding — трение на пределе.
    struct Rows {
        Vector3 c, f, armA, armB;
        bool sliding;
    };

    Vector3 inertiaTimes(size_t i, Vector3 v) const;
    Contact contactOf(const ContactPair& pair, int point, Real dt) const;
    void collide(Real dt);
    void predict(Real dt);
    Rows rowsOf(const Contact& contact) const;
    void moveBody(size_t i, Real dt);
    void updateDuals();
    void finish(Real dt);

    Settings s_;
    Vector3 gravity_{0, Real(-kGravity), 0};
    std::vector<Body> bodies_;
    std::vector<Vector3> x_, v_, w_, vPrev_;
    std::vector<Quaternion> q_;
    std::vector<Pose> start_, target_; // позы начала шага и позы без контактов
    std::vector<Contact> contacts_;
    std::vector<std::vector<int>> touching_; // контакты каждого тела
    std::map<ContactId, Kept> memory_;
    double collideMs_ = 0, solveMs_ = 0;
    int frames_ = 0;
};

Avbd::Avbd(const Settings& settings, const SceneSpec& spec) : s_(settings) {
    for (const BoxSpec& b : spec.boxes) {
        Body body = Body::box({b.half.x, b.half.y, b.half.z}, b.density, {b.at.x, b.at.y, b.at.z});
        body.orientation = {b.turn.w, b.turn.x, b.turn.y, b.turn.z};
        bodies_.push_back(body);
        x_.push_back(body.position), q_.push_back(body.orientation);
        v_.push_back({b.velocity.x, b.velocity.y, b.velocity.z}), w_.push_back({});
        if (!body.isStatic()) bodies_.back().linearMomentum = (1 / body.mass.inverseMass) * v_.back();
    }
    vPrev_ = v_, start_.resize(bodies_.size()), target_.resize(bodies_.size()), touching_.resize(bodies_.size());
}

Avbd::~Avbd() {
    if (std::getenv("AVBD_PHASES") && frames_ > 0)
        std::fprintf(stderr, "avbd: поиск контактов %.2f мс на кадр, решение %.2f мс, контактных точек %zu\n", collideMs_ / frames_, solveMs_ / frames_, contacts_.size());
}

// I v в осях мира: R (I ⊙ (Rᵀ v)).
Vector3 Avbd::inertiaTimes(size_t i, Vector3 v) const {
    const Body& body = bodies_[i];
    const Vector3 inverse = body.mass.inverseInertia, local = body.orientation.rotateInverse(v);
    return body.orientation.rotate({local.x / inverse.x, local.y / inverse.y, local.z / inverse.z});
}

Avbd::Contact Avbd::contactOf(const ContactPair& pair, int point, Real dt) const {
    const ContactPoint& found = pair.manifold.points[size_t(point)];
    const Body &A = bodies_[size_t(pair.a)], &B = bodies_[size_t(pair.b)];
    Contact contact{};
    contact.a = pair.a, contact.b = pair.b, contact.key = {pair.a, pair.b, found.id, pair.partA, pair.partB};
    const Vector3 n = pair.manifold.normal, aside = std::fabs(n.x) < Real(0.57) ? Vector3{1, 0, 0} : Vector3{0, 1, 0};
    const Vector3 t1 = (1 / n.cross(aside).length()) * n.cross(aside);
    contact.basis[0] = n, contact.basis[1] = t1, contact.basis[2] = n.cross(t1);
    contact.rA = A.orientation.rotateInverse(found.onA - A.position), contact.rB = B.orientation.rotateInverse(found.onB - B.position);
    contact.friction = std::sqrt(A.friction * B.friction);
    const Real reduced = 1 / (A.mass.inverseMass + B.mass.inverseMass);
    const Real perMass = s_.perKilogram ? reduced : Real(1);
    const Real least = std::max(perMass * s_.penaltyMin, s_.massScale * reduced / (dt * dt));
    contact.beta = perMass * s_.beta;
    contact.penalty = {least, least, least};
    const auto known = memory_.find(contact.key);
    if (known != memory_.end()) {
        contact.lambda = (s_.alpha * s_.gamma) * known->second.lambda;
        for (int r = 0; r < 3; ++r) at(contact.penalty, r) = std::clamp(s_.gamma * at(known->second.penalty, r), least, s_.penaltyMax);
        contact.stick = known->second.stick;
        if (contact.stick) contact.rA = known->second.rA, contact.rB = known->second.rB; // точка держится: якоря прежние
    }
    const Vector3 apart = (B.position + B.orientation.rotate(contact.rB)) - (A.position + A.orientation.rotate(contact.rA));
    contact.c0 = {contact.basis[0].dot(apart), contact.basis[1].dot(apart), contact.basis[2].dot(apart)};
    return contact;
}

void Avbd::collide(Real dt) {
    PairFinder finder(bodies_, gravity_);
    finder.boundary = Real(1e-3);
    contacts_.clear();
    for (std::vector<int>& list : touching_) list.clear();
    for (const ContactPair& pair : finder.pairs(dt))
        for (int point = 0; point < pair.manifold.count; ++point) {
            touching_[size_t(pair.a)].push_back(int(contacts_.size())), touching_[size_t(pair.b)].push_back(int(contacts_.size()));
            contacts_.push_back(contactOf(pair, point, dt));
        }
}

// Позы без контактов и начальное приближение. Тело, которое в прошлом шаге не ускорялось вдоль
// тяжести (оно на опоре), начинает проходы с места, а не из падения на g dt².
void Avbd::predict(Real dt) {
    for (size_t i = 0; i < bodies_.size(); ++i) {
        start_[i] = target_[i] = {x_[i], q_[i]};
        if (bodies_[i].isStatic()) continue;
        const Real along = std::clamp((v_[i] - vPrev_[i]).dot(gravity_) / (dt * gravity_.dot(gravity_)), Real(0), Real(1));
        target_[i] = {x_[i] + dt * v_[i] + (dt * dt) * gravity_, (Quaternion::fromRotationVector(dt * w_[i]) * q_[i]).unit()};
        x_[i] = x_[i] + dt * v_[i] + (along * dt * dt) * gravity_, q_[i] = target_[i].orientation;
    }
}

Avbd::Rows Avbd::rowsOf(const Contact& contact) const {
    const size_t a = size_t(contact.a), b = size_t(contact.b);
    Rows rows{};
    rows.armA = q_[a].rotate(contact.rA), rows.armB = q_[b].rotate(contact.rB);
    const Vector3 moved = (x_[b] - start_[b].position) - (x_[a] - start_[a].position);
    const Vector3 turnedA = turned(q_[a], start_[a].orientation), turnedB = turned(q_[b], start_[b].orientation);
    for (int r = 0; r < 3; ++r) {
        const Vector3 e = contact.basis[r];
        // Зазор точки, которая ещё не коснулась, входит целиком: сближаться до касания можно.
        const Real kept = r == 0 && contact.c0.x > 0 ? contact.c0.x : (1 - s_.alpha) * at(contact.c0, r);
        at(rows.c, r) = kept + e.dot(moved) + rows.armB.cross(e).dot(turnedB) - rows.armA.cross(e).dot(turnedA);
        at(rows.f, r) = at(contact.penalty, r) * at(rows.c, r) + at(contact.lambda, r);
    }
    rows.f.x = std::min(rows.f.x, Real(0));
    const Real limit = contact.friction * std::fabs(rows.f.x), size = std::sqrt(rows.f.y * rows.f.y + rows.f.z * rows.f.z);
    rows.sliding = size > limit;
    if (rows.sliding) rows.f.y *= limit / size, rows.f.z *= limit / size;
    return rows;
}

void Avbd::moveBody(size_t i, Real dt) {
    const Real mass = 1 / bodies_[i].mass.inverseMass, rate = 1 / (dt * dt);
    Real H[6][6] = {};
    for (int k = 0; k < 3; ++k) {
        Vector3 axis{};
        at(axis, k) = 1;
        const Vector3 column = rate * inertiaTimes(i, axis);
        H[k][k] = mass * rate, H[3][3 + k] = column.x, H[4][3 + k] = column.y, H[5][3 + k] = column.z;
    }
    Six g{(mass * rate) * (x_[i] - target_[i].position), rate * inertiaTimes(i, turned(q_[i], target_[i].orientation))};
    for (int index : touching_[i]) {
        const Contact& contact = contacts_[size_t(index)];
        const Rows rows = rowsOf(contact);
        if (rows.f.x == 0) continue; // точка не давит: ни силы, ни жёсткости
        const bool first = size_t(contact.a) == i;
        for (int r = 0; r < 3; ++r) {
            const Vector3 e = (first ? Real(-1) : Real(1)) * contact.basis[r];
            const Six J{e, (first ? rows.armA : rows.armB).cross(e)};
            g.lin += at(rows.f, r) * J.lin, g.ang += at(rows.f, r) * J.ang;
            addOuter(H, J, at(contact.penalty, r));
        }
    }
    const Six d = solve(H, g);
    x_[i] += d.lin, q_[i] = (Quaternion::fromRotationVector(d.ang) * q_[i]).unit();
}

void Avbd::updateDuals() {
    for (Contact& contact : contacts_) {
        const Rows rows = rowsOf(contact);
        contact.lambda = rows.f;
        contact.stick = contact.stick && rows.f.x < 0; // точка, которая не давит, на месте не держится: якоря у неё каждый раз новые
        if (rows.f.x == 0 || rows.sliding) continue;
        contact.penalty.x = std::min(contact.penalty.x + contact.beta * std::fabs(rows.c.x), s_.penaltyMax);
        for (int r = 1; r < 3; ++r) at(contact.penalty, r) = std::min(at(contact.penalty, r) + contact.beta * std::fabs(at(rows.c, r)), s_.penaltyMax);
        contact.stick = std::sqrt(rows.c.y * rows.c.y + rows.c.z * rows.c.z) < s_.stick;
    }
}

// Скорости — по сдвигу за шаг; тела и память контактов — к следующему шагу.
void Avbd::finish(Real dt) {
    for (size_t i = 0; i < bodies_.size(); ++i) {
        Body& body = bodies_[i];
        if (body.isStatic()) continue;
        vPrev_[i] = v_[i];
        v_[i] = (1 / dt) * (x_[i] - start_[i].position), w_[i] = (1 / dt) * turned(q_[i], start_[i].orientation);
        body.position = x_[i], body.orientation = q_[i];
        body.linearMomentum = (1 / body.mass.inverseMass) * v_[i], body.angularMomentum = inertiaTimes(i, w_[i]);
    }
    memory_.clear();
    for (const Contact& contact : contacts_) memory_[contact.key] = {contact.lambda, contact.penalty, contact.rA, contact.rB, contact.stick};
}

void Avbd::step(Real dt) {
    const StepMemory memory;
    const auto t0 = std::chrono::steady_clock::now();
    collide(dt);
    const auto t1 = std::chrono::steady_clock::now();
    predict(dt);
    for (int pass = 0; pass < s_.iterations; ++pass) {
        for (size_t i = 0; i < bodies_.size(); ++i)
            if (!bodies_[i].isStatic()) moveBody(i, dt);
        updateDuals();
    }
    finish(dt);
    const auto t2 = std::chrono::steady_clock::now();
    collideMs_ += std::chrono::duration<double, std::milli>(t1 - t0).count(), solveMs_ += std::chrono::duration<double, std::milli>(t2 - t1).count();
    ++frames_;
}

std::vector<State> Avbd::states() const {
    std::vector<State> all;
    for (size_t i = 0; i < bodies_.size(); ++i)
        all.push_back({{x_[i].x, x_[i].y, x_[i].z}, {q_[i].w, q_[i].x, q_[i].y, q_[i].z}, {v_[i].x, v_[i].y, v_[i].z}, {w_[i].x, w_[i].y, w_[i].z}, false});
    return all;
}

Real setting(const char* name, Real otherwise) {
    const char* text = std::getenv(name);
    return text ? Real(std::atof(text)) : otherwise;
}

} // namespace

int main(int argc, char** argv) {
    Settings settings;
    settings.iterations = argc > 1 ? std::atoi(argv[1]) : 4;
    settings.alpha = setting("AVBD_ALPHA", settings.alpha), settings.beta = setting("AVBD_BETA", settings.beta);
    settings.gamma = setting("AVBD_GAMMA", settings.gamma), settings.massScale = setting("AVBD_MASS_SCALE", 0);
    settings.perKilogram = std::getenv("AVBD_PER_KG") != nullptr;
    if (const char* threads = std::getenv("KP_THREADS")) WorkerPool::shared().resize(std::atoi(threads));
    const std::string engine = "avbd " + std::to_string(settings.iterations) + (settings.massScale > 0 ? " k=m/dt²" : settings.perKilogram ? " на кг" : "");
    compare(engine, [&](const SceneSpec& spec, const Play& play) {
        if (!spec.joints.empty()) std::fprintf(stderr, "avbd: сочленения сцены %s не поддержаны — числа этой строки не годятся\n", spec.key);
        Avbd solver(settings, spec);
        play([&](double dt) { solver.step(Real(dt)); }, [&] { return solver.states(); });
    });
}
