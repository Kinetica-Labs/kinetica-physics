#pragma once
// При касании совпадающих рёбер возможны разные нормали. Запоминаем показанную площадку
// каждой пары тел. Существование пары проверяем в текущих позах, в том числе при отрыве.
#include <collision/PairFinder.h>
#include <debug/DebugGeometry.h>
#include <algorithm>
#include <map>

class ContactOverlay {
public:
    void clear() { world_ = nullptr; }

    const kp::DebugGeometry& sample(const kp::World& world, bool raw = false) {
        if (world_ != &world || observed_.size() != world.bodies().size()) {
            pairs_.clear();
            observed_.clear();
            markers_ = {};
            world_ = &world;
        }
        if (raw) {
            raw_.points.clear();
            raw_.lines.clear();
            raw_.addContactMarkers(world);
            return raw_;
        }
        const auto& bodies = world.bodies();
        if (observed_.size() == bodies.size() &&
            std::equal(observed_.begin(), observed_.end(), bodies.begin(),
                       [](const Pose& saved, const kp::Body& body) { return saved.identical(Pose(body)); }))
            return markers_;
        observed_.clear();
        for (const auto& body : bodies) observed_.emplace_back(body);
        update(world);
        return markers_;
    }

private:
    struct Pose {
        kp::Transform transform;
        const kp::ConvexShape* shape = nullptr;
        const std::vector<kp::Body::Part>* parts = nullptr;
        kp::Real radius = 0;
        bool fixed = false;

        Pose() = default;
        explicit Pose(const kp::Body& b) : transform(b.pose()), shape(b.shape.get()), parts(b.parts.get()),
                                           radius(shape ? shape->boundingRadius() : 0), fixed(b.isStatic()) {}
        bool sameShape(const Pose& b) const { return shape == b.shape && parts == b.parts && radius == b.radius && fixed == b.fixed; }
        bool identical(const Pose& b) const {
            const auto p = transform.position - b.transform.position;
            const auto a = transform.orientation, q = b.transform.orientation;
            return sameShape(b) && p.x == 0 && p.y == 0 && p.z == 0 &&
                   a.w == q.w && a.x == q.x && a.y == q.y && a.z == q.z;
        }
        bool near(const Pose& b) const {
            // |Δp| + 2 R sin(θ/2) ограничивает смещение любой точки формы.
            // 0,1 мкм на тело, в 1000 раз меньше полосы касания. Отсчёт от последнего выбора стрелок.
            constexpr kp::Real tolerance = kp::kContactDistance / 1000;
            const auto turn = transform.orientation * b.transform.orientation.conjugate();
            const auto chord = 2 * radius * kp::Vector3{turn.x, turn.y, turn.z}.length();
            return sameShape(b) && (transform.position - b.transform.position).length() + chord <= tolerance;
        }
    };

    struct Patch {
        Pose a, b;
        kp::DebugGeometry markers;
        bool present = false, redraw = false;
    };
    void update(const kp::World& world) {
        for (auto& item : pairs_) item.second.present = false;
        for (const auto& contact : kp::PairFinder(world.bodies(), world.gravity).pairs(0)) {
            if (!contact.inContact() || contact.manifold.count == 0) continue;
            auto& patch = pairs_[{contact.a, contact.b}];
            if (!patch.present) {
                const auto& a = observed_[size_t(contact.a)];
                const auto& b = observed_[size_t(contact.b)];
                patch.redraw = patch.markers.points.empty() || !patch.a.near(a) || !patch.b.near(b);
                if (patch.redraw) {
                    patch.a = a;
                    patch.b = b;
                    patch.markers.points.clear();
                    patch.markers.lines.clear();
                }
                patch.present = true;
            }
            if (patch.redraw) patch.markers.add(contact.manifold);
        }
        markers_.points.clear();
        markers_.lines.clear();
        for (auto it = pairs_.begin(); it != pairs_.end();) {
            if (!it->second.present) {
                it = pairs_.erase(it);
                continue;
            }
            const auto& patch = it->second.markers;
            markers_.points.insert(markers_.points.end(), patch.points.begin(), patch.points.end());
            markers_.lines.insert(markers_.lines.end(), patch.lines.begin(), patch.lines.end());
            ++it;
        }
    }
    const kp::World* world_ = nullptr;
    std::vector<Pose> observed_;
    std::map<std::pair<int, int>, Patch> pairs_;
    kp::DebugGeometry markers_, raw_;
};
