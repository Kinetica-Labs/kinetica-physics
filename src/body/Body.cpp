#include <kp/body/Body.h>

#include <kp/body/MassFrame.h>

#include <utility>

namespace kp {

Body Body::hull(const std::vector<Vector3>& points, Real density, Vector3 position) {
    const ConvexHull model = ConvexHull::fromPoints(points);
    const MassFrame mass = MassFrame::of(model, density);
    // Форма переводится в оси тела: начало в центре масс, оси — главные.
    std::vector<Vector3> vertices;
    for (Vector3 v : model.vertices()) vertices.push_back(mass.frame.localPoint(v));
    std::vector<std::vector<int>> loops;
    for (const ConvexHull::Face& face : model.faces()) loops.push_back(face.loop);
    Body body;
    body.shape = std::make_shared<ConvexHull>(std::move(vertices), loops);
    body.mass = mass.mass;
    body.position = position + mass.frame.position;
    body.orientation = mass.frame.orientation;
    return body;
}

} // namespace kp
