#pragma once
// Два разных выпуклых тела в метрах. Контуры сверху подобны нижним и сдвинуты,
// поэтому каждая боковая грань — плоский четырёхугольник. Обход граней наружу.
#include <math/Vector3.h>
#include <vector>

namespace hull_sweep {
struct Mesh {
    std::vector<kp::Vector3> vertices;
    std::vector<std::vector<int>> faces;
};

inline Mesh tapered(const std::vector<kp::Vector3>& base, double bottom, double top,
                    double scale, double shiftX, double shiftY) {
    Mesh m;
    const int n = int(base.size());
    for (auto p : base) m.vertices.push_back({p.x,p.y,kp::Real(bottom)});
    for (auto p : base) m.vertices.push_back({kp::Real(scale*p.x+shiftX),kp::Real(scale*p.y+shiftY),kp::Real(top)});
    m.faces.resize(2);
    for (int i=0; i<n; ++i) {
        m.faces[0].push_back(n-1-i);
        m.faces[1].push_back(n+i);
        const int j = (i+1)%n;
        m.faces.push_back({i,j,n+j,n+i});
    }
    return m;
}
inline Mesh shapeA() {
    return tapered({{0.48,0.04,0},{0.16,0.36,0},{-0.30,0.295,0},{-0.46,-0.14,0},{0.04,-0.33,0}},
                   -0.24,0.31,0.74,0.046,-0.019);
}
inline Mesh shapeB() {
    return tapered({{0.39,-0.03,0},{0.29,0.26,0},{-0.05,0.40,0},{-0.42,0.22,0},{-0.36,-0.19,0},{0.08,-0.35,0}},
                   -0.28,0.22,0.83,-0.055,0.038);
}
}
