#include "GlRenderer.h"

#include <GLFW/glfw3.h>

#include <collision/shapes/Capsule.h>
#include <collision/shapes/ConvexHull.h>
#include <collision/shapes/Cylinder.h>

#include <cmath>
#include <cstdio>
#include <vector>

using namespace kp;

namespace {

// Цвет по смыслу: тело, неподвижное тело, точка контакта, нормаль контакта, ближайшие точки,
// узел дерева поиска пар.
void setColour(DebugKind kind) {
    static const float colours[][3] = {{0.60f, 0.80f, 1.00f}, {0.47f, 0.47f, 0.47f}, {1.00f, 0.27f, 0.27f},
                                       {1.00f, 0.80f, 0.20f}, {0.27f, 1.00f, 0.53f}, {0.55f, 0.35f, 0.75f}};
    glColor3fv(colours[int(kind)]);
}

// Клетка 2 × 2 светлых и тёмных квадратов; координаты текстуры — линейные по осям тела, с наклоном,
// чтобы клетка была видна на гранях любого направления.
void useChecker(double cell) {
    static GLuint texture = 0;
    if (!texture) {
        const unsigned char pixels[12] = {255, 255, 255, 170, 170, 170, 170, 170, 170, 255, 255, 255};
        glGenTextures(1, &texture);
        glBindTexture(GL_TEXTURE_2D, texture);
        glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, 2, 2, 0, GL_RGB, GL_UNSIGNED_BYTE, pixels);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST), glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    }
    glBindTexture(GL_TEXTURE_2D, texture);
    const double k = 0.5 / cell, s[4] = {k, 0, 0.37 * k, 0}, t[4] = {0, k, 0.61 * k, 0};
    glTexGeni(GL_S, GL_TEXTURE_GEN_MODE, GL_OBJECT_LINEAR), glTexGeni(GL_T, GL_TEXTURE_GEN_MODE, GL_OBJECT_LINEAR);
    glTexGendv(GL_S, GL_OBJECT_PLANE, s), glTexGendv(GL_T, GL_OBJECT_PLANE, t);
}

// Поверхность вращения вокруг оси Y тела: отрезки профиля (r, y) с нормалью (nr, ny) в его плоскости.
struct Meridian {
    double r0, y0, r1, y1, nr0, ny0, nr1, ny1;
};
void revolve(const std::vector<Meridian>& profile) {
    const int around = 24;
    for (const Meridian& m : profile) {
        glBegin(GL_QUAD_STRIP);
        for (int k = 0; k <= around; ++k) {
            const double angle = 2 * 3.14159265358979323846 * k / around, c = std::cos(angle), z = std::sin(angle);
            glNormal3d(m.nr0 * c, m.ny0, m.nr0 * z), glVertex3d(m.r0 * c, m.y0, m.r0 * z);
            glNormal3d(m.nr1 * c, m.ny1, m.nr1 * z), glVertex3d(m.r1 * c, m.y1, m.r1 * z);
        }
        glEnd();
    }
}
// Полусферы радиуса r с центрами на высотах ±h и боковая стенка между ними: h = 0 — шар.
std::vector<Meridian> capsuleProfile(double r, double h) {
    std::vector<Meridian> profile;
    const int steps = 8;
    for (int half = 0; half < 2; ++half)
        for (int k = 0; k < steps; ++k) {
            const double a0 = 1.5707963267948966 * k / steps, a1 = 1.5707963267948966 * (k + 1) / steps, up = half ? -1 : 1;
            profile.push_back({r * std::cos(a0), up * (h + r * std::sin(a0)), r * std::cos(a1), up * (h + r * std::sin(a1)), std::cos(a0), up * std::sin(a0), std::cos(a1), up * std::sin(a1)});
        }
    if (h > 0) profile.push_back({r, h, r, -h, 1, 0, 1, 0});
    return profile;
}

void drawShape(const ConvexShape& shape) {
    if (const auto* hull = dynamic_cast<const ConvexHull*>(&shape)) {
        for (const ConvexHull::Face& face : hull->faces()) {
            glBegin(GL_POLYGON);
            glNormal3d(face.normal.x, face.normal.y, face.normal.z);
            for (const int v : face.loop) glVertex3d(hull->vertices()[size_t(v)].x, hull->vertices()[size_t(v)].y, hull->vertices()[size_t(v)].z);
            glEnd();
        }
    } else if (const auto* capsule = dynamic_cast<const Capsule*>(&shape)) {
        revolve(capsuleProfile(shape.radius, capsule->halfLength));
    } else if (const auto* cylinder = dynamic_cast<const Cylinder*>(&shape)) {
        const double r = cylinder->coreRadius + shape.radius, h = cylinder->coreHalfHeight + shape.radius;
        revolve({{0, h, r, h, 0, 1, 0, 1}, {r, h, r, -h, 1, 0, 1, 0}, {r, -h, 0, -h, 0, -1, 0, -1}});
    } else {
        revolve(capsuleProfile(shape.radius, 0));
    }
}

// Тело закрашенным, в его осях: клетка cell метров.
void drawBody(const Body& body, double cell) {
    useChecker(cell);
    for (int k = 0; body.shape && k < body.partCount(); ++k) {
        const Body part = body.part(k);
        const Transform pose = part.pose();
        const Vector3 x = pose.direction({1, 0, 0}), y = pose.direction({0, 1, 0}), z = pose.direction({0, 0, 1}), p = pose.position;
        const double matrix[16] = {x.x, x.y, x.z, 0, y.x, y.y, y.z, 0, z.x, z.y, z.z, 0, p.x, p.y, p.z, 1};
        glPushMatrix();
        glMultMatrixd(matrix);
        drawShape(*part.shape);
        glPopMatrix();
    }
}

// Закрашенные тела: свет сверху-сбоку, цвет по номеру тела, уснувшие — темнее. Неподвижные — серые,
// полупрозрачные, с метровой клеткой и рисуются последними: сквозь стенку чаши видно, что в ней.
void drawFilled(const World& world) {
    static const float palette[][3] = {{0.90f, 0.55f, 0.35f}, {0.45f, 0.70f, 0.90f}, {0.60f, 0.80f, 0.45f}, {0.85f, 0.75f, 0.40f}, {0.75f, 0.55f, 0.85f}, {0.45f, 0.80f, 0.75f}};
    const float direction[4] = {0.4f, 1, 0.6f, 0}, ambient[4] = {0.35f, 0.35f, 0.35f, 1}, diffuse[4] = {0.75f, 0.75f, 0.75f, 1};
    glEnable(GL_LIGHTING), glEnable(GL_LIGHT0), glEnable(GL_COLOR_MATERIAL), glEnable(GL_NORMALIZE), glEnable(GL_TEXTURE_2D);
    glEnable(GL_TEXTURE_GEN_S), glEnable(GL_TEXTURE_GEN_T), glEnable(GL_POLYGON_OFFSET_FILL);
    glPolygonOffset(1, 1);
    glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, 1);
    glLightfv(GL_LIGHT0, GL_POSITION, direction), glLightfv(GL_LIGHT0, GL_AMBIENT, ambient), glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuse);
    const auto fixed = [&](size_t i) { return world.bodies()[i].isStatic() && !world.asleep(BodyId(int(i))) && !isDriven(world.bodies()[i]); };
    for (size_t i = 0; i < world.bodies().size(); ++i) {
        if (fixed(i)) continue;
        const float shade = world.asleep(BodyId(int(i))) ? 0.6f : 1;
        glColor3f(shade * palette[i % 6][0], shade * palette[i % 6][1], shade * palette[i % 6][2]);
        drawBody(world.bodies()[i], 0.1);
    }
    glEnable(GL_BLEND), glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA), glDepthMask(GL_FALSE);
    glColor4f(0.55f, 0.55f, 0.60f, 0.45f);
    for (size_t i = 0; i < world.bodies().size(); ++i)
        if (fixed(i)) drawBody(world.bodies()[i], 1);
    glDisable(GL_BLEND), glDepthMask(GL_TRUE);
    glDisable(GL_LIGHTING), glDisable(GL_COLOR_MATERIAL), glDisable(GL_TEXTURE_2D), glDisable(GL_TEXTURE_GEN_S), glDisable(GL_TEXTURE_GEN_T), glDisable(GL_POLYGON_OFFSET_FILL);
}

} // namespace

void render(const DebugGeometry& geometry, const Camera& camera, int left, int width, int height, const World* filled) {
    glClearColor(0.07f, 0.07f, 0.07f, 1);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glViewport(left, 0, width, height);

    // Перспектива: в кадр попадает всё между ближней и дальней плоскостью.
    const double nearPlane = Camera::kNearPlane, farPlane = 500, halfHeight = Camera::kTanHalfFov * nearPlane, aspect = double(width) / height;
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glFrustum(-halfHeight * aspect, halfHeight * aspect, -halfHeight, halfHeight, nearPlane, farPlane);
    glMatrixMode(GL_MODELVIEW);
    glLoadMatrixd(camera.view().data());

    glEnable(GL_DEPTH_TEST);
    if (filled) drawFilled(*filled);
    glLineWidth(1.5f);
    glBegin(GL_LINES);
    for (const DebugLine& line : geometry.lines) {
        setColour(line.kind);
        glVertex3d(line.from.x, line.from.y, line.from.z);
        glVertex3d(line.to.x, line.to.y, line.to.z);
    }
    glEnd();

    glDisable(GL_DEPTH_TEST); // точки контакта видны и сквозь тела
    glPointSize(7);
    glBegin(GL_POINTS);
    for (const DebugPoint& point : geometry.points) {
        setColour(point.kind);
        glVertex3d(point.position.x, point.position.y, point.position.z);
    }
    glEnd();
}

bool saveScreenshot(const std::string& path, int width, int height) {
    std::vector<unsigned char> pixels(size_t(width) * size_t(height) * 3);
    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, pixels.data());
    std::FILE* file = std::fopen(path.c_str(), "wb");
    if (!file) return false;
    std::fprintf(file, "P6\n%d %d\n255\n", width, height);
    // OpenGL отдаёт строки снизу вверх, файл хранит сверху вниз.
    for (int row = height - 1; row >= 0; --row) std::fwrite(&pixels[size_t(row) * size_t(width) * 3], 1, size_t(width) * 3, file);
    std::fclose(file);
    return true;
}
