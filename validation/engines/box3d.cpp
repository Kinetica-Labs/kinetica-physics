// Box3D на общих сценах (common.h). В сборку движка не входит: исходники — research/refs/box3d.
// Собирает и прогоняет `validation/run.py box3d`; вручную:
//   cmake -S research/refs/box3d -B build/box3d -DCMAKE_BUILD_TYPE=Release -DBOX3D_SAMPLES=OFF \
//         -DBOX3D_UNIT_TESTS=OFF -DBOX3D_BENCHMARKS=OFF && cmake --build build/box3d --target box3d
//   g++ -O3 -DNDEBUG -std=c++17 -Iresearch/refs/box3d/include validation/engines/box3d.cpp \
//       build/box3d/src/libbox3d.a -lpthread -o build/box3d/compare-box3d
//
//   compare-box3d [ПОДШАГОВ [ГЦ_КОНТАКТА ГЦ_СОЧЛЕНЕНИЙ]]     по умолчанию 4 подшага, 30 и 60 Гц, как в Box3D
#include "stress/Stress1000.h"
#include <box3d/box3d.h>

#include <cstdlib>

static b3Vec3 toBox3d(V3 v) { return {float(v.x), float(v.y), float(v.z)}; }

static void addJoint(b3WorldId world, const std::vector<b3BodyId>& ids, const SceneSpec& s, const JointSpec& j, float hertz) {
    const BoxSpec &a = s.boxes[size_t(j.a)], &b = s.boxes[size_t(j.b)];
    const V3 la = unrotate(a.turn, j.anchor - a.at), lb = unrotate(b.turn, j.anchor - b.at);
    if (j.weld) {
        b3WeldJointDef d = b3DefaultWeldJointDef();
        d.base.bodyIdA = ids[size_t(j.a)], d.base.bodyIdB = ids[size_t(j.b)];
        d.base.localFrameA.p = toBox3d(la), d.base.localFrameB.p = toBox3d(lb), d.base.constraintHertz = hertz;
        b3CreateWeldJoint(world, &d);
    } else {
        b3RevoluteJointDef d = b3DefaultRevoluteJointDef();
        d.base.bodyIdA = ids[size_t(j.a)], d.base.bodyIdB = ids[size_t(j.b)];
        d.base.localFrameA.p = toBox3d(la), d.base.localFrameB.p = toBox3d(lb), d.base.constraintHertz = hertz;
        b3CreateRevoluteJoint(world, &d);
    }
}

int main(int argc, char** argv) {
    const int substeps = argc > 1 ? std::atoi(argv[1]) : 4;
    const float contactHertz = argc > 3 ? float(std::atof(argv[2])) : b3DefaultWorldDef().contactHertz;
    const float jointHertz = argc > 3 ? float(std::atof(argv[3])) : b3DefaultWeldJointDef().base.constraintHertz;
    const std::string stiffness = argc > 3 ? " " + std::string(argv[2]) + "/" + argv[3] + " Гц" : "";
    const bool stressRun = stress::requested();
    if (stressRun) stress::require(substeps == 1, "stress requires one substep per external call");
    stress::compareOrStress("box3d " + std::to_string(substeps) + stiffness, [&](const SceneSpec& s, const Play& play) {
        b3WorldDef wd = b3DefaultWorldDef();
        wd.contactHertz = contactHertz;
        wd.gravity = {0, -float(kGravity), 0};
        wd.enableSleep = false;
        const b3WorldId world = b3CreateWorld(&wd);
        if (stressRun) b3World_EnableWarmStarting(world, true);
        std::vector<b3BodyId> ids;
        for (const BoxSpec& b : s.boxes) {
            b3BodyDef bd = b3DefaultBodyDef();
            bd.type = b.density == 0 ? b3_staticBody : b3_dynamicBody;
            bd.position = toBox3d(b.at), bd.linearVelocity = toBox3d(b.velocity);
            bd.rotation = {{float(b.turn.x), float(b.turn.y), float(b.turn.z)}, float(b.turn.w)};
            const b3BodyId id = b3CreateBody(world, &bd);
            b3ShapeDef sd = b3DefaultShapeDef();
            sd.density = float(b.density == 0 ? 1000 : b.density);
            sd.baseMaterial.friction = 0.5f;
            if (stressRun) sd.baseMaterial.restitution = 0;
            b3BoxHull box = b3MakeBoxHull(float(b.half.x), float(b.half.y), float(b.half.z));
            b3CreateHullShape(id, &sd, &box.base);
            ids.push_back(id);
        }
        for (const JointSpec& j : s.joints) addJoint(world, ids, s, j, jointHertz);
        play([&](double dt) { b3World_Step(world, float(dt), substeps); }, [&] {
            std::vector<State> st;
            for (b3BodyId id : ids) {
                const b3Pos p = b3Body_GetPosition(id);
                const b3Quat q = b3Body_GetRotation(id);
                const b3Vec3 v = b3Body_GetLinearVelocity(id), w = b3Body_GetAngularVelocity(id);
                st.push_back({{p.x, p.y, p.z}, {q.s, q.v.x, q.v.y, q.v.z}, {v.x, v.y, v.z}, {w.x, w.y, w.z}, !b3Body_IsAwake(id)});
            }
            return st; });
        b3DestroyWorld(world);
    });
}
