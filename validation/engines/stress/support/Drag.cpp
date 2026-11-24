// Мышь без отрисовки: пиксель -> луч Camera -> Viewer::grabAt -> dragTo -> настоящий шаг мира.
#include "Viewer.h"
#include <math/WorkerPool.h>
#include <array>
#include <cstdint>
#include <cstdio>
#include <cstring>

using namespace kp;
namespace {
World pyramid() {
    World w;
    w.addBody(Body::box({5,.5,5},0,{0,-.5,0}));
    for(int row=0; row<10; ++row)
        for(int k=0; k<10-row; ++k)
            w.addBody(Body::box({.2,.2,.2},1000,{Real(.4)*(Real(k)-Real(9-row)/2),Real(.2)+Real(.4)*row,0}));
    return w;
}
constexpr int width=1110, height=820;
}
const std::vector<Scene>& scenes() {
    static const std::vector<Scene> only{{"pyramid55",pyramid,{0,1.4,0},9}};
    return only;
}

// То же представление 13 чисел тела и FNV-1a, что у validation/engines/common.h.
std::uint64_t finalHash(const World& world) {
    std::uint64_t hash=14695981039346656037ull;
    for (const auto& body : world.bodies()) {
        const auto p=body.position, v=body.linearVelocity(), w=body.angularVelocity();
        const auto q=body.orientation;
        const double values[]={p.x,p.y,p.z,q.w,q.x,q.y,q.z,v.x,v.y,v.z,w.x,w.y,w.z};
        unsigned char bytes[sizeof values];
        std::memcpy(bytes,values,sizeof values);
        for (unsigned char byte : bytes) hash=(hash^byte)*1099511628211ull;
    }
    return hash;
}

// Время одного World::step в той же сцене без мыши, после 23 с оседания.
// Чтение поз, расчёт хэша и вывод идут после 120 измеренных кадров.
int restTime(bool support) {
    Viewer view;
    view.load(0);
    view.measure();
    view.shockPropagation = support;
    for (int frame=0; frame<1380; ++frame) view.step();
    std::array<double,120> us{};
    for (double& sample : us) {
        view.step();
        sample = 1000 * view.lastStep;
    }
    const auto hash=finalHash(view.world);
    std::puts("frame,step_us,final_hash");
    for (size_t i=0; i<us.size(); ++i)
        std::printf("%zu,%.3f,%016llx\n",i+1,us[i],static_cast<unsigned long long>(hash));
    return 0;
}

int main(int argc, char** argv) {
    WorkerPool::shared().resize(1);
    if (argc == 2 && std::strcmp(argv[1], "--switch") == 0) {
        Viewer view;
        view.load(0);
        if (!view.shockPropagation || !view.world.shockPropagation) return 1;
        view.shockPropagation = false;
        view.step();
        if (view.frame != 1 || view.world.shockPropagation) return 1;
        view.load(0);
        if (view.world.shockPropagation) return 1;
        view.shockPropagation = true;
        view.step();
        if (view.frame != 1 || !view.world.shockPropagation) return 1;
        view.shockPropagation = false;
        view.measure();
        return view.shockPropagation && view.world.shockPropagation && view.asMeasured() ? 0 : 1;
    }
    if (argc >= 2 && std::strcmp(argv[1], "--rest-time") == 0) {
        if (argc == 2) return restTime(true);
        if (argc == 3 && std::strcmp(argv[2], "--shock-off") == 0) return restTime(false);
        if (argc == 3 && std::strcmp(argv[2], "--shock-on") == 0) return restTime(true);
        return 2;
    }
    std::puts("requested,selected,mass_kg,ray_anchor_px,first_target_m,first_body_mm,peak_shift_m,end_shift_m,others_over_5cm,max_other_m,late_mean_frame_ms");
    for(int desired : {55,33,5}){
        Viewer view;
        view.load(0);
        view.measure();
        for(int i=0; i<1380; ++i) view.step();
        double x=0,y=0;
        if(!view.camera.project(view.world.bodies()[desired].position,width,height,x,y)) return 2;
        const Vector3 direction=view.camera.ray(x,y,width,height);
        if(!view.grabAt(direction)){std::printf("%d,-1,0,0,0,0,0,0,0,0,0\n",desired);continue;}
        const int held=view.grab.body;
        std::vector<Vector3> start;
        for(const auto& body : view.world.bodies()) start.push_back(body.position);
        const auto& tether=*view.grab.tether;
        const Vector3 firstAnchor=view.world.bodies()[tether.a].pose().point(tether.anchorA);
        double pixX=0,pixY=0;
        view.camera.project(firstAnchor,width,height,pixX,pixY);
        Real target=0,firstMove=0,peak=0,other=0;
        double ms=0;
        int others=0;
        for(int frame=1;frame<=150;++frame){
            if(frame<=90){
                const double f=std::min(1.,frame/30.);
                view.dragTo(view.camera.ray(x+125*f,y-65*f,width,height));
                if(frame==1) target=(view.world.bodies()[tether.a].pose().point(tether.anchorA)-firstAnchor).length();
            } else if(frame==91) view.release();
            view.step();
            const Real delta=(view.world.bodies()[held].position-start[held]).length();
            if(frame==1) firstMove=delta;
            peak=std::max(peak,delta);
            int moved=0;
            for(size_t j=1;j<start.size();++j) if(int(j)!=held){
                const Real d=(view.world.bodies()[j].position-start[j]).length();
                moved+=d>.05; other=std::max(other,d);
            }
            others=std::max(others,moved);
            ms+=view.lastStep;
        }
        std::printf("%d,%d,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%d,%.9g,%.9g\n",desired,held,
                    1/view.world.bodies()[held].mass.inverseMass,std::hypot(pixX-x,pixY-y),target,1000*firstMove,peak,
                    (view.world.bodies()[held].position-start[held]).length(),others,other,ms/150);
        std::fflush(stdout);
    }
}
