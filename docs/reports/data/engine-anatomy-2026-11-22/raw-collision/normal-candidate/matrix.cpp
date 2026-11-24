#include <dynamics/event_driven/ContactSystem.h>
#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <io.h>
using namespace kp;
namespace {
std::uint32_t state = 19730819;
std::uint32_t roll() { state = 1664525u * state + 1013904223u; return state; }
Real scalar() { return (int(roll() >> 17) - 16384) / 4096.0; }
Vector3 vec() { return {scalar(), scalar(), scalar()}; }
template<class T> void send(T v) { if (std::fwrite(&v,sizeof(v),1,stdout)!=1) std::abort(); }
}
int main() {
    _setmode(_fileno(stdout), _O_BINARY);
    for (unsigned caseId = 0; caseId < 1024; ++caseId) {
        const unsigned count = 1 + roll()%12, n = caseId%25;
        std::vector<Body> bodies(count);
        for (Body& body : bodies) {
            if (roll()%5 || caseId%7 != 0) {
                const Vector3 p=vec();
                body.mass={.5 + std::abs(scalar()),{1 + std::abs(p.x),2 + std::abs(p.y),3 + std::abs(p.z)}};
            }
            body.orientation=Quaternion{1,scalar(),scalar(),scalar()}.unit();
        }
        StepArray<ContactSystem::Line> lines;
        for (unsigned k=0;k<n;++k) {
            const int a=int(roll()%count);
            const int b=(k%4==0 ? a : int(roll()%count));
            lines.push_back({a,b,vec(),vec(),vec()});
        }
        const auto matrix=ContactSystem::responseMatrix(bodies,lines);
        send(std::uint32_t(matrix.size()));
        for (const auto& row : matrix) {
            send(std::uint32_t(row.size()));
            for (const auto& e:row) { send(std::uint32_t(e.column));send(e.value); }
        }
    }
}
