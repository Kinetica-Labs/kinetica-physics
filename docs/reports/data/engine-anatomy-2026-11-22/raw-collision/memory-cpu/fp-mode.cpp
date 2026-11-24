// Isolate x86 denormal handling without recompiling any solver object.
#include <xmmintrin.h>
#include <cstdio>
#include <cstdlib>

int originalGripMain(int argc, char** argv);
int main(int argc, char** argv) {
    const unsigned original=_mm_getcsr();
    constexpr unsigned flags=0x3f, flush=(1u<<15)|(1u<<6);
    const char* requested=std::getenv("KP_AUDIT_FLUSH");
    const bool enable=requested && requested[0]=='1';
    const unsigned start=(original & ~(flags|flush)) | (enable ? flush : 0);
    _mm_setcsr(start);
    const int result=originalGripMain(argc,argv);
    const unsigned finish=_mm_getcsr();
    _mm_setcsr(original);
    std::fprintf(stderr,"initial=%08x start=%08x finish=%08x denormal_operand=%d\n",
                 original,start,finish,int((finish & 2)!=0));
    return result;
}
