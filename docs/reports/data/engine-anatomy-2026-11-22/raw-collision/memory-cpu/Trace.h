#pragma once
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <cstdint>
#include <cstdio>
#include <cstring>

namespace {
constexpr int frames = 12;
struct Origin {
    void* returnPC[frames]{};
    unsigned short depth = 0;
    size_t requests = 0, bytes = 0;
};
Origin origins[256];
size_t used = 0;
void traceAllocation(size_t bytes) {
    void* pc[frames]{};
    const auto depth = CaptureStackBackTrace(0, frames, pc, nullptr);
    for (size_t i = 0; i <= used; ++i) {
        if (i == used) {
            if (used == 256) std::abort();
            std::memcpy(origins[i].returnPC, pc, depth * sizeof(void*));
            origins[i].depth = depth;
            ++used;
        }
        auto& site = origins[i];
        if (site.depth == depth && !std::memcmp(site.returnPC, pc, depth * sizeof(void*))) {
            ++site.requests;
            site.bytes += bytes;
            return;
        }
    }
}
}
void writeHeapOrigins() {
    const auto base = reinterpret_cast<std::uintptr_t>(GetModuleHandleW(nullptr));
    std::puts("origin,calls,bytes,depth,return_rvas");
    for (size_t i=0; i<used; ++i) {
        const auto& site=origins[i];
        std::printf("%zu,%zu,%zu,%u,", i, site.requests, site.bytes, unsigned(site.depth));
        for (unsigned j=0; j<site.depth; ++j) {
            const auto pc = reinterpret_cast<std::uintptr_t>(site.returnPC[j]);
            std::printf("%s%llx", j ? " " : "", static_cast<unsigned long long>(pc-base));
        }
        std::putchar('\n');
    }
}
