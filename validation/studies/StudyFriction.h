#pragma once
#include <cstdlib>
#include <string_view>

namespace study {
// Один переключатель для сравнительных приборов; настройка движка задаётся через World::soft.
inline bool coupledFrictionRequested() {
    const char* option = std::getenv("KP_COUPLED_FRICTION");
    return option && std::string_view(option) == "1";
}
}
