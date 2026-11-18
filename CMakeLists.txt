# kinetica-physics — движок твёрдых тел. Одна сборка на всё:
#   cmake -B build && cmake --build build -j && ctest --test-dir build
cmake_minimum_required(VERSION 3.16)
project(kinetica-physics CXX)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
enable_testing()

add_subdirectory(src)                # библиотека libkinetica-physics.a
add_subdirectory(tests)              # модульные проверки: unit-tests
add_subdirectory(examples)           # примеры использования
add_subdirectory(validation/studies) # приборы точности нового ядра
add_subdirectory(validation/engines) # наш движок на сценах сравнения с Box3D
# Окно просмотра на OpenGL — по желанию, потому что ему нужна внешняя библиотека GLFW:
#   cmake -B build -DKP_VIEWER=ON
option(KP_VIEWER "Окно просмотра на OpenGL (GLFW)" OFF)
if(KP_VIEWER)
  add_subdirectory(viewer)
endif()
# Сравнение со старым ядром (validation/) собирает validation/run.py: ему нужна библиотека phys_realflow.
