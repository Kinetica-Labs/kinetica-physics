# Примеры идут по слоям движка снизу вверх: каталог — слой, файл — одна возможность.
# Начинать с 05_world/FallingBox; остальные показывают, из чего мир собран.
# Стандартный набор сцен — общий для окна просмотра и прибора с замерами.
add_library(kinetica-scenes STATIC scenes/StandardScenes.cpp scenes/ExactScenes.cpp)
target_link_libraries(kinetica-scenes PUBLIC kinetica-physics)
target_compile_options(kinetica-scenes PRIVATE -Wall -Wextra)

foreach(example
    01_math/Rotation
    02_geometry/Distance 02_geometry/Contact 02_geometry/TimeOfImpact 02_geometry/Hull
    03_collision/Pairs
    04_dynamics/FreeSpin
    05_world/FallingBox 05_world/BouncingBall 05_world/BoxStack 05_world/DebugView
    05_world/Pendulum 05_world/PushedBox 05_world/Platform 05_world/Rail 05_world/Explosion 05_world/FloatingBox 05_world/GearTrain)
  get_filename_component(name ${example} NAME)
  add_executable(${name} ${example}.cpp)
  target_link_libraries(${name} PRIVATE kinetica-physics)
  target_include_directories(${name} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
  target_compile_options(${name} PRIVATE -Wall -Wextra)
endforeach()
add_test(NAME floating-box COMMAND FloatingBox)
add_test(NAME gear-train COMMAND GearTrain)
