# Приборы: не проверки «да/нет», а замеры, из которых выводятся решения .
foreach(study GeometryErrors Stack RotationMap DropStatistics TreeStatistics CcdStatistics CaseTable)
  string(REGEX REPLACE "([a-z])([A-Z])" "\\1-\\2" name ${study})
  string(TOLOWER study-${name} name)
  add_executable(${name} ${study}.cpp)
  target_link_libraries(${name} PRIVATE kinetica-physics)
endforeach()
add_executable(study-standard-scenes StandardScenes.cpp)
target_link_libraries(study-standard-scenes PRIVATE kinetica-scenes)
add_executable(study-state-hash StateHash.cpp)
target_link_libraries(study-state-hash PRIVATE kinetica-scenes)
if(NOT WIN32) # подмена malloc и цепочки вызовов — средствами Linux
  add_executable(study-allocations Allocations.cpp)
  target_link_libraries(study-allocations PRIVATE kinetica-scenes ${CMAKE_DL_LIBS})
  target_link_options(study-allocations PRIVATE -rdynamic) # имена функций в цепочках вызовов
endif()
add_executable(study-frame-times FrameTimes.cpp)
target_link_libraries(study-frame-times PRIVATE kinetica-scenes)
add_executable(study-frame-stages FrameStages.cpp)
target_link_libraries(study-frame-stages PRIVATE kinetica-scenes)
target_include_directories(study-frame-stages PRIVATE ${PROJECT_SOURCE_DIR}/validation/engines) # трёхмерная пирамида замера скорости
add_executable(study-scale Scale.cpp)
target_link_libraries(study-scale PRIVATE kinetica-physics)
add_executable(study-overlap-spread OverlapSpread.cpp)
target_link_libraries(study-overlap-spread PRIVATE kinetica-scenes)
add_executable(study-window-pull WindowPull.cpp)
target_link_libraries(study-window-pull PRIVATE kinetica-scenes)
find_package(Threads REQUIRED)
target_link_libraries(study-case-table PRIVATE Threads::Threads) # клетки таблицы считаются параллельно
add_executable(study-exact-scenes ExactScenes.cpp)
target_link_libraries(study-exact-scenes PRIVATE kinetica-scenes)
add_executable(study-pull-cube PullCube.cpp)
target_link_libraries(study-pull-cube PRIVATE kinetica-physics)
target_include_directories(study-pull-cube PRIVATE ${PROJECT_SOURCE_DIR}/validation/engines) # сцена pyramid55 и мерка перекрытия
