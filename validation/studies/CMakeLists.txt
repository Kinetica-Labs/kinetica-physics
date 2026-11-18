# Приборы: не проверки «да/нет», а замеры, из которых выводятся решения .
foreach(study GeometryErrors Stack RotationMap CubeOnCorner DropStatistics TreeStatistics CcdStatistics CaseTable)
  string(REGEX REPLACE "([a-z])([A-Z])" "\\1-\\2" name ${study})
  string(TOLOWER study-${name} name)
  add_executable(${name} ${study}.cpp)
  target_link_libraries(${name} PRIVATE kinetica-physics)
endforeach()
add_executable(study-standard-scenes StandardScenes.cpp)
target_link_libraries(study-standard-scenes PRIVATE kinetica-scenes)
add_executable(study-state-hash StateHash.cpp)
target_link_libraries(study-state-hash PRIVATE kinetica-scenes)
find_package(Threads REQUIRED)
target_link_libraries(study-case-table PRIVATE Threads::Threads) # клетки таблицы считаются параллельно
add_executable(study-exact-scenes ExactScenes.cpp)
target_link_libraries(study-exact-scenes PRIVATE kinetica-scenes)
add_executable(study-pull-cube PullCube.cpp)
target_link_libraries(study-pull-cube PRIVATE kinetica-physics)
target_include_directories(study-pull-cube PRIVATE ${PROJECT_SOURCE_DIR}/validation/engines) # сцена pyramid55 и мерка перекрытия
