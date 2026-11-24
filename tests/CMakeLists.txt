# Модульные проверки движка: каждая сверяет одну функцию с известным точным ответом.
add_executable(unit-tests main.cpp GeometryTests.cpp HullTests.cpp CollisionTests.cpp DynamicsTests.cpp GearTests.cpp LinkageTests.cpp WheelTests.cpp VehicleTests.cpp PropertyTests.cpp)
target_link_libraries(unit-tests PRIVATE kinetica-physics)
target_compile_options(unit-tests PRIVATE -Wall -Wextra)
add_test(NAME unit-tests COMMAND unit-tests)
# Правило детерминизма: в библиотеке нет вызовов sin, cos, exp и прочих функций libm.
add_test(NAME no-libm COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/check_no_libm.sh $<TARGET_FILE:kinetica-physics>)
# То же для сцен: синус, вычисленный компилятором при сборке, и синус библиотеки расходятся в последнем
# бите, и сцена с ним строилась по-разному у g++ и clang++ (чаша, 2026-11-20).
add_test(NAME no-libm-scenes COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/check_no_libm.sh $<TARGET_FILE:kinetica-scenes>)

# Леммы площадки касания проверяет решатель Z3, если он есть в системе (proofs/).
find_program(Z3_SOLVER z3)
if(Z3_SOLVER)
  add_test(NAME contact-patch-lemmas COMMAND ${CMAKE_SOURCE_DIR}/proofs/check.sh)
endif()

# Модели Why3 (proofs/why3/) — если Why3 установлен (opam, переключатель default).
find_program(WHY3 why3 HINTS $ENV{HOME}/.opam/default/bin)
if(WHY3)
  add_test(NAME why3-models COMMAND ${CMAKE_SOURCE_DIR}/proofs/why3/check.sh)
endif()
