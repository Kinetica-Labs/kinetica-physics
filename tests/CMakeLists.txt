# Модульные проверки движка: каждая сверяет одну функцию с известным точным ответом.
add_executable(unit-tests main.cpp GeometryTests.cpp HullTests.cpp CollisionTests.cpp DynamicsTests.cpp)
target_link_libraries(unit-tests PRIVATE kinetica-physics)
target_compile_options(unit-tests PRIVATE -Wall -Wextra)
add_test(NAME unit-tests COMMAND unit-tests)
# Правило детерминизма: в библиотеке нет вызовов sin, cos, exp и прочих функций libm.
add_test(NAME no-libm COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/check_no_libm.sh $<TARGET_FILE:kinetica-physics>)

# Леммы площадки касания проверяет решатель Z3, если он есть в системе (proofs/).
find_program(Z3_SOLVER z3)
if(Z3_SOLVER)
  add_test(NAME contact-patch-lemmas COMMAND ${CMAKE_SOURCE_DIR}/proofs/check.sh)
endif()
