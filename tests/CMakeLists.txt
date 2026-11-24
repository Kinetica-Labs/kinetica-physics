add_executable(unit-tests main.cpp GeometryTests.cpp HullTests.cpp CollisionTests.cpp
  ContactPatchTests.cpp DynamicsTests.cpp FrictionTests.cpp GearTests.cpp LinkageTests.cpp
  WheelTests.cpp VehicleTests.cpp PropertyTests.cpp)
target_link_libraries(unit-tests PRIVATE kinetica-physics)
target_compile_options(unit-tests PRIVATE -Wall -Wextra)
add_test(NAME unit-tests COMMAND unit-tests)

function(check_math_symbols name library)
  add_test(NAME ${name} COMMAND ${CMAKE_COMMAND}
    "-DNM=${CMAKE_NM}" "-DLIBRARY=$<TARGET_FILE:${library}>"
    -P "${CMAKE_CURRENT_SOURCE_DIR}/check_no_libm.cmake")
endfunction()
check_math_symbols(no-libm kinetica-physics)
check_math_symbols(no-libm-scenes kinetica-scenes)

if(UNIX)
  find_program(Z3_SOLVER z3)
  if(Z3_SOLVER)
    add_test(NAME contact-patch-lemmas COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/proofs/check.sh)
  endif()
  find_program(WHY3 why3 HINTS "$ENV{HOME}/.opam/default/bin")
  if(WHY3)
    add_test(NAME why3-models COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/proofs/why3/check.sh)
  endif()
endif()

add_subdirectory(regression)
