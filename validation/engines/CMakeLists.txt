# Наш движок на общих сценах сравнения; Box3D и ReactPhysics3D собираются по рецепту в их файлах.
add_executable(compare-ours ours.cpp)
target_link_libraries(compare-ours PRIVATE kinetica-physics)
target_compile_options(compare-ours PRIVATE -Wall -Wextra -Wno-missing-field-initializers) # сцены — короткие списки полей
