cmake_minimum_required(VERSION 3.16)
project(kinetica-physics LANGUAGES CXX)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release)
endif()

include(CTest)
add_subdirectory(src)
add_subdirectory(examples)

option(KP_VIEWER "Собрать интерактивный просмотрщик OpenGL" OFF)
if(KP_VIEWER)
  add_subdirectory(viewer)
endif()

if(BUILD_TESTING)
  add_subdirectory(tests)
endif()
