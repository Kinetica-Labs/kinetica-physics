# Окно просмотра на OpenGL. Внешние библиотеки нужны только ему, движок от них не зависит:
#   GLFW — окно и ввод (zlib): берётся установленная в системе, иначе скачивается версия 3.4;
#   Dear ImGui — текст, кнопки и галочки в окне (MIT): скачивается версия 1.92.7.
# Скачанное закреплено контрольной суммой; перечень — в THIRD_PARTY_NOTICES.md.
include(FetchContent)

find_package(glfw3 3.3 QUIET)
if(NOT glfw3_FOUND)
  set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
  set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
  set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
  set(GLFW_INSTALL OFF CACHE BOOL "" FORCE)
  set(GLFW_BUILD_WAYLAND OFF CACHE BOOL "" FORCE) # X11 хватает; Wayland потребовал бы wayland-scanner
  FetchContent_Declare(glfw
    URL https://github.com/glfw/glfw/archive/7b6aead9fb88b3623e3b3725ebb42670cbe4c579.tar.gz
    URL_HASH SHA256=169d49096e339caabbc12e1762273c5cb9130a535a282ba46cf5c14ab6e55694
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
  FetchContent_MakeAvailable(glfw)
endif()
find_package(OpenGL REQUIRED)

# У ImGui нет своей сборки: его исходники просто добавляются в проект. Рисует он через старый
# OpenGL с фиксированным конвейером — тот же, которым окно рисует линии.
FetchContent_Declare(imgui
  URL https://github.com/ocornut/imgui/archive/refs/tags/v1.92.7.tar.gz
  URL_HASH SHA256=b21b14ce1ef6dd4d85fa54f68f8449a9be94f3b453aba7fe4ea2a9764e43a5ef
  DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
FetchContent_MakeAvailable(imgui)
add_library(imgui STATIC
  ${imgui_SOURCE_DIR}/imgui.cpp
  ${imgui_SOURCE_DIR}/imgui_draw.cpp
  ${imgui_SOURCE_DIR}/imgui_tables.cpp
  ${imgui_SOURCE_DIR}/imgui_widgets.cpp
  ${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
  ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl2.cpp)
target_include_directories(imgui SYSTEM PUBLIC ${imgui_SOURCE_DIR} ${imgui_SOURCE_DIR}/backends)
target_link_libraries(imgui PUBLIC glfw OpenGL::GL)

configure_file(data/hull-events.csv data/hull-events.csv COPYONLY)
add_executable(viewer main.cpp Viewer.cpp ImpulseTool.cpp Panel.cpp GlRenderer.cpp Scenes.cpp)
target_link_libraries(viewer PRIVATE kinetica-physics kinetica-scenes imgui)
target_compile_options(viewer PRIVATE -Wall -Wextra)
target_compile_definitions(viewer PRIVATE KP_HULL_EVENTS_FILE="${CMAKE_CURRENT_BINARY_DIR}/data/hull-events.csv")
