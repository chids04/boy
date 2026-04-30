cmake_minimum_required(VERSION 3.21)

project(boy C)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_library(GbCore
    src/common.h
    src/cpu.h
    src/cpu.c
    src/cpu_utils.c
    src/utils.h
    src/mmu.h
    src/mmu.c
    src/timer.h
    src/timer.c
    src/boy.h
    src/boy.c
    src/log.h
    src/log.c
    src/ppu.h
    src/ppu.c
    src/test.h
    src/test.c
)


target_include_directories(GbCore PUBLIC src)
target_compile_options(GbCore PRIVATE  -Wall -Wextra)

# raylib depenceny

set(RAYLIB_VERSION 5.5)
find_package(raylib ${RAYLIB_VERSION} QUIET) # QUIET or REQUIRED
if (NOT raylib_FOUND) # If there's none, fetch and build raylib
  include(FetchContent)
  FetchContent_Declare(
    raylib
    DOWNLOAD_EXTRACT_TIMESTAMP OFF
    URL https://github.com/raysan5/raylib/archive/refs/tags/${RAYLIB_VERSION}.tar.gz
  )
  FetchContent_GetProperties(raylib)
  if (NOT raylib_POPULATED) # Have we downloaded raylib yet?
    set(FETCHCONTENT_QUIET NO)
    FetchContent_MakeAvailable(raylib)
  endif()
endif()

set(RAYGUI_VERSION 4.0)
find_package(raygui ${RAYGUI_VERSION} QUIET) # QUIET or REQUIRED
if (NOT raygui_FOUND) # If there's none, fetch and build raylib
  FetchContent_Declare(
      raygui
      DOWNLOAD_EXTRACT_TIMESTAMP OFF
      URL https://github.com/raysan5/raygui/archive/refs/tags/${RAYGUI_VERSION}.tar.gz
    )

  FetchContent_GetProperties(raygui)
  if (NOT raygui_POPULATED) # Have we downloaded raylib yet?
    set(FETCHCONTENT_QUIET NO)
    FetchContent_MakeAvailable(raygui)
  endif()
endif()

set(RAYGUI_ROOT ${raygui_SOURCE_DIR})
add_subdirectory(${raygui_SOURCE_DIR}/projects/CMake ${CMAKE_BINARY_DIR}/raygui)

# get raygui dependency
#
#

add_executable(
    ${PROJECT_NAME}
    src/main.c
)

target_link_libraries(${PROJECT_NAME} GbCore raylib raygui)

if (${PLATFORM} STREQUAL "Web")
  set_target_properties(${PROJECT_NAME} PROPERTIES SUFFIX ".html") # Tell Emscripten to build an example.html file.
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -s USE_GLFW=3 -s ASSERTIONS=1 -s WASM=1 -s ASYNCIFY -s GL_ENABLE_GET_PROC_ADDRESS=1")
endif()

# Checks if OSX and links appropriate frameworks (Only required on MacOS)
if (APPLE)
  target_link_libraries(${PROJECT_NAME} "-framework IOKit")
  target_link_libraries(${PROJECT_NAME} "-framework Cocoa")
  target_link_libraries(${PROJECT_NAME} "-framework OpenGL")
endif()

# building tests

# include(FetchContent)
# FetchContent_Declare(
#   googletest
#   URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
#   DOWNLOAD_EXTRACT_TIMESTAMP false
# )

# FetchContent_MakeAvailable(googletest)



# enable_testing()

# add_executable(run_tests
#     test/main.cpp
#     test/cpu/instructions.cpp
# )


# target_include_directories(run_tests PUBLIC
#     test/cpu/
# )

# target_link_libraries(run_tests GbCore GTest::gtest)

# include(GoogleTest)
# gtest_discover_tests(run_tests)
