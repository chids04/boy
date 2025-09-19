cmake_minimum_required(VERSION 3.14)

project(boy C CXX)

add_library(GbCore
    src/gbcore.c
    src/cpu.c
)

target_include_directories(GbCore PUBLIC src)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
  DOWNLOAD_EXTRACT_TIMESTAMP false
)

FetchContent_MakeAvailable(googletest)

enable_testing()

add_executable(run_tests test/cpu/instructions.cpp)

target_link_libraries(run_tests PRIVATE GbCore GTest::gtest_main)

include(GoogleTest)
gtest_discover_tests(run_tests)
