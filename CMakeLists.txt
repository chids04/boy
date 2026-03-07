cmake_minimum_required(VERSION 3.21)

project(boy C CXX)

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
)

target_include_directories(GbCore PUBLIC src)
target_compile_options(GbCore PRIVATE  -Wall -Wextra)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(
    gb_emu
    src/main.c
)

target_link_libraries(gb_emu GbCore)

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
