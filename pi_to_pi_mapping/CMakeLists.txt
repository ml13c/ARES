cmake_minimum_required(VERSION 3.16)
project(ARES_PiToPiMapping LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(ares_shared
    src/protocol.cpp
    src/localization.cpp
)
target_include_directories(ares_shared PUBLIC include)

add_executable(rover src/rover.cpp src/mock_fpga.cpp)
target_link_libraries(rover PRIVATE ares_shared)

add_executable(operator src/operator.cpp src/grid_map.cpp)
target_link_libraries(operator PRIVATE ares_shared)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    foreach(target ares_shared rover operator)
        target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic)
    endforeach()
endif()

include(CTest)
if(BUILD_TESTING)
    add_executable(ares_unit_tests tests/unit_tests.cpp src/grid_map.cpp)
    target_link_libraries(ares_unit_tests PRIVATE ares_shared)
    add_test(NAME ares_unit_tests COMMAND ares_unit_tests)

    # Python standard library only. It launches rover/operator over localhost.
    find_package(Python3 COMPONENTS Interpreter QUIET)
    if(Python3_Interpreter_FOUND)
        add_test(NAME rover_operator_integration
            COMMAND ${Python3_EXECUTABLE}
                    ${CMAKE_CURRENT_SOURCE_DIR}/tests/integration_test.py
                    $<TARGET_FILE:operator> $<TARGET_FILE:rover>)
        set_tests_properties(rover_operator_integration PROPERTIES TIMEOUT 15)
    endif()
endif()
