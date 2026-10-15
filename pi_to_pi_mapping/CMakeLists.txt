cmake_minimum_required(VERSION 3.16)
project(ARES_PiToPiMapping LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(ares_shared
    src/protocol.cpp
)
target_include_directories(ares_shared PUBLIC include)

add_executable(rover src/rover.cpp src/mock_fpga.cpp)
target_link_libraries(rover PRIVATE ares_shared)

add_executable(operator src/operator.cpp src/grid_map.cpp)
target_link_libraries(operator PRIVATE ares_shared)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(ares_shared PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(rover PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(operator PRIVATE -Wall -Wextra -Wpedantic)
endif()
