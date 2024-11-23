cmake_minimum_required(VERSION 3.10)
project(n-body-simulation)

# Set the C++ standard (adjust based on your requirements)
set(CMAKE_CXX_STANDARD 26)

# Set the paths to the Raylib headers and library
set(RAYLIB_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/include)
set(RAYLIB_LIBRARY ${CMAKE_SOURCE_DIR}/lib/libraylib.a)

# Add the include directories to the project
include_directories(${RAYLIB_INCLUDE_DIR})

# Add the executable
add_executable(n-body-simulation main.cpp
        simulation/planet.cpp
        simulation/planet.h
        simulation/rigid_body.cpp
        simulation/rigid_body.h
        simulation/simulation_governor.cpp
        simulation/simulation_governor.h
        simulation/constants.h
        include/style_amber.h
)
find_package(nlohmann_json CONFIG REQUIRED)

find_package(glaze CONFIG REQUIRED)
#target_link_libraries(n-body-simulation PRIVATE glaze::glaze)

#target_link_libraries(n-body-simulation PRIVATE nlohmann_json::nlohmann_json)
# Link raylib and other necessary libraries (like OpenGL, pthread, etc.)
target_link_libraries(n-body-simulation ${RAYLIB_LIBRARY} GL pthread dl rt X11)

# Optional: Set any additional compile flags if needed
# target_compile_options(n-body-simulation PRIVATE -Wall -Wextra)