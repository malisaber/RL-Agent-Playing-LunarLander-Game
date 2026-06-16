cmake_minimum_required(VERSION 3.24)

project(LunarLander VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(LunarLander
    src/Lunar_Lander_Environment.cpp
    src/NN.cpp
    src/Physix.cpp
    src/Shape.cpp
    src/Utilities.cpp
    src/Vapor.cpp
    src/main.cpp
)

target_include_directories(LunarLander PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

if (MSVC)
    target_compile_definitions(LunarLander PRIVATE
        NOMINMAX
        WIN32_LEAN_AND_MEAN
        _CRT_SECURE_NO_WARNINGS
    )
endif()

find_package(glad CONFIG REQUIRED)
find_package(glfw3 CONFIG REQUIRED)
find_package(glm CONFIG REQUIRED)
find_package(SFML COMPONENTS graphics window system CONFIG REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui videoio)
find_package(Bullet CONFIG REQUIRED)
find_package(unofficial-box2d CONFIG REQUIRED)
find_package(Torch REQUIRED)

separate_arguments(TORCH_CXX_FLAGS_LIST NATIVE_COMMAND "${TORCH_CXX_FLAGS}")

target_compile_options(LunarLander PRIVATE ${TORCH_CXX_FLAGS_LIST})

target_link_libraries(LunarLander PRIVATE
    glad::glad
    glfw
    glm::glm
    SFML::Graphics
    SFML::Window
    SFML::System
    ${OpenCV_LIBS}
    BulletDynamics
    BulletCollision
    LinearMath
    unofficial::box2d::box2d
    ${TORCH_LIBRARIES}
)

set_target_properties(LunarLander PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)
