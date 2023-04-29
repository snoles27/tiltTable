# Set minimum required version of CMake
cmake_minimum_required(VERSION 3.12)

# Include build functions from Pico SDK
include(/Users/Sam/Code/pico/pico-sdk/external/pico_sdk_import.cmake)

# Set name of project (as PROJECT_NAME) and C/C   standards
project(tiltTable C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Creates a pico-sdk subdirectory in our project for the libraries
pico_sdk_init()

#getting Eigen library
find_package (Eigen3 3.3 REQUIRED NO_MODULE)

# Tell CMake where to find the executable source file
add_executable(${PROJECT_NAME} 
    main.cpp
    servos.cpp
    circle.cpp
    tiltCalcs.cpp
    helper.cpp
)

include_directories(tiltTable PUBLIC
                         "/Users/Sam/Code/tiltTable/include"
                    )

# Create map/bin/hex/uf2 files
pico_add_extra_outputs(${PROJECT_NAME})

# Link to pico_stdlib (gpio, time, etc. functions)
target_link_libraries(${PROJECT_NAME} 
    pico_stdlib
    hardware_pwm
    Eigen3::Eigen
)

# Enable usb output, disable uart output
pico_enable_stdio_usb(${PROJECT_NAME} 1)
pico_enable_stdio_uart(${PROJECT_NAME} 0)