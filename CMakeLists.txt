cmake_minimum_required(VERSION 3.27)
project(SwitchProControllerLibrary)

set(CMAKE_CXX_STANDARD 23)
include_directories(/opt/homebrew/include)
link_directories(/opt/homebrew/lib)
include_directories(.)

add_executable(SwitchProControllerLibrary
        src/main.cpp
        src/lib/SerialPort.h
        src/lib/SwitchReport.h
        src/lib/SwitchControlLibrary.cpp
        src/lib/SwitchControlLibrary.h)

target_link_libraries(SwitchProControllerLibrary PRIVATE
        serialport
)
