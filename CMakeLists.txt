cmake_minimum_required(VERSION 3.27)
project(SwitchProControllerLibrary)

set(CMAKE_CXX_STANDARD 14)

include_directories(.)

add_executable(SwitchProControllerLibrary
        src/main.cpp
        src/lib/SerialPort.h
        src/lib/SwitchReport.h
        src/lib/SwitchControlLibrary.cpp
        src/lib/SwitchControlLibrary.h)
