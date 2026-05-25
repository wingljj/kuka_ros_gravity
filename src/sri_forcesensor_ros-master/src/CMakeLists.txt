cmake_minimum_required(VERSION 3.9)
project(src)

set(CMAKE_CXX_STANDARD 11)

add_executable(src
        sriforcesensor.cpp
        talker.cpp
        TCPClient.cpp
        TCPServer.cpp)
