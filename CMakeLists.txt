cmake_minimum_required(VERSION 3.17)
project(my_half_turtle_manager)

set(CMAKE_CXX_STANDARD 20)

add_executable(my_half_turtle_manager main.cpp

        bipropellant-api/HoverboardAPI.h
        bipropellant-api/HoverboardAPI.cpp
        bipropellant-api/protocolFunctions.h
        bipropellant-api/protocolFunctions.c

        bipropellant-api/hbprotocol/protocol.h
        bipropellant-api/hbprotocol/protocol_private.h

        bipropellant-api/hbprotocol/protocol.c
        bipropellant-api/hbprotocol/machine_protocol.c
#        bipropellant-api/hbprotocol/cobsr.c
#        bipropellant-api/hbprotocol/cobsr.h
        bipropellant-api/hbprotocol/ascii_protocol.h
        bipropellant-api/hbprotocol/ascii_protocol.c
        )