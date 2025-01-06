cmake_minimum_required(VERSION 3.18)
project(rasteriser)

if(NOT CMAKE_BUILD_TYPE) #Default to release if the user passes nothing.
        set(CMAKE_BUILD_TYPE "Release")
endif()

set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT DemoBitBlt)

add_executable(rasteriser WIN32
    windows.cpp
    dds-ktx.h  tiny_gltf.h
    math.h
    log.h
    load_gltf.cpp
    load_gltf.h
    draw_scene.cpp
    draw_scene.h
    raster.cpp
    raster.h
    bake.cpp
    bake.h
)

target_compile_definitions(rasteriser PRIVATE $<$<CONFIG:Debug>:DEBUG=1> $<$<NOT:$<CONFIG:Debug>>:RELEASE=1> ) #Defines DEBUG=1 or RELEASE=1

