cmake_minimum_required(VERSION 3.22)
project(sable_vox3d LANGUAGES C CXX)

set(CMAKE_C_STANDARD 17)
set(CMAKE_C_STANDARD_REQUIRED YES)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED YES)

find_package(Java REQUIRED)
find_package(JNI REQUIRED)

if (EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/vox3D/src")
    set(VOX3D_DIR "${CMAKE_CURRENT_SOURCE_DIR}/vox3D")
elseif (EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/vox3d/src")
    set(VOX3D_DIR "${CMAKE_CURRENT_SOURCE_DIR}/vox3d")
elseif (EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/include/box3d")
    set(VOX3D_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
elseif (EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../../vox3D/src")
    set(VOX3D_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../vox3D")
else()
    message(FATAL_ERROR "Could not find Vox3D source directory in ${CMAKE_CURRENT_SOURCE_DIR}/vox3D, ${CMAKE_CURRENT_SOURCE_DIR}/vox3d, or ${CMAKE_CURRENT_SOURCE_DIR}/../../vox3D")
endif()

message(STATUS "Using Vox3D source directory: ${VOX3D_DIR}")

file(GLOB VOX3D_SOURCES
    "${VOX3D_DIR}/src/*.c"
)

set(SABLE_VOX3D_SOURCES
    src/sable_vox3d.c
    ${VOX3D_SOURCES}
)

add_library(sable_vox3d SHARED ${SABLE_VOX3D_SOURCES})

target_include_directories(sable_vox3d PRIVATE
    ${JNI_INCLUDE_DIRS}
    "${VOX3D_DIR}/include"
    "${VOX3D_DIR}/src"
    src
)

if (MSVC)
    target_compile_options(sable_vox3d PRIVATE /W3 /O2 /Oi /Ot /fp:precise)
    target_compile_definitions(sable_vox3d PRIVATE _CRT_SECURE_NO_WARNINGS BOX3D_DOUBLE_PRECISION)
else()
    target_compile_options(sable_vox3d PRIVATE -O3 -fPIC)
    target_compile_definitions(sable_vox3d PRIVATE BOX3D_DOUBLE_PRECISION)
    target_link_libraries(sable_vox3d PRIVATE m)
endif()

set_target_properties(sable_vox3d PROPERTIES OUTPUT_NAME "sable_vox3d")

if (WIN32)
    set_target_properties(sable_vox3d PROPERTIES PREFIX "")
    set(SABLE_VOX3D_RESOURCE_OS "windows")
elseif (APPLE)
    set(SABLE_VOX3D_RESOURCE_OS "macos")
else()
    set(SABLE_VOX3D_RESOURCE_OS "linux")
endif()

string(TOLOWER "${CMAKE_SYSTEM_PROCESSOR}" SABLE_VOX3D_PROCESSOR)
if (SABLE_VOX3D_PROCESSOR MATCHES "^(amd64|x86_64|x64)$")
    set(SABLE_VOX3D_RESOURCE_ARCH "x86_64")
elseif (SABLE_VOX3D_PROCESSOR MATCHES "^(aarch64|arm64)$")
    set(SABLE_VOX3D_RESOURCE_ARCH "aarch64")
else()
    set(SABLE_VOX3D_RESOURCE_ARCH "${SABLE_VOX3D_PROCESSOR}")
endif()

set(SABLE_VOX3D_RESOURCE_DIR
    "${CMAKE_CURRENT_SOURCE_DIR}/../common/src/main/resources/natives/sable_vox3d/${SABLE_VOX3D_RESOURCE_OS}/${SABLE_VOX3D_RESOURCE_ARCH}")
if (WIN32 AND SABLE_VOX3D_RESOURCE_ARCH STREQUAL "x86_64")
    # Retain the original flat Windows resource for compatibility with older jars.
    add_custom_target(package_sable_vox3d_native ALL
        COMMAND ${CMAKE_COMMAND} -E make_directory "${SABLE_VOX3D_RESOURCE_DIR}"
        COMMAND ${CMAKE_COMMAND} -E copy_if_different "$<TARGET_FILE:sable_vox3d>"
            "${SABLE_VOX3D_RESOURCE_DIR}/$<TARGET_FILE_NAME:sable_vox3d>"
        COMMAND ${CMAKE_COMMAND} -E copy_if_different "$<TARGET_FILE:sable_vox3d>"
            "${CMAKE_CURRENT_SOURCE_DIR}/../common/src/main/resources/natives/sable_vox3d.dll"
        DEPENDS sable_vox3d
        COMMENT "Packaging the Vox3D native for ${SABLE_VOX3D_RESOURCE_OS}/${SABLE_VOX3D_RESOURCE_ARCH}"
    )
else()
    add_custom_target(package_sable_vox3d_native ALL
        COMMAND ${CMAKE_COMMAND} -E make_directory "${SABLE_VOX3D_RESOURCE_DIR}"
        COMMAND ${CMAKE_COMMAND} -E copy_if_different "$<TARGET_FILE:sable_vox3d>"
            "${SABLE_VOX3D_RESOURCE_DIR}/$<TARGET_FILE_NAME:sable_vox3d>"
        DEPENDS sable_vox3d
        COMMENT "Packaging the Vox3D native for ${SABLE_VOX3D_RESOURCE_OS}/${SABLE_VOX3D_RESOURCE_ARCH}"
    )
endif()

