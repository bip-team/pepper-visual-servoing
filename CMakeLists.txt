cmake_minimum_required(VERSION 2.8.3)

project(pepper-visual-servoing)

set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")

set(CMAKE_VERBOSE_MAKEFILE ON)

find_package(qibuild)
find_package(VISP REQUIRED)

include(pepper_visp OPTIONAL)

set(PEPPER_VISP_INTRINSIC_CAMERA_FILE "${CMAKE_SOURCE_DIR}/config/pepper_forehead_camera_kQVGA.xml" CACHE STRING "")
set(PEPPER_VISP_CONFIG_FILE           "${CMAKE_SOURCE_DIR}/config/pepper_vs_forehead_config.yaml"   CACHE STRING "")
set(PEPPER_VISP_DATA_PATH             "${CMAKE_SOURCE_DIR}/data/"                                   CACHE STRING "")
set(PEPPER_VISP_LOG_VELOCITY          ON                                                            CACHE BOOL   "")
set(PEPPER_VISP_USE_PEPPER_CONTROLLER OFF                                                           CACHE BOOL   "")

configure_file(${PROJECT_SOURCE_DIR}/cmake/vp_pepper_visp_config.h.cmake.in
               ${PROJECT_SOURCE_DIR}/pepper-visp/vp_pepper_visp_config.h
              )

###########
## yaml-cpp
###########

add_custom_target(TGT_yaml-cpp_build_libraries)

# yaml-cpp 
set(YAML-CPP_DIR          "${PROJECT_SOURCE_DIR}/contrib/yaml-cpp")
set(CMAKE_CXX_FLAGS_DEBUG "")
set(CMAKE_BUILD_TYPE      Release)

add_subdirectory("${YAML-CPP_DIR}" EXCLUDE_FROM_ALL)
set_property(TARGET yaml-cpp APPEND_STRING PROPERTY COMPILE_FLAGS " -fPIC ")
add_dependencies(TGT_yaml-cpp_build_libraries yaml-cpp)
include_directories(SYSTEM "${YAML-CPP_DIR}/include/")

add_subdirectory(pepper-visp)
