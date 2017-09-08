cmake_minimum_required(VERSION 2.8.3)

project(pepper-visual-servoing)

set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")

set(CMAKE_VERBOSE_MAKEFILE ON)

find_package(qibuild)
find_package(VISP REQUIRED)

include(pepper_visp OPTIONAL)

set(PEPPER_VISP_CONFIG_PATH           "${CMAKE_SOURCE_DIR}/config/"                               CACHE STRING "")
set(PEPPER_VISP_INTRINSIC_CAMERA_FILE "${CMAKE_SOURCE_DIR}/config/pepper_cameras.xml"             CACHE STRING "")
set(PEPPER_VISP_FOREHEAD_CONFIG_FILE  "${CMAKE_SOURCE_DIR}/config/pepper_vs_forehead_config.yaml" CACHE STRING "")
set(PEPPER_VISP_CHIN_CONFIG_FILE      "${CMAKE_SOURCE_DIR}/config/pepper_vs_chin_config.yaml"     CACHE STRING "")
set(PEPPER_VISP_DATA_PATH             "${CMAKE_SOURCE_DIR}/data/"                                 CACHE STRING "")
set(PEPPER_VISP_LOG_VELOCITY          ON                                                          CACHE BOOL   "")
set(PEPPER_VISP_USE_PEPPER_CONTROLLER OFF                                                         CACHE BOOL   "")

configure_file(${PROJECT_SOURCE_DIR}/cmake/vp_pepper_visp_config.h.cmake.in
               ${PROJECT_SOURCE_DIR}/include/pepper-visp/vp_pepper_visp_config.h
              )

##############
## config-yaml
##############

add_custom_target(TGT_build_libraries)

# config-yaml 
set(CONFIG-YAML_DIR       "${PROJECT_SOURCE_DIR}/contrib/config-yaml")
set(CMAKE_CXX_FLAGS_DEBUG "")
set(CMAKE_BUILD_TYPE      Release)

add_subdirectory("${CONFIG-YAML_DIR}" EXCLUDE_FROM_ALL)
set_property(TARGET yaml-cpp APPEND_STRING PROPERTY COMPILE_FLAGS " -fPIC ")
add_dependencies(TGT_build_libraries TGT_config-yaml_build_libraries)

include_directories(SYSTEM "${CONFIG-YAML_DIR}/contrib/yaml-cpp/include/")
include_directories(SYSTEM "${CONFIG-YAML_DIR}/include/")
include_directories(SYSTEM "${PROJECT_SOURCE_DIR}/include/")

add_subdirectory(src)
