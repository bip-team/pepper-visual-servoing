cmake_minimum_required(VERSION 2.8.3)

project(pepper-vs)

set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")

find_package(qibuild)
find_package(VISP REQUIRED)

include(pepper_visp OPTIONAL)

set(PEPPER_VISP_INTRINSIC_CAMERA_FILE "${CMAKE_SOURCE_DIR}/config/camera_kQVGA.xml" CACHE STRING "")
set(PEPPER_VISP_LOG_FILE_PATH         "${CMAKE_SOURCE_DIR}/data/"                   CACHE STRING "")
set(PEPPER_VISP_LOG_VELOCITY          ON                                            CACHE BOOL   "")
set(PEPPER_VISP_USE_PEPPER_CONTROLLER OFF                                           CACHE BOOL   "")

configure_file(${PROJECT_SOURCE_DIR}/cmake/vp_pepper_visp_config.h.cmake.in
               ${PROJECT_SOURCE_DIR}/pepper-visp/vp_pepper_visp_config.h
              )

add_subdirectory(pepper-visp)
