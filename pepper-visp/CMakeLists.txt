file(GLOB SOURCES
     "*.cpp")

file(GLOB INCLUDES
     "*.h")

foreach(SOURCE ${SOURCES})
    get_filename_component(tgt ${SOURCE} NAME_WE)
    qi_create_bin(${tgt} ${SOURCE} ${INCLUDES})
    qi_use_lib(${tgt} VISP ALCOMMON ALPROXIES ALVISION BOOST EIGEN3)
endforeach()
