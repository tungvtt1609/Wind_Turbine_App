cmake_minimum_required (VERSION 3.0)
# Creating the project
project(DTUWEC LANGUAGES Fortran)

# Set the source code directory

set(ROOT_DIR ${CMAKE_CURRENT_SOURCE_DIR})
set(MODSRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src/modules)
set(IPCSRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src/individual_pitch_controller)
set(DTUWEC_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src/dtu_we_controller)
set(BLADED_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src/dtu_we_controller_bladed)
set(FLAP_SRC_DIR   ${CMAKE_CURRENT_SOURCE_DIR}/src/flap_controller_individual_aep_u_f)
set(FLAPCYC_SRC_DIR   ${CMAKE_CURRENT_SOURCE_DIR}/src/flap_controller_cyclic)
set(BUILD_TYPE SHARED)
set(LIB static)


message("Configuring DTU WEC source files: " ${DTUWEC_SRC_DIR})
add_subdirectory(${DTUWEC_SRC_DIR})

message("Configuring Bladed Interface source files: " ${BLADED_SRC_DIR})
add_subdirectory(${BLADED_SRC_DIR})

message("Configuring Individual Pitch Controller source files: " ${IPCSRC_DIR})
add_subdirectory(${IPCSRC_DIR})

message("Configuring flap controller source files: " ${FLAP_SRC_DIR})
add_subdirectory(${FLAP_SRC_DIR})

message("Configuring cyclic flap controller source files: " ${FLAPCYC_SRC_DIR})
add_subdirectory(${FLAPCYC_SRC_DIR})