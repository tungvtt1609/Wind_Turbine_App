# Set the project name
project(dtu_we_controller LANGUAGES Fortran)

message("Configuring Subproject: " ${PROJECT_NAME})
message("Root Directory is: " ${ROOT_DIR})

# set source code
set(SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR})
set(DTUWEC_SRC_DIR ${SRC_DIR})
file(GLOB_RECURSE MODSRC "${ROOT_DIR}/src/modules/global_constants.f90"
                         "${ROOT_DIR}/src/modules/global_variables.f90"
					     "${ROOT_DIR}/src/modules/misc_mod.f90"
					     "${ROOT_DIR}/src/modules/user_defined_types.f90")


file(GLOB_RECURSE SRC "${SRC_DIR}/dtu_we_controller.f90"
                      "${SRC_DIR}/dtu_we_controller_fcns.f90"
					  "${SRC_DIR}/turbine_controller.f90"
					  "${SRC_DIR}/safety_system.f90"
					  "${SRC_DIR}/floating_controller.f90"
    )
list (APPEND SRC ${MODSRC})

message("Including the utils: " "${ROOT_DIR}/utils/cmake/CMakeLists.txt") 
include(${ROOT_DIR}/utils/cmake/CMakeLists.txt)

