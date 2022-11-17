# Set the project name
project(flap_controller_cyclic LANGUAGES Fortran)

message("Configuring Subproject: " ${PROJECT_NAME})
message("Root Directory is: " ${ROOT_DIR})

# set source code
set(FLAPCYC_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR})
set(SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR})

file(GLOB_RECURSE MODSRC "${ROOT_DIR}/src/modules/global_constants.f90"
                         "${ROOT_DIR}/src/modules/global_variables.f90"
					     "${ROOT_DIR}/src/modules/misc_mod.f90"
					     "${ROOT_DIR}/src/modules/user_defined_types.f90"
	)

set(SRC ${FLAPCYC_SRC_DIR}/flap_controller_cyclic.f90 
                      flap_controller_cyclic_fcns.f90					  
	)	
list(APPEND SRC ${MODSRC})

message("Including the utils: " "${ROOT_DIR}/utils/cmake/CMakeLists.txt")
include(${ROOT_DIR}/utils/cmake/CMakeLists.txt)



