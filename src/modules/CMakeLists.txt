# Set the library name
project(modules LANGUAGES Fortran)

set(MODSRC_DIR ${CMAKE_CURRENT_SOURCE_DIR})

# Add a shared library
add_library(common_module STATIC ${MODSRC_DIR}/global_constants.f90
                                            global_variables.f90
											misc_mod.f90
											user_defined_types.f90)
