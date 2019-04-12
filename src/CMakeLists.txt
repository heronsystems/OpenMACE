cmake_minimum_required(VERSION 2.8)
project(MACE)

macro(use_cxx11)
  if (CMAKE_VERSION VERSION_LESS "3.1")
    if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
      set (CMAKE_CXX_FLAGS "--std=gnu++0x ${CMAKE_CXX_FLAGS}")
    endif ()
  else ()
    set (CMAKE_CXX_STANDARD 11)
  endif ()
endmacro(use_cxx11)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})
find_package(Eigen3 REQUIRED)
message("Using Eigen library at: " ${EIGEN3_INCLUDE_DIR})
include_directories(${EIGEN3_INCLUDE_DIR})

set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

set (CMAKE_C_FLAGS "-fPIC ${CMAKE_CXX_FLAGS}")
set (CMAKE_CXX_FLAGS "-fPIC ${CMAKE_CXX_FLAGS}")

set (CMAKE_MODULE_LINKER_FLAGS "-fPIC ${CMAKE_MODULE_LINKER_FLAGS}")
set (CMAKE_SHARED_LINKER_FLAGS "-fPIC ${CMAKE_SHARED_LINKER_FLAGS}")
set (CMAKE_STATIC_LINKER_FLAGS "-fPIC ${CMAKE_STATIC_LINKER_FLAGS}")

set(MACE_MAJOR_VERSION 0)
set(MACE_MINOR_VERSION 1)
set(MACE_PATCH_VERSION 1)
set(MACE_VERSION
  ${MACE_MAJOR_VERSION}.${MACE_MINOR_VERSION}.${MACE_PATCH_VERSION})
 
# Offer the user the choice of overriding the installation directories
set(INSTALL_LIB_DIR lib CACHE PATH "Installation directory for libraries")
set(INSTALL_BIN_DIR bin CACHE PATH "Installation directory for executables")
set(INSTALL_INCLUDE_DIR include CACHE PATH
  "Installation directory for header files")
if(WIN32 AND NOT CYGWIN)
  set(DEF_INSTALL_CMAKE_DIR CMake)
else()
  set(DEF_INSTALL_CMAKE_DIR lib/CMake/MACE)
endif()
set(INSTALL_CMAKE_DIR ${DEF_INSTALL_CMAKE_DIR} CACHE PATH
  "Installation directory for CMake files")
 
# Make relative paths absolute (needed later on)
foreach(p LIB BIN INCLUDE CMAKE)
  set(var INSTALL_${p}_DIR)
  if(NOT IS_ABSOLUTE "${${var}}")
    set(${var} "${CMAKE_INSTALL_PREFIX}/${${var}}")
  endif()
endforeach()
 
# set up include-directories
include_directories(
  "${PROJECT_SOURCE_DIR}"   # to find foo/foo.h
  "${PROJECT_BINARY_DIR}")  # to find foo/config.h
 
# Add sub-directories
add_subdirectory(mace_core)
add_subdirectory(comms)
add_subdirectory(module_path_planning_NASAPhase2)
add_subdirectory(module_RTA_NASAPhase2)
add_subdirectory(module_vehicle_MAVLINK)
add_subdirectory(mace)
 
# The interesting stuff goes here
# ===============================
 
# Add all targets to the build-tree export set
export(TARGETS mace_core comms module_path_planning_NASAPhase2 module_RTA_NASAPhase2 module_vehicle_MAVLINK MACE
  FILE "${PROJECT_BINARY_DIR}/MACETargets.cmake")
 
# Export the package for use from the build-tree
# (this registers the build-tree with a global CMake-registry)
export(PACKAGE MACE)
 
# Create the MACEConfig.cmake and MACEConfigVersion files
file(RELATIVE_PATH REL_INCLUDE_DIR "${INSTALL_CMAKE_DIR}"
   "${INSTALL_INCLUDE_DIR}")
# ... for the build tree
set(CONF_INCLUDE_DIRS "${PROJECT_SOURCE_DIR}" "${PROJECT_BINARY_DIR}")
configure_file(MACEConfig.cmake.in
  "${PROJECT_BINARY_DIR}/MACEConfig.cmake" @ONLY)
# ... for the install tree
set(CONF_INCLUDE_DIRS "\${MACE_CMAKE_DIR}/${REL_INCLUDE_DIR}")
configure_file(MACEConfig.cmake.in
  "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/MACEConfig.cmake" @ONLY)
# ... for both
configure_file(MACEConfigVersion.cmake.in
  "${PROJECT_BINARY_DIR}/MACEConfigVersion.cmake" @ONLY)
 
# Install the MACEConfig.cmake and MACEConfigVersion.cmake
install(FILES
  "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/MACEConfig.cmake"
  "${PROJECT_BINARY_DIR}/MACEConfigVersion.cmake"
  DESTINATION "${INSTALL_CMAKE_DIR}" COMPONENT dev)
 
# Install the export set for use with the install-tree
install(EXPORT MACETargets DESTINATION
  "${INSTALL_CMAKE_DIR}" COMPONENT dev)
