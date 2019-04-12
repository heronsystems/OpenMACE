use_cxx11()

file(GLOB lib_srcs *.cpp)
file(GLOB lib_hdrs *.h)
file(GLOB lib_hppdrs *.hpp)

find_package(Qt5SerialPort)

add_executable(MACE ${lib_srcs} ${lib_hdrs} ${lib_hppdrs})

find_package (Threads)

include_directories(${PROJECT_SOURCE_DIR}/../mavlink_cpp/V2/ardupilotmega)

target_link_libraries (MACE ${CMAKE_THREAD_LIBS_INIT})

target_link_libraries(MACE mace_core)
target_link_libraries(MACE comms)
target_link_libraries(MACE module_path_planning_NASAPhase2)
target_link_libraries(MACE module_RTA_NASAPhase2)
target_link_libraries(MACE module_vehicle_MAVLINK)
target_link_libraries(MACE Qt5::SerialPort)

#ADD_LIBRARY(exif_lib STATIC IMPORTED)
#SET_TARGET_PROPERTIES(exif_lib PROPERTIES
 #   IMPORTED_LOCATION /usr/local/lib/libexif.a)

#target_link_libraries(libjpeg exif_lib)

install(TARGETS MACE
  # IMPORTANT: Add the libjpeg library to the "export-set"
  EXPORT MACETargets
  RUNTIME DESTINATION "${INSTALL_BIN_DIR}" COMPONENT bin
  LIBRARY DESTINATION "${INSTALL_LIB_DIR}" COMPONENT shlib
  ARCHIVE DESTINATION "${INSTALL_LIB_DIR}" COMPONENT shlib)
    
    
#INSTALL(FILES ${lib_hdrs}
#    DESTINATION "${INSTALL_INCLUDE_DIR}/mace_core"
    #)
