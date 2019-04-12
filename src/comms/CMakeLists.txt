use_cxx11()

file(GLOB lib_srcs *.cpp)
file(GLOB lib_hdrs *.h)

find_package(Qt5SerialPort)

include_directories(${PROJECT_SOURCE_DIR}/../mavlink_cpp/V2/ardupilotmega)

add_library(comms ${lib_srcs} ${lib_hdrs})
 
target_link_libraries(comms Qt5::SerialPort)
 
install(TARGETS comms
  EXPORT MACETargets
  RUNTIME DESTINATION "${INSTALL_BIN_DIR}" COMPONENT bin
  LIBRARY DESTINATION "${INSTALL_LIB_DIR}" COMPONENT shlib
  ARCHIVE DESTINATION "${INSTALL_LIB_DIR}" COMPONENT shlib)
    
    
INSTALL(FILES ${lib_hdrs}
    DESTINATION "${INSTALL_INCLUDE_DIR}/comms"
    )
    
