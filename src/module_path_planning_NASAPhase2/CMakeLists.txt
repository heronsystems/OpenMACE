use_cxx11()

file(GLOB lib_srcs *.cpp)
file(GLOB lib_hdrs *.h)

add_library(module_path_planning_NASAPhase2 ${lib_srcs} ${lib_hdrs})
 
target_link_libraries(module_path_planning_NASAPhase2 mace_core)
 
install(TARGETS module_path_planning_NASAPhase2
  # IMPORTANT: Add the libjpeg library to the "export-set"
  EXPORT MACETargets
  RUNTIME DESTINATION "${INSTALL_BIN_DIR}" COMPONENT bin
  LIBRARY DESTINATION "${INSTALL_LIB_DIR}" COMPONENT shlib
  ARCHIVE DESTINATION "${INSTALL_LIB_DIR}" COMPONENT shlib)
    
    
INSTALL(FILES ${lib_hdrs}
    DESTINATION "${INSTALL_INCLUDE_DIR}/module_path_planning_NASAPhase2"
    )
    
