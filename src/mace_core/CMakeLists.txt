use_cxx11()

file(GLOB lib_srcs *.cpp)
file(GLOB lib_hdrs *.h)


add_library(mace_core ${lib_srcs} ${lib_hdrs})


#ADD_LIBRARY(exif_lib STATIC IMPORTED)
#SET_TARGET_PROPERTIES(exif_lib PROPERTIES
 #   IMPORTED_LOCATION /usr/local/lib/libexif.a)

#target_link_libraries(libjpeg exif_lib)

install(TARGETS mace_core
  # IMPORTANT: Add the libjpeg library to the "export-set"
  EXPORT MACETargets
  RUNTIME DESTINATION "${INSTALL_BIN_DIR}" COMPONENT bin
  LIBRARY DESTINATION "${INSTALL_LIB_DIR}" COMPONENT shlib
  ARCHIVE DESTINATION "${INSTALL_LIB_DIR}" COMPONENT shlib)
    
    
INSTALL(FILES ${lib_hdrs}
    DESTINATION "${INSTALL_INCLUDE_DIR}/mace_core"
    )
    
