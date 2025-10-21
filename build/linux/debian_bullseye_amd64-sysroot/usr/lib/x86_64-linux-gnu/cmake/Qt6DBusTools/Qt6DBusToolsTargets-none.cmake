#----------------------------------------------------------------
# Generated CMake target import file for configuration "None".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "Qt6::qdbuscpp2xml" for configuration "None"
set_property(TARGET Qt6::qdbuscpp2xml APPEND PROPERTY IMPORTED_CONFIGURATIONS NONE)
set_target_properties(Qt6::qdbuscpp2xml PROPERTIES
  IMPORTED_LOCATION_NONE "${_IMPORT_PREFIX}/lib/qt6/bin/qdbuscpp2xml"
  )

list(APPEND _cmake_import_check_targets Qt6::qdbuscpp2xml )
list(APPEND _cmake_import_check_files_for_Qt6::qdbuscpp2xml "${_IMPORT_PREFIX}/lib/qt6/bin/qdbuscpp2xml" )

# Import target "Qt6::qdbusxml2cpp" for configuration "None"
set_property(TARGET Qt6::qdbusxml2cpp APPEND PROPERTY IMPORTED_CONFIGURATIONS NONE)
set_target_properties(Qt6::qdbusxml2cpp PROPERTIES
  IMPORTED_LOCATION_NONE "${_IMPORT_PREFIX}/lib/qt6/bin/qdbusxml2cpp"
  )

list(APPEND _cmake_import_check_targets Qt6::qdbusxml2cpp )
list(APPEND _cmake_import_check_files_for_Qt6::qdbusxml2cpp "${_IMPORT_PREFIX}/lib/qt6/bin/qdbusxml2cpp" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
