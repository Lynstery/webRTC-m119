#----------------------------------------------------------------
# Generated CMake target import file for configuration "None".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "Qt6::uic" for configuration "None"
set_property(TARGET Qt6::uic APPEND PROPERTY IMPORTED_CONFIGURATIONS NONE)
set_target_properties(Qt6::uic PROPERTIES
  IMPORTED_LOCATION_NONE "${_IMPORT_PREFIX}/lib/qt6/libexec/uic"
  )

list(APPEND _cmake_import_check_targets Qt6::uic )
list(APPEND _cmake_import_check_files_for_Qt6::uic "${_IMPORT_PREFIX}/lib/qt6/libexec/uic" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
