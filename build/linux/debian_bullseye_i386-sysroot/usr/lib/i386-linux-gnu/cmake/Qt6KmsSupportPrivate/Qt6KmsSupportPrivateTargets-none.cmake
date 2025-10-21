#----------------------------------------------------------------
# Generated CMake target import file for configuration "None".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "Qt6::KmsSupportPrivate" for configuration "None"
set_property(TARGET Qt6::KmsSupportPrivate APPEND PROPERTY IMPORTED_CONFIGURATIONS NONE)
set_target_properties(Qt6::KmsSupportPrivate PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_NONE "CXX"
  IMPORTED_LOCATION_NONE "${_IMPORT_PREFIX}/lib/i386-linux-gnu/libQt6KmsSupport.a"
  )

list(APPEND _cmake_import_check_targets Qt6::KmsSupportPrivate )
list(APPEND _cmake_import_check_files_for_Qt6::KmsSupportPrivate "${_IMPORT_PREFIX}/lib/i386-linux-gnu/libQt6KmsSupport.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
