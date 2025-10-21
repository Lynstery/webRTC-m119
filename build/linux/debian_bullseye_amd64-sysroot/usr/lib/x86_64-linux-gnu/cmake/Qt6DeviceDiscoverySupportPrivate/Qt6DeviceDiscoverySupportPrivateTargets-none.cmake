#----------------------------------------------------------------
# Generated CMake target import file for configuration "None".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "Qt6::DeviceDiscoverySupportPrivate" for configuration "None"
set_property(TARGET Qt6::DeviceDiscoverySupportPrivate APPEND PROPERTY IMPORTED_CONFIGURATIONS NONE)
set_target_properties(Qt6::DeviceDiscoverySupportPrivate PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_NONE "CXX"
  IMPORTED_LOCATION_NONE "${_IMPORT_PREFIX}/lib/x86_64-linux-gnu/libQt6DeviceDiscoverySupport.a"
  )

list(APPEND _cmake_import_check_targets Qt6::DeviceDiscoverySupportPrivate )
list(APPEND _cmake_import_check_files_for_Qt6::DeviceDiscoverySupportPrivate "${_IMPORT_PREFIX}/lib/x86_64-linux-gnu/libQt6DeviceDiscoverySupport.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
