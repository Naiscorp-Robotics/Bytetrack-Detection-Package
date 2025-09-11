#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "bytetrack_cpp::bytetrack_cpp" for configuration ""
set_property(TARGET bytetrack_cpp::bytetrack_cpp APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(bytetrack_cpp::bytetrack_cpp PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libbytetrack_cpp.so"
  IMPORTED_SONAME_NOCONFIG "libbytetrack_cpp.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS bytetrack_cpp::bytetrack_cpp )
list(APPEND _IMPORT_CHECK_FILES_FOR_bytetrack_cpp::bytetrack_cpp "${_IMPORT_PREFIX}/lib/libbytetrack_cpp.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
