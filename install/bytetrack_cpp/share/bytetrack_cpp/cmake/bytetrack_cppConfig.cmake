# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_bytetrack_cpp_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED bytetrack_cpp_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(bytetrack_cpp_FOUND FALSE)
  elseif(NOT bytetrack_cpp_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(bytetrack_cpp_FOUND FALSE)
  endif()
  return()
endif()
set(_bytetrack_cpp_CONFIG_INCLUDED TRUE)

# output package information
if(NOT bytetrack_cpp_FIND_QUIETLY)
  message(STATUS "Found bytetrack_cpp: 0.1.0 (${bytetrack_cpp_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'bytetrack_cpp' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${bytetrack_cpp_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(bytetrack_cpp_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "ament_cmake_export_targets-extras.cmake;ament_cmake_export_dependencies-extras.cmake")
foreach(_extra ${_extras})
  include("${bytetrack_cpp_DIR}/${_extra}")
endforeach()
