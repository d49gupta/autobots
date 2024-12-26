# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_autobots_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED autobots_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(autobots_FOUND FALSE)
  elseif(NOT autobots_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(autobots_FOUND FALSE)
  endif()
  return()
endif()
set(_autobots_CONFIG_INCLUDED TRUE)

# output package information
if(NOT autobots_FIND_QUIETLY)
  message(STATUS "Found autobots: 0.0.0 (${autobots_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'autobots' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${autobots_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(autobots_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${autobots_DIR}/${_extra}")
endforeach()
