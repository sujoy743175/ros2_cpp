# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_avoid_obstacle_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED avoid_obstacle_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(avoid_obstacle_FOUND FALSE)
  elseif(NOT avoid_obstacle_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(avoid_obstacle_FOUND FALSE)
  endif()
  return()
endif()
set(_avoid_obstacle_CONFIG_INCLUDED TRUE)

# output package information
if(NOT avoid_obstacle_FIND_QUIETLY)
  message(STATUS "Found avoid_obstacle: 0.0.0 (${avoid_obstacle_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'avoid_obstacle' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${avoid_obstacle_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(avoid_obstacle_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${avoid_obstacle_DIR}/${_extra}")
endforeach()
