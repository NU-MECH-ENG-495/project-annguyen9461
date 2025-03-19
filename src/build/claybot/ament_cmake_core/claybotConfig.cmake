# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_claybot_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED claybot_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(claybot_FOUND FALSE)
  elseif(NOT claybot_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(claybot_FOUND FALSE)
  endif()
  return()
endif()
set(_claybot_CONFIG_INCLUDED TRUE)

# output package information
if(NOT claybot_FIND_QUIETLY)
  message(STATUS "Found claybot: 0.0.0 (${claybot_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'claybot' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT claybot_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(claybot_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${claybot_DIR}/${_extra}")
endforeach()
