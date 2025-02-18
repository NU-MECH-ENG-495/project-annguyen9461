# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_claybot_srvcli_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED claybot_srvcli_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(claybot_srvcli_FOUND FALSE)
  elseif(NOT claybot_srvcli_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(claybot_srvcli_FOUND FALSE)
  endif()
  return()
endif()
set(_claybot_srvcli_CONFIG_INCLUDED TRUE)

# output package information
if(NOT claybot_srvcli_FIND_QUIETLY)
  message(STATUS "Found claybot_srvcli: 0.0.0 (${claybot_srvcli_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'claybot_srvcli' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT claybot_srvcli_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(claybot_srvcli_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${claybot_srvcli_DIR}/${_extra}")
endforeach()
