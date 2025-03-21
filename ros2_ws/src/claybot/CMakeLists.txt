cmake_minimum_required(VERSION 3.8)
project(claybot)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(claybot_interfaces REQUIRED)

# Add Doxygen support
option(BUILD_DOCUMENTATION "Build API documentation with Doxygen" ON)
if(BUILD_DOCUMENTATION)
  find_package(Doxygen)
  if(DOXYGEN_FOUND)
    # Configure Doxygen input and output files
    set(DOXYGEN_IN ${CMAKE_CURRENT_SOURCE_DIR}/docs/Doxyfile.in)
    set(DOXYGEN_OUT ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)

    # Create documentation directory if it doesn't exist
    file(MAKE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/docs)

    # Check if Doxyfile.in exists, if not, create it
    if(NOT EXISTS ${DOXYGEN_IN})
      # Create a basic Doxyfile.in with correct settings
      file(WRITE ${DOXYGEN_IN} "PROJECT_NAME           = \"${PROJECT_NAME}\"\n")
      file(APPEND ${DOXYGEN_IN} "PROJECT_NUMBER         = \"${PROJECT_VERSION}\"\n")
      file(APPEND ${DOXYGEN_IN} "PROJECT_BRIEF          = \"Package for controlling Claybot\"\n")
      file(APPEND ${DOXYGEN_IN} "OUTPUT_DIRECTORY       = \"@CMAKE_CURRENT_BINARY_DIR@\"\n")
      file(APPEND ${DOXYGEN_IN} "INPUT                  = \"@CMAKE_CURRENT_SOURCE_DIR@/include\" \\\n")
      file(APPEND ${DOXYGEN_IN} "                         \"@CMAKE_CURRENT_SOURCE_DIR@/src\"\n")
      file(APPEND ${DOXYGEN_IN} "FILE_PATTERNS          = *.h \\\n")
      file(APPEND ${DOXYGEN_IN} "                         *.hpp \\\n")
      file(APPEND ${DOXYGEN_IN} "                         *.cpp \\\n")
      file(APPEND ${DOXYGEN_IN} "                         *.c \\\n")
      file(APPEND ${DOXYGEN_IN} "                         *.cc \\\n")
      file(APPEND ${DOXYGEN_IN} "                         *.py\n")
      file(APPEND ${DOXYGEN_IN} "RECURSIVE              = YES\n")
      file(APPEND ${DOXYGEN_IN} "GENERATE_HTML          = YES\n")
      file(APPEND ${DOXYGEN_IN} "HTML_OUTPUT            = html\n")
      file(APPEND ${DOXYGEN_IN} "GENERATE_LATEX         = NO\n")
      file(APPEND ${DOXYGEN_IN} "HAVE_DOT               = YES\n")
      file(APPEND ${DOXYGEN_IN} "EXTRACT_ALL            = YES\n")
      file(APPEND ${DOXYGEN_IN} "EXTRACT_PRIVATE        = YES\n")
      file(APPEND ${DOXYGEN_IN} "EXTRACT_PACKAGE        = YES\n")
      file(APPEND ${DOXYGEN_IN} "EXTRACT_STATIC         = YES\n")
      file(APPEND ${DOXYGEN_IN} "EXTRACT_LOCAL_CLASSES  = YES\n")
      file(APPEND ${DOXYGEN_IN} "SOURCE_BROWSER         = YES\n")
    endif()

    # Configure the Doxyfile 
    configure_file(${DOXYGEN_IN} ${DOXYGEN_OUT} @ONLY)

    # Add a custom target to run Doxygen
    add_custom_target(docs
      COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUT}
      WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
      COMMENT "Generating API documentation with Doxygen"
      VERBATIM
    )

    # Create an html directory if it doesn't exist to prevent installation errors
    add_custom_command(
      TARGET docs
      POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_CURRENT_BINARY_DIR}/html
      COMMENT "Ensuring html directory exists"
    )
    
    # Install the documentation
    # With this:
    install(CODE "
    if(EXISTS \"${CMAKE_CURRENT_BINARY_DIR}/html\")
      message(STATUS \"Installing: \${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/docs/html\")
      file(COPY \"${CMAKE_CURRENT_BINARY_DIR}/html\" DESTINATION \"\${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/docs\")
    else()
      message(STATUS \"Documentation not found: ${CMAKE_CURRENT_BINARY_DIR}/html\")
      
      # Attempt to run Doxygen again to generate docs
      execute_process(
        COMMAND ${DOXYGEN_EXECUTABLE} \"${DOXYGEN_OUT}\"
        WORKING_DIRECTORY \"${CMAKE_CURRENT_BINARY_DIR}\"
        RESULT_VARIABLE DOXYGEN_RESULT
      )
      
      if(EXISTS \"${CMAKE_CURRENT_BINARY_DIR}/html\")
        message(STATUS \"Documentation generated during install: Installing to \${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/docs/html\")
        file(COPY \"${CMAKE_CURRENT_BINARY_DIR}/html\" DESTINATION \"\${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/docs\")
      else()
        message(STATUS \"Documentation still not found after generation attempt. Creating minimal placeholder.\")
        file(MAKE_DIRECTORY \"\${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/docs/html\")
        file(WRITE \"\${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/docs/html/index.html\" \"<html><head><title>${PROJECT_NAME} Documentation</title></head><body><h1>${PROJECT_NAME} Documentation</h1><p>Documentation generation is in progress. Please build the 'docs' target manually.</p></body></html>\")
      endif()
    endif()
    ")

  else(DOXYGEN_FOUND)
    message(STATUS "Doxygen not found, documentation will not be built")
  endif(DOXYGEN_FOUND)
endif(BUILD_DOCUMENTATION)

include_directories(
  include
)

add_executable(control_turn src/control_turn.cpp)
ament_target_dependencies(control_turn
  claybot_interfaces
  dynamixel_sdk
  rclcpp
)

# Install executables
install(TARGETS 
  control_turn
  DESTINATION lib/${PROJECT_NAME})
  
# Install launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Python scripts
install(
  PROGRAMS 
  scripts/cut_shape_srv.py
  scripts/move_updown_srv.py
  scripts/move_updown_cli.py
  scripts/cut_shape_srv.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
