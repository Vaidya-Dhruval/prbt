cmake_minimum_required(VERSION 3.5)
project(pilz_utils)

# Compile as C++11
add_compile_options(-std=c++11 -Wall -Wextra -Wno-unused-parameter -Werror)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

###################################
## ament specific configuration ##
###################################
ament_package()

###########
## Build ##
###########

# Specify additional locations of header files
include_directories(
  include
)

#############
## Install ##
#############

# Install header files
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

#############
## Linting ##
#############
if(AMENT_ENABLE_CLANG_TIDY)
  find_program(
    CLANG_TIDY_EXE
    NAMES "clang-tidy"
    DOC "Path to clang-tidy executable"
  )
  if(NOT CLANG_TIDY_EXE)
    message(FATAL_ERROR "clang-tidy not found.")
  else()
    message(STATUS "clang-tidy found: ${CLANG_TIDY_EXE}")
    set(CMAKE_CXX_CLANG_TIDY "${CLANG_TIDY_EXE}")
  endif()
endif()
