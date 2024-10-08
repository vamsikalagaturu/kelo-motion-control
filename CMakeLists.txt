cmake_minimum_required(VERSION 3.15)
project(kelo_motion_control VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)

set(CMAKE_INSTALL_DIR share/${PROJECT_NAME}/cmake)

set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

list(APPEND CMAKE_PREFIX_PATH ${CMAKE_BINARY_DIR}/../build/)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_BINARY_DIR}/../build/)

LINK_DIRECTORIES(${CMAKE_INSTALL_PREFIX}/lib/)

find_package(soem REQUIRED)

list(APPEND local_INCLUDE_DIRS ${CMAKE_INSTALL_PREFIX}/include/)

include_directories(
  include
  ${local_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED
	src/EthercatCommunication.c
  src/KeloMotionControl.c
	src/KELORobotKinematics.c
	src/PlatformToWheelSolver.c
	src/SmartWheelKinematics.c
  src/mediator.c
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}
	soem
	pthread
	gsl
	gslcblas
	m	
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}-targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Generate the CMake config file
include(CMakePackageConfigHelpers)
# Generate the version file accompanying the project's config file
configure_package_config_file(${PROJECT_NAME}-config.cmake.in
                              ${PROJECT_NAME}-config.cmake
                              INSTALL_DESTINATION ${CMAKE_INSTALL_DIR})

write_basic_package_version_file(${PROJECT_NAME}-config-version.cmake
                                 VERSION ${PROJECT_VERSION}
                                 COMPATIBILITY AnyNewerVersion)

# Install the config and configversion
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config.cmake
              ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config-version.cmake
        DESTINATION ${CMAKE_INSTALL_DIR})

# Make the targets accessible from this packages's build tree
export(
  EXPORT ${PROJECT_NAME}-targets
  NAMESPACE ${PROJECT_NAME}::
  FILE "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-targets.cmake"
)

# Make the targets accessible from this packages's install tree
install(
  EXPORT ${PROJECT_NAME}-targets
  FILE ${PROJECT_NAME}-targets.cmake
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION ${CMAKE_INSTALL_DIR})

install(DIRECTORY include/
  DESTINATION include
  FILES_MATCHING PATTERN "*.h"
)

# add all cpp files in the test directory as test sources
file(GLOB test_sources test/*.c test/*.cpp)

# make executables
foreach(source ${test_sources})
  message(STATUS "Adding test: ${source}")
  get_filename_component(name ${source} NAME_WE)
  add_executable(${name} ${source})
  target_link_libraries(${name} ${PROJECT_NAME} soem pthread gsl gslcblas m)

  set_target_properties(${name} PROPERTIES INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
  set_target_properties(${name} PROPERTIES BUILD_WITH_INSTALL_RPATH TRUE)

  install(TARGETS ${name} RUNTIME DESTINATION bin)
endforeach(source ${test_sources})