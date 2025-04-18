cmake_minimum_required(VERSION 3.5)

project(
  "romea_core_path_following"
  VERSION 0.1.0
  #  DESCRIPTION
  #  HOMEPAGE_URL
  LANGUAGES CXX)

find_package(romea_core_common REQUIRED)
find_package(romea_core_control REQUIRED)
find_package(romea_core_mobile_base REQUIRED)
find_package(romea_core_path REQUIRED)

find_package(GSL REQUIRED)
find_package(BLAS REQUIRED)
find_package(nlohmann_json 3.7 REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/logs.cpp
  src/fsm.cpp
  src/setpoint.cpp
  src/utils.cpp
  src/external_control/platoon.cpp
  src/lateral_control/back_stepping.cpp
  src/lateral_control/classic.cpp
  src/lateral_control/predictive.cpp
  src/lateral_control/front_rear_decoupled.cpp
  src/lateral_control/skid_backstepping.cpp
  src/longitudinal_control/classic.cpp
  src/sliding_observer/extended/cinematic_lyapunov.cpp
  src/sliding_observer/extended/cinematic_linear_tangent.cpp
  src/sliding_observer/skid/picard_backstepping.cpp
  src/sliding_observer/skid/picard_lyapunov.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
  ${romea_core_common_INCLUDE_DIRS}
  ${romea_core_control_INCLUDE_DIRS}
  ${romea_core_mobile_base_INCLUDE_DIRS}
  ${romea_core_path_INCLUDE_DIRS})

target_compile_options(${PROJECT_NAME} PRIVATE
  -Wall -Wextra -O3 -std=c++17)

target_link_libraries(${PROJECT_NAME} PUBLIC
  romea_core_common::romea_core_common
  romea_core_control::romea_core_control
  romea_core_mobile_base::romea_core_mobile_base
  romea_core_path::romea_core_path)

target_link_libraries(${PROJECT_NAME} PRIVATE
  GSL::gsl ${BLAS_LIBRARIES} nlohmann_json::nlohmann_json)

include(GNUInstallDirs)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

install(FILES package.xml DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME})

include(CMakePackageConfigHelpers)
write_basic_package_version_file(
  "${PROJECT_NAME}ConfigVersion.cmake"
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion)

configure_package_config_file(
  "${PROJECT_SOURCE_DIR}/cmake/${PROJECT_NAME}Config.cmake.in"
  "${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake" INSTALL_DESTINATION
  ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME}/cmake)

install(
  EXPORT ${PROJECT_NAME}Targets
  FILE ${PROJECT_NAME}Targets.cmake
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME}/cmake)

install(FILES "${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
  "${PROJECT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
  DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME}/cmake)

install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

set(CPACK_RESOURCE_FILE_LICENSE "${PROJECT_SOURCE_DIR}/LICENSE")

option(BUILD_TESTING "BUILD WITH TESTS" ON)

if(BUILD_TESTING)
  enable_testing()
  add_subdirectory(test)
endif()
