cmake_minimum_required(VERSION 3.16)
project(wuji_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# --- Wuji C SDK -------------------------------------------------------------------------------------------------------
# Default: the release tarball checked into src/third_party/wuji-sdk-c (no network at build time), unpacked into the
# build tree. -DWUJI_SDK_ROOT=<dir with include/wuji_sdk.h and lib/libwuji_sdk_c.so> uses another copy.
set(WUJI_SDK_VERSION "2026.8.31" CACHE STRING "wuji-sdk-c release in src/third_party/wuji-sdk-c")
set(WUJI_SDK_THIRD_PARTY_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/wuji-sdk-c" CACHE PATH
  "Directory holding wuji-sdk-c-<version>-<triplet>.tar.gz")

if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64|ARM64")
  set(WUJI_SDK_TRIPLET "aarch64-linux-gnu")
else()
  set(WUJI_SDK_TRIPLET "x86_64-linux-gnu")
endif()

if(WUJI_SDK_ROOT)
  set(WUJI_SDK_INCLUDE_DIR "${WUJI_SDK_ROOT}/include")
  set(WUJI_SDK_LIB "${WUJI_SDK_ROOT}/lib/libwuji_sdk_c.so")
  message(STATUS "Wuji SDK override -DWUJI_SDK_ROOT=${WUJI_SDK_ROOT}")
else()
  set(_wuji_name "wuji-sdk-c-${WUJI_SDK_VERSION}-${WUJI_SDK_TRIPLET}")
  set(_wuji_tarball "${WUJI_SDK_THIRD_PARTY_DIR}/${_wuji_name}.tar.gz")
  set(_wuji_extract_root "${CMAKE_CURRENT_BINARY_DIR}/wuji-sdk-c")
  set(WUJI_SDK_INCLUDE_DIR "${_wuji_extract_root}/${_wuji_name}/include")
  set(WUJI_SDK_LIB "${_wuji_extract_root}/${_wuji_name}/lib/libwuji_sdk_c.so")
  if(NOT EXISTS "${_wuji_tarball}")
    message(FATAL_ERROR "Wuji SDK tarball not found: ${_wuji_tarball} (or set -DWUJI_SDK_ROOT=)")
  endif()
  if(NOT EXISTS "${WUJI_SDK_LIB}" OR "${_wuji_tarball}" IS_NEWER_THAN "${WUJI_SDK_LIB}")
    file(REMOVE_RECURSE "${_wuji_extract_root}")
    file(MAKE_DIRECTORY "${_wuji_extract_root}")
    file(ARCHIVE_EXTRACT INPUT "${_wuji_tarball}" DESTINATION "${_wuji_extract_root}")
  endif()
  message(STATUS "Using wuji-sdk-c ${WUJI_SDK_VERSION} (${WUJI_SDK_TRIPLET}) from ${WUJI_SDK_THIRD_PARTY_DIR}")
endif()

if(NOT EXISTS "${WUJI_SDK_INCLUDE_DIR}/wuji_sdk.h" OR NOT EXISTS "${WUJI_SDK_LIB}")
  message(FATAL_ERROR "Wuji SDK incomplete: need ${WUJI_SDK_INCLUDE_DIR}/wuji_sdk.h and ${WUJI_SDK_LIB}")
endif()

add_library(wuji_sdk_c SHARED IMPORTED)
set_target_properties(wuji_sdk_c PROPERTIES
  IMPORTED_LOCATION "${WUJI_SDK_LIB}"
  INTERFACE_INCLUDE_DIRECTORIES "${WUJI_SDK_INCLUDE_DIR}"
)

add_library(${PROJECT_NAME} SHARED
  src/hands/hand2/wuji_hand2_hardware.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
ament_target_dependencies(${PROJECT_NAME} PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
# PRIVATE: pluginlib loads the plugin; the SDK is not re-exported.
target_link_libraries(${PROJECT_NAME} PRIVATE wuji_sdk_c)
# The SDK is installed next to the plugin and found through $ORIGIN.
set_target_properties(${PROJECT_NAME} PROPERTIES
  BUILD_WITH_INSTALL_RPATH FALSE
  INSTALL_RPATH "$ORIGIN"
  INSTALL_RPATH_USE_LINK_PATH TRUE
)

pluginlib_export_plugin_description_file(hardware_interface wuji_hardware.xml)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(FILES "${WUJI_SDK_LIB}" DESTINATION lib)

ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_package()
