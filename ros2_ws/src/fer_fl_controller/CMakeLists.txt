cmake_minimum_required(VERSION 3.8)
project(fer_fl_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(orocos_kdl_vendor REQUIRED)

# ── CasADi (pip-installed; also works for system-wide installs) ───────────────
find_package(casadi REQUIRED
  HINTS
    "$ENV{HOME}/.local/lib/python3.12/site-packages/casadi/cmake"
    "/usr/local/lib/python3.12/site-packages/casadi/cmake"
    "/usr/lib/python3/dist-packages/casadi/cmake"
)

# casadi_DIR → directory containing casadi-config.cmake (…/casadi/cmake).
# One level up is the installation root that holds libcasadi.so and all plugins.
get_filename_component(CASADI_LIBRARY_DIR "${casadi_DIR}" DIRECTORY)

# The pip-installed CasADi cmake config propagates _GLIBCXX_USE_CXX11_ABI=0 as
# an INTERFACE definition, but the actual libcasadi.so was compiled with old-ABI
# std::string (Ss mangling), so consumers MUST be compiled with the same flag.
# ROS 2 Jazzy uses new-ABI exclusively.  We resolve this by:
#  1. Clearing the INTERFACE definition so it does not leak to the main node.
#  2. Creating a thin shared-library wrapper (casadi_qp_wrapper) that is compiled
#     with old ABI via a PRIVATE target_compile_definitions and exposes only a
#     pure-C interface — no C++ types cross the shared-library boundary.
set_target_properties(casadi::casadi PROPERTIES INTERFACE_COMPILE_DEFINITIONS "")

# ── Existing executables (unchanged — original node stays exactly as-is) ──────
add_executable(feedback_linearization_node src/feedback_linearization_node.cpp)
add_executable(mpc_controller_node         src/mpc_controller_node.cpp)
add_executable(fk_ik_waypoint_tester       src/fk_ik_waypoint_tester.cpp)

foreach(target
  feedback_linearization_node
  mpc_controller_node
  fk_ik_waypoint_tester
)
  ament_target_dependencies(${target}
    rclcpp
    sensor_msgs
    std_msgs
    kdl_parser
    orocos_kdl_vendor
  )

  target_link_libraries(${target}
    orocos-kdl
  )

  target_include_directories(${target} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
endforeach()

# ── CasADi QP wrapper shared library ──────────────────────────────────────────
# Compiled with _GLIBCXX_USE_CXX11_ABI=0 (old ABI) to match pip-installed CasADi.
# Exposes only a C interface (casadi_qp_wrapper.h) so no C++ ABI crosses the
# shared-library boundary at runtime.
add_library(casadi_qp_wrapper SHARED
  src/casadi_qp_wrapper.cpp
)

target_compile_definitions(casadi_qp_wrapper PRIVATE
  "_GLIBCXX_USE_CXX11_ABI=0"
  "CASADI_INSTALL_DIR=\"${CASADI_LIBRARY_DIR}\""
)

target_link_libraries(casadi_qp_wrapper PRIVATE
  casadi::casadi
)

target_include_directories(casadi_qp_wrapper PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

set_target_properties(casadi_qp_wrapper PROPERTIES
  BUILD_RPATH            "${CASADI_LIBRARY_DIR}"
  INSTALL_RPATH          "${CASADI_LIBRARY_DIR}"
  INSTALL_RPATH_USE_LINK_PATH TRUE
)

# ── New coupled-QP node ────────────────────────────────────────────────────────
# Compiled with the default (new) ABI to match ROS 2 Jazzy.  Calls CasADi only
# through the C interface of casadi_qp_wrapper — no CasADi headers included here.
add_executable(mpc_coupled_qp_node src/mpc_coupled_qp_node.cpp)

ament_target_dependencies(mpc_coupled_qp_node
  rclcpp
  sensor_msgs
  std_msgs
  kdl_parser
  orocos_kdl_vendor
)

target_link_libraries(mpc_coupled_qp_node
  orocos-kdl
  casadi_qp_wrapper
)

target_include_directories(mpc_coupled_qp_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

# Provide the CasADi library directory as a compile-time constant for diagnostics.
target_compile_definitions(mpc_coupled_qp_node PRIVATE
  "CASADI_INSTALL_DIR=\"${CASADI_LIBRARY_DIR}\""
)

# Embed RPATH so the executable finds casadi_qp_wrapper.so at runtime.
set_target_properties(mpc_coupled_qp_node PROPERTIES
  BUILD_RPATH            "${CMAKE_CURRENT_BINARY_DIR}:${CASADI_LIBRARY_DIR}"
  INSTALL_RPATH          "$ORIGIN:${CASADI_LIBRARY_DIR}"
  INSTALL_RPATH_USE_LINK_PATH TRUE
)

# ── Install ────────────────────────────────────────────────────────────────────
install(TARGETS
  feedback_linearization_node
  mpc_controller_node
  fk_ik_waypoint_tester
  mpc_coupled_qp_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install the wrapper as a SHARED LIBRARY with explicit execute permissions
# (plain DESTINATION without LIBRARY keyword gives 644 on some CMake/colcon combos).
install(TARGETS casadi_qp_wrapper
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  PERMISSIONS
    OWNER_READ OWNER_WRITE OWNER_EXECUTE
    GROUP_READ GROUP_EXECUTE
    WORLD_READ WORLD_EXECUTE
)

ament_package()
