cmake_minimum_required(VERSION 3.8)
project(px4_mpc_controller)

set(CMAKE_CXX_STANDARD 17)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(super_planner REQUIRED)
find_package(Eigen3 REQUIRED)

# acados
set(ACADOS_SOURCE_DIR $ENV{ACADOS_SOURCE_DIR})
if(NOT ACADOS_SOURCE_DIR)
  set(ACADOS_SOURCE_DIR "$ENV{HOME}/acados")
endif()

set(ACADOS_INCLUDE_DIR "${ACADOS_SOURCE_DIR}/include")
set(ACADOS_LIB_DIR "${ACADOS_SOURCE_DIR}/lib")

# Generated solver directory (acados puts code in c_generated_code/)
set(GENERATED_DIR "${CMAKE_CURRENT_SOURCE_DIR}/generated/c_generated_code")

# Check that generated files exist
if(NOT EXISTS "${GENERATED_DIR}/acados_solver_quadrotor.h")
  message(WARNING "acados solver not yet generated. Run: python3 scripts/generate_mpc_solver.py")
  message(WARNING "Skipping mpc_trajectory_node build.")
  # Still create ament package so colcon doesn't fail
  ament_package()
  return()
endif()

# Collect generated C source files (exclude main_*.c which have their own main())
file(GLOB GENERATED_SOURCES
  "${GENERATED_DIR}/acados_solver_quadrotor.c"
  "${GENERATED_DIR}/acados_sim_solver_quadrotor.c"
  "${GENERATED_DIR}/quadrotor_model/*.c"
)

# MPC trajectory node
add_executable(mpc_trajectory_node
  src/mpc_trajectory_node.cpp
  ${GENERATED_SOURCES}
)

target_include_directories(mpc_trajectory_node PRIVATE
  include
  ${GENERATED_DIR}
  ${ACADOS_INCLUDE_DIR}
  ${ACADOS_INCLUDE_DIR}/acados
  ${ACADOS_INCLUDE_DIR}/blasfeo/include
  ${ACADOS_INCLUDE_DIR}/hpipm/include
  ${EIGEN3_INCLUDE_DIR}
)

target_link_directories(mpc_trajectory_node PRIVATE
  ${ACADOS_LIB_DIR}
)

target_link_libraries(mpc_trajectory_node
  acados
  hpipm
  blasfeo
  m
)

ament_target_dependencies(mpc_trajectory_node
  rclcpp std_msgs geometry_msgs px4_msgs super_planner
)

# Install
install(TARGETS mpc_trajectory_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
