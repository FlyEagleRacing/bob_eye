cmake_minimum_required(VERSION 3.22)

project(a2rl_bs_msgs)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(rosidl_default_generators REQUIRED)

set(idl_files
  msg/GpsTimeSync.idl
  msg/EgoState.idl
  msg/Localization.idl
  msg/WheelsData.idl
  msg/Timestamp.idl
  msg/ControllerDebug.idl
  msg/CartesianFrame.idl
  msg/CartesianFrameState.idl
  msg/RaceControlReport.idl
  msg/ReferencePath.idl
  msg/ModuleStatusReport.idl
  msg/HL_Msg_03.idl
  msg/VectornavIns.idl
  msg/FlyeagleEyePlannerReport.idl
  msg/ControllerStatus.idl
  msg/Kistler_Vel_Angle.idl
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${idl_files}
  DEPENDENCIES builtin_interfaces std_msgs sensor_msgs
)

add_library(${PROJECT_NAME}_topics INTERFACE)

target_include_directories(${PROJECT_NAME}_topics
  INTERFACE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
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

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

ament_package()


