cmake_minimum_required(VERSION 3.8)

set(CMAKE_CXX_STANDARD 20)

project(eav24_bsu_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(a2rl_bs_msgs REQUIRED)

set(idl_files
    msg/SW_Versions_03.idl
    msg/RC_Status_01.idl
    msg/DiagnosticWord_02.idl
    msg/EM_Status_01.idl
    msg/ICE_Status_02.idl
    msg/ICE_Status_01.idl
    msg/DiagnosticWord_01.idl
    msg/HL_Trajectory_02.idl
    msg/HL_Trajectory_01.idl
    msg/BSU_Status_01.idl
    msg/PDUs_Status_01.idl
    msg/PSA_Status_02.idl
    msg/SW_Versions_01.idl
    msg/SW_Versions_02.idl
    msg/PSA_Status_01.idl
    msg/CBA_Status_RR.idl
    msg/CBA_Status_RL.idl
    msg/CBA_Status_FR.idl
    msg/CBA_Status_FL.idl
    msg/Wheels_Speed_01.idl
    msg/HL_Msg_03.idl
    msg/BSU_Ranges_02.idl
    msg/BSU_Ranges_01.idl
    msg/HL_Msg_02.idl
    msg/HL_Msg_01.idl
    msg/Tpms_Front.idl
    msg/Tpms_Rear.idl
    msg/Tyre_Surface_Temp_Front.idl
    msg/Tyre_Surface_Temp_Rear.idl
    msg/Brake_Disk_Temp.idl
    msg/Kistler_Acc_Body.idl
    msg/Kistler_Angle_Vel_Body.idl
    msg/Kistler_Correvit.idl
    msg/Kistler_Distance.idl
    msg/Kistler_Vel_Angle.idl

)

rosidl_generate_interfaces(${PROJECT_NAME}
    ${idl_files}
  DEPENDENCIES
    a2rl_bs_msgs
)

ament_package()
