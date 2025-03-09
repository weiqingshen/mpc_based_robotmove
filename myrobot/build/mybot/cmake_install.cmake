# Install script for directory: /home/fins/myrobot/src/mybot

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/fins/myrobot/install/mybot")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/move_group_interface" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/move_group_interface")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/move_group_interface"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/mybot" TYPE EXECUTABLE FILES "/home/fins/myrobot/build/mybot/move_group_interface")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/move_group_interface" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/move_group_interface")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/move_group_interface"
         OLD_RPATH "/home/fins/Workspace/moveit2_ws/install/moveit_ros_planning_interface/lib:/home/fins/Workspace/moveit2_ws/install/moveit_visual_tools/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_move_group/lib:/opt/ros/humble/lib/x86_64-linux-gnu:/opt/ros/humble/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_warehouse/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_planning/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_occupancy_map_monitor/lib:/home/fins/Workspace/moveit2_ws/install/moveit_core/lib:/home/fins/Workspace/moveit2_ws/install/srdfdom/lib:/opt/ros/humble/opt/rviz_ogre_vendor/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/move_group_interface")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/end_effector_feedback" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/end_effector_feedback")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/end_effector_feedback"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/mybot" TYPE EXECUTABLE FILES "/home/fins/myrobot/build/mybot/end_effector_feedback")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/end_effector_feedback" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/end_effector_feedback")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/end_effector_feedback"
         OLD_RPATH "/home/fins/Workspace/moveit2_ws/install/moveit_ros_planning_interface/lib:/home/fins/Workspace/moveit2_ws/install/moveit_visual_tools/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_move_group/lib:/opt/ros/humble/lib/x86_64-linux-gnu:/opt/ros/humble/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_warehouse/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_planning/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_occupancy_map_monitor/lib:/home/fins/Workspace/moveit2_ws/install/moveit_core/lib:/home/fins/Workspace/moveit2_ws/install/srdfdom/lib:/opt/ros/humble/opt/rviz_ogre_vendor/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/end_effector_feedback")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/execute" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/execute")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/execute"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/mybot" TYPE EXECUTABLE FILES "/home/fins/myrobot/build/mybot/execute")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/execute" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/execute")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/execute"
         OLD_RPATH "/home/fins/Workspace/moveit2_ws/install/moveit_ros_planning_interface/lib:/home/fins/Workspace/moveit2_ws/install/moveit_visual_tools/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_move_group/lib:/opt/ros/humble/lib/x86_64-linux-gnu:/opt/ros/humble/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_warehouse/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_planning/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_occupancy_map_monitor/lib:/home/fins/Workspace/moveit2_ws/install/moveit_core/lib:/home/fins/Workspace/moveit2_ws/install/srdfdom/lib:/opt/ros/humble/opt/rviz_ogre_vendor/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/execute")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/set_target_pose" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/set_target_pose")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/set_target_pose"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/mybot" TYPE EXECUTABLE FILES "/home/fins/myrobot/build/mybot/set_target_pose")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/set_target_pose" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/set_target_pose")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/set_target_pose"
         OLD_RPATH "/home/fins/Workspace/moveit2_ws/install/moveit_ros_planning_interface/lib:/home/fins/Workspace/moveit2_ws/install/moveit_visual_tools/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_move_group/lib:/opt/ros/humble/lib/x86_64-linux-gnu:/opt/ros/humble/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_warehouse/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_planning/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_occupancy_map_monitor/lib:/home/fins/Workspace/moveit2_ws/install/moveit_core/lib:/home/fins/Workspace/moveit2_ws/install/srdfdom/lib:/opt/ros/humble/opt/rviz_ogre_vendor/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/set_target_pose")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/arm_execute_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/arm_execute_node")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/arm_execute_node"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/mybot" TYPE EXECUTABLE FILES "/home/fins/myrobot/build/mybot/arm_execute_node")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/arm_execute_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/arm_execute_node")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/arm_execute_node"
         OLD_RPATH "/home/fins/Workspace/moveit2_ws/install/moveit_ros_planning_interface/lib:/home/fins/Workspace/moveit2_ws/install/moveit_visual_tools/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_move_group/lib:/opt/ros/humble/lib/x86_64-linux-gnu:/opt/ros/humble/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_warehouse/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_planning/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_occupancy_map_monitor/lib:/home/fins/Workspace/moveit2_ws/install/moveit_core/lib:/home/fins/Workspace/moveit2_ws/install/srdfdom/lib:/opt/ros/humble/opt/rviz_ogre_vendor/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/arm_execute_node")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/trajectory_buffer_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/trajectory_buffer_node")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/trajectory_buffer_node"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/mybot" TYPE EXECUTABLE FILES "/home/fins/myrobot/build/mybot/trajectory_buffer_node")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/trajectory_buffer_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/trajectory_buffer_node")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/trajectory_buffer_node"
         OLD_RPATH "/home/fins/Workspace/moveit2_ws/install/moveit_ros_planning_interface/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_move_group/lib:/opt/ros/humble/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_warehouse/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_planning/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_occupancy_map_monitor/lib:/home/fins/Workspace/moveit2_ws/install/moveit_core/lib:/opt/ros/humble/lib/x86_64-linux-gnu:/home/fins/Workspace/moveit2_ws/install/srdfdom/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/trajectory_buffer_node")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/arm_execute_storage" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/arm_execute_storage")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/arm_execute_storage"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/mybot" TYPE EXECUTABLE FILES "/home/fins/myrobot/build/mybot/arm_execute_storage")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/arm_execute_storage" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/arm_execute_storage")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/arm_execute_storage"
         OLD_RPATH "/home/fins/Workspace/moveit2_ws/install/moveit_ros_planning_interface/lib:/home/fins/Workspace/moveit2_ws/install/moveit_visual_tools/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_move_group/lib:/opt/ros/humble/lib/x86_64-linux-gnu:/opt/ros/humble/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_warehouse/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_planning/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_occupancy_map_monitor/lib:/home/fins/Workspace/moveit2_ws/install/moveit_core/lib:/home/fins/Workspace/moveit2_ws/install/srdfdom/lib:/opt/ros/humble/opt/rviz_ogre_vendor/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/arm_execute_storage")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/arm_execute_processing" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/arm_execute_processing")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/arm_execute_processing"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/mybot" TYPE EXECUTABLE FILES "/home/fins/myrobot/build/mybot/arm_execute_processing")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/arm_execute_processing" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/arm_execute_processing")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/arm_execute_processing"
         OLD_RPATH "/home/fins/Workspace/moveit2_ws/install/moveit_ros_planning_interface/lib:/home/fins/Workspace/moveit2_ws/install/moveit_visual_tools/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_move_group/lib:/opt/ros/humble/lib/x86_64-linux-gnu:/opt/ros/humble/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_warehouse/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_planning/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_occupancy_map_monitor/lib:/home/fins/Workspace/moveit2_ws/install/moveit_core/lib:/home/fins/Workspace/moveit2_ws/install/srdfdom/lib:/opt/ros/humble/opt/rviz_ogre_vendor/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/arm_execute_processing")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/arm_execute_storage_trajectory" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/arm_execute_storage_trajectory")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/arm_execute_storage_trajectory"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/mybot" TYPE EXECUTABLE FILES "/home/fins/myrobot/build/mybot/arm_execute_storage_trajectory")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/arm_execute_storage_trajectory" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/arm_execute_storage_trajectory")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/arm_execute_storage_trajectory"
         OLD_RPATH "/home/fins/Workspace/moveit2_ws/install/moveit_ros_planning_interface/lib:/home/fins/Workspace/moveit2_ws/install/moveit_visual_tools/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_move_group/lib:/opt/ros/humble/lib/x86_64-linux-gnu:/opt/ros/humble/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_warehouse/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_planning/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_occupancy_map_monitor/lib:/home/fins/Workspace/moveit2_ws/install/moveit_core/lib:/home/fins/Workspace/moveit2_ws/install/srdfdom/lib:/opt/ros/humble/opt/rviz_ogre_vendor/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/arm_execute_storage_trajectory")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/arm_execute_second_storage_trajectory" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/arm_execute_second_storage_trajectory")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/arm_execute_second_storage_trajectory"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/mybot" TYPE EXECUTABLE FILES "/home/fins/myrobot/build/mybot/arm_execute_second_storage_trajectory")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/arm_execute_second_storage_trajectory" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/arm_execute_second_storage_trajectory")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/arm_execute_second_storage_trajectory"
         OLD_RPATH "/home/fins/Workspace/moveit2_ws/install/moveit_ros_planning_interface/lib:/home/fins/Workspace/moveit2_ws/install/moveit_visual_tools/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_move_group/lib:/opt/ros/humble/lib/x86_64-linux-gnu:/opt/ros/humble/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_warehouse/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_planning/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_occupancy_map_monitor/lib:/home/fins/Workspace/moveit2_ws/install/moveit_core/lib:/home/fins/Workspace/moveit2_ws/install/srdfdom/lib:/opt/ros/humble/opt/rviz_ogre_vendor/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/arm_execute_second_storage_trajectory")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/arm_execute_processing_trajectory" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/arm_execute_processing_trajectory")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/arm_execute_processing_trajectory"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/mybot" TYPE EXECUTABLE FILES "/home/fins/myrobot/build/mybot/arm_execute_processing_trajectory")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/arm_execute_processing_trajectory" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/arm_execute_processing_trajectory")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/arm_execute_processing_trajectory"
         OLD_RPATH "/home/fins/Workspace/moveit2_ws/install/moveit_ros_planning_interface/lib:/home/fins/Workspace/moveit2_ws/install/moveit_visual_tools/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_move_group/lib:/opt/ros/humble/lib/x86_64-linux-gnu:/opt/ros/humble/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_warehouse/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_planning/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_occupancy_map_monitor/lib:/home/fins/Workspace/moveit2_ws/install/moveit_core/lib:/home/fins/Workspace/moveit2_ws/install/srdfdom/lib:/opt/ros/humble/opt/rviz_ogre_vendor/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/arm_execute_processing_trajectory")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/arm_execute_total_trajectory_final" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/arm_execute_total_trajectory_final")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/arm_execute_total_trajectory_final"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/mybot" TYPE EXECUTABLE FILES "/home/fins/myrobot/build/mybot/arm_execute_total_trajectory_final")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/arm_execute_total_trajectory_final" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/arm_execute_total_trajectory_final")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/arm_execute_total_trajectory_final"
         OLD_RPATH "/home/fins/Workspace/moveit2_ws/install/moveit_ros_planning_interface/lib:/home/fins/Workspace/moveit2_ws/install/moveit_visual_tools/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_move_group/lib:/opt/ros/humble/lib/x86_64-linux-gnu:/opt/ros/humble/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_warehouse/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_planning/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_occupancy_map_monitor/lib:/home/fins/Workspace/moveit2_ws/install/moveit_core/lib:/home/fins/Workspace/moveit2_ws/install/srdfdom/lib:/opt/ros/humble/opt/rviz_ogre_vendor/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/arm_execute_total_trajectory_final")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/arm_execute_total_trajectory_second" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/arm_execute_total_trajectory_second")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/arm_execute_total_trajectory_second"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/mybot" TYPE EXECUTABLE FILES "/home/fins/myrobot/build/mybot/arm_execute_total_trajectory_second")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/arm_execute_total_trajectory_second" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/arm_execute_total_trajectory_second")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/arm_execute_total_trajectory_second"
         OLD_RPATH "/home/fins/Workspace/moveit2_ws/install/moveit_ros_planning_interface/lib:/home/fins/Workspace/moveit2_ws/install/moveit_visual_tools/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_move_group/lib:/opt/ros/humble/lib/x86_64-linux-gnu:/opt/ros/humble/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_warehouse/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_planning/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_occupancy_map_monitor/lib:/home/fins/Workspace/moveit2_ws/install/moveit_core/lib:/home/fins/Workspace/moveit2_ws/install/srdfdom/lib:/opt/ros/humble/opt/rviz_ogre_vendor/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/arm_execute_total_trajectory_second")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/arm_execute_total_trajectory_second_final" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/arm_execute_total_trajectory_second_final")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/arm_execute_total_trajectory_second_final"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/mybot" TYPE EXECUTABLE FILES "/home/fins/myrobot/build/mybot/arm_execute_total_trajectory_second_final")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/arm_execute_total_trajectory_second_final" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/arm_execute_total_trajectory_second_final")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/arm_execute_total_trajectory_second_final"
         OLD_RPATH "/home/fins/Workspace/moveit2_ws/install/moveit_ros_planning_interface/lib:/home/fins/Workspace/moveit2_ws/install/moveit_visual_tools/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_move_group/lib:/opt/ros/humble/lib/x86_64-linux-gnu:/opt/ros/humble/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_warehouse/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_planning/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_occupancy_map_monitor/lib:/home/fins/Workspace/moveit2_ws/install/moveit_core/lib:/home/fins/Workspace/moveit2_ws/install/srdfdom/lib:/opt/ros/humble/opt/rviz_ogre_vendor/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/arm_execute_total_trajectory_second_final")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/arm_execute_total_trajectory" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/arm_execute_total_trajectory")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/arm_execute_total_trajectory"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/mybot" TYPE EXECUTABLE FILES "/home/fins/myrobot/build/mybot/arm_execute_total_trajectory")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/arm_execute_total_trajectory" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/arm_execute_total_trajectory")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/arm_execute_total_trajectory"
         OLD_RPATH "/home/fins/Workspace/moveit2_ws/install/moveit_ros_planning_interface/lib:/home/fins/Workspace/moveit2_ws/install/moveit_visual_tools/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_move_group/lib:/opt/ros/humble/lib/x86_64-linux-gnu:/opt/ros/humble/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_warehouse/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_planning/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_occupancy_map_monitor/lib:/home/fins/Workspace/moveit2_ws/install/moveit_core/lib:/home/fins/Workspace/moveit2_ws/install/srdfdom/lib:/opt/ros/humble/opt/rviz_ogre_vendor/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/arm_execute_total_trajectory")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/arm_execute_circle_trajectory" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/arm_execute_circle_trajectory")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/arm_execute_circle_trajectory"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/mybot" TYPE EXECUTABLE FILES "/home/fins/myrobot/build/mybot/arm_execute_circle_trajectory")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/arm_execute_circle_trajectory" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/arm_execute_circle_trajectory")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/arm_execute_circle_trajectory"
         OLD_RPATH "/home/fins/Workspace/moveit2_ws/install/moveit_ros_planning_interface/lib:/home/fins/Workspace/moveit2_ws/install/moveit_visual_tools/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_move_group/lib:/opt/ros/humble/lib/x86_64-linux-gnu:/opt/ros/humble/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_warehouse/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_planning/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_occupancy_map_monitor/lib:/home/fins/Workspace/moveit2_ws/install/moveit_core/lib:/home/fins/Workspace/moveit2_ws/install/srdfdom/lib:/opt/ros/humble/opt/rviz_ogre_vendor/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/arm_execute_circle_trajectory")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/arm_execute_circle" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/arm_execute_circle")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/arm_execute_circle"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/mybot" TYPE EXECUTABLE FILES "/home/fins/myrobot/build/mybot/arm_execute_circle")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/arm_execute_circle" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/arm_execute_circle")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/arm_execute_circle"
         OLD_RPATH "/home/fins/Workspace/moveit2_ws/install/moveit_ros_planning_interface/lib:/home/fins/Workspace/moveit2_ws/install/moveit_visual_tools/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_move_group/lib:/opt/ros/humble/lib/x86_64-linux-gnu:/opt/ros/humble/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_warehouse/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_planning/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_occupancy_map_monitor/lib:/home/fins/Workspace/moveit2_ws/install/moveit_core/lib:/home/fins/Workspace/moveit2_ws/install/srdfdom/lib:/opt/ros/humble/opt/rviz_ogre_vendor/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/arm_execute_circle")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mybot/src" TYPE DIRECTORY FILES "/home/fins/myrobot/src/mybot/src/" FILES_MATCHING REGEX "/[^/]*\\.cpp$" REGEX "/[^/]*\\.h$" REGEX "/[^/]*\\.hpp$" REGEX "/[^/]*\\.c$")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mybot/config" TYPE DIRECTORY FILES "/home/fins/myrobot/src/mybot/config/")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mybot" TYPE DIRECTORY FILES "/home/fins/myrobot/src/mybot/launch" REGEX "/setup\\_assistant\\.launch$" EXCLUDE)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mybot" TYPE FILE FILES "/home/fins/myrobot/src/mybot/.setup_assistant")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/package_run_dependencies" TYPE FILE FILES "/home/fins/myrobot/build/mybot/ament_cmake_index/share/ament_index/resource_index/package_run_dependencies/mybot")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/parent_prefix_path" TYPE FILE FILES "/home/fins/myrobot/build/mybot/ament_cmake_index/share/ament_index/resource_index/parent_prefix_path/mybot")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mybot/environment" TYPE FILE FILES "/opt/ros/humble/share/ament_cmake_core/cmake/environment_hooks/environment/ament_prefix_path.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mybot/environment" TYPE FILE FILES "/home/fins/myrobot/build/mybot/ament_cmake_environment_hooks/ament_prefix_path.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mybot/environment" TYPE FILE FILES "/opt/ros/humble/share/ament_cmake_core/cmake/environment_hooks/environment/path.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mybot/environment" TYPE FILE FILES "/home/fins/myrobot/build/mybot/ament_cmake_environment_hooks/path.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mybot" TYPE FILE FILES "/home/fins/myrobot/build/mybot/ament_cmake_environment_hooks/local_setup.bash")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mybot" TYPE FILE FILES "/home/fins/myrobot/build/mybot/ament_cmake_environment_hooks/local_setup.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mybot" TYPE FILE FILES "/home/fins/myrobot/build/mybot/ament_cmake_environment_hooks/local_setup.zsh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mybot" TYPE FILE FILES "/home/fins/myrobot/build/mybot/ament_cmake_environment_hooks/local_setup.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mybot" TYPE FILE FILES "/home/fins/myrobot/build/mybot/ament_cmake_environment_hooks/package.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/packages" TYPE FILE FILES "/home/fins/myrobot/build/mybot/ament_cmake_index/share/ament_index/resource_index/packages/mybot")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mybot/cmake" TYPE FILE FILES
    "/home/fins/myrobot/build/mybot/ament_cmake_core/mybotConfig.cmake"
    "/home/fins/myrobot/build/mybot/ament_cmake_core/mybotConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mybot" TYPE FILE FILES "/home/fins/myrobot/src/mybot/package.xml")
endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/fins/myrobot/build/mybot/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
