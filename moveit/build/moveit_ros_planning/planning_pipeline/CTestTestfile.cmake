# CMake generated Testfile for 
# Source directory: /home/moveit/src/moveit2/moveit_ros/planning/planning_pipeline
# Build directory: /home/moveit/build/moveit_ros_planning/planning_pipeline
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test([=[moveit_planning_pipeline_test]=] "/usr/bin/python3" "-u" "/opt/ros/humble/share/ament_cmake_test/cmake/run_test.py" "/home/moveit/build/moveit_ros_planning/test_results/moveit_ros_planning/moveit_planning_pipeline_test.gtest.xml" "--package-name" "moveit_ros_planning" "--output-file" "/home/moveit/build/moveit_ros_planning/ament_cmake_gtest/moveit_planning_pipeline_test.txt" "--command" "/home/moveit/build/moveit_ros_planning/planning_pipeline/moveit_planning_pipeline_test" "--gtest_output=xml:/home/moveit/build/moveit_ros_planning/test_results/moveit_ros_planning/moveit_planning_pipeline_test.gtest.xml")
set_tests_properties([=[moveit_planning_pipeline_test]=] PROPERTIES  LABELS "gtest" REQUIRED_FILES "/home/moveit/build/moveit_ros_planning/planning_pipeline/moveit_planning_pipeline_test" TIMEOUT "60" WORKING_DIRECTORY "/home/moveit/build/moveit_ros_planning/planning_pipeline" _BACKTRACE_TRIPLES "/opt/ros/humble/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/humble/share/ament_cmake_gtest/cmake/ament_add_gtest_test.cmake;86;ament_add_test;/opt/ros/humble/share/ament_cmake_gtest/cmake/ament_add_gtest.cmake;93;ament_add_gtest_test;/home/moveit/src/moveit2/moveit_ros/planning/planning_pipeline/CMakeLists.txt;31;ament_add_gtest;/home/moveit/src/moveit2/moveit_ros/planning/planning_pipeline/CMakeLists.txt;0;")
subdirs("../gtest")
