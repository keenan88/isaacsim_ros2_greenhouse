# CMake generated Testfile for 
# Source directory: /home/moveit/src/moveit2/moveit_kinematics/test
# Build directory: /home/moveit/build/moveit_kinematics/test
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test([=[test_launch_fanuc-kdl-singular.test.py]=] "/usr/bin/python3" "-u" "/opt/ros/humble/share/ament_cmake_test/cmake/run_test.py" "/home/moveit/build/moveit_kinematics/test_results/moveit_kinematics/test_launch_fanuc-kdl-singular.test.py.xunit.xml" "--package-name" "moveit_kinematics" "--output-file" "/home/moveit/build/moveit_kinematics/ros_test/test_launch_fanuc-kdl-singular.test.py.txt" "--command" "ros2" "test" "/home/moveit/src/moveit2/moveit_kinematics/test/launch/fanuc-kdl-singular.test.py" "test_binary_dir:=/home/moveit/build/moveit_kinematics/test" "--junit-xml=/home/moveit/build/moveit_kinematics/test_results/moveit_kinematics/test_launch_fanuc-kdl-singular.test.py.xunit.xml" "--package-name=moveit_kinematics")
set_tests_properties([=[test_launch_fanuc-kdl-singular.test.py]=] PROPERTIES  TIMEOUT "60" WORKING_DIRECTORY "/home/moveit/build/moveit_kinematics/test" _BACKTRACE_TRIPLES "/opt/ros/humble/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/humble/share/ros_testing/cmake/add_ros_test.cmake;73;ament_add_test;/home/moveit/src/moveit2/moveit_kinematics/test/CMakeLists.txt;20;add_ros_test;/home/moveit/src/moveit2/moveit_kinematics/test/CMakeLists.txt;0;")
add_test([=[test_launch_fanuc-kdl.test.py]=] "/usr/bin/python3" "-u" "/opt/ros/humble/share/ament_cmake_test/cmake/run_test.py" "/home/moveit/build/moveit_kinematics/test_results/moveit_kinematics/test_launch_fanuc-kdl.test.py.xunit.xml" "--package-name" "moveit_kinematics" "--output-file" "/home/moveit/build/moveit_kinematics/ros_test/test_launch_fanuc-kdl.test.py.txt" "--command" "ros2" "test" "/home/moveit/src/moveit2/moveit_kinematics/test/launch/fanuc-kdl.test.py" "test_binary_dir:=/home/moveit/build/moveit_kinematics/test" "--junit-xml=/home/moveit/build/moveit_kinematics/test_results/moveit_kinematics/test_launch_fanuc-kdl.test.py.xunit.xml" "--package-name=moveit_kinematics")
set_tests_properties([=[test_launch_fanuc-kdl.test.py]=] PROPERTIES  TIMEOUT "60" WORKING_DIRECTORY "/home/moveit/build/moveit_kinematics/test" _BACKTRACE_TRIPLES "/opt/ros/humble/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/humble/share/ros_testing/cmake/add_ros_test.cmake;73;ament_add_test;/home/moveit/src/moveit2/moveit_kinematics/test/CMakeLists.txt;21;add_ros_test;/home/moveit/src/moveit2/moveit_kinematics/test/CMakeLists.txt;0;")
add_test([=[test_launch_panda-kdl-singular.test.py]=] "/usr/bin/python3" "-u" "/opt/ros/humble/share/ament_cmake_test/cmake/run_test.py" "/home/moveit/build/moveit_kinematics/test_results/moveit_kinematics/test_launch_panda-kdl-singular.test.py.xunit.xml" "--package-name" "moveit_kinematics" "--output-file" "/home/moveit/build/moveit_kinematics/ros_test/test_launch_panda-kdl-singular.test.py.txt" "--command" "ros2" "test" "/home/moveit/src/moveit2/moveit_kinematics/test/launch/panda-kdl-singular.test.py" "test_binary_dir:=/home/moveit/build/moveit_kinematics/test" "--junit-xml=/home/moveit/build/moveit_kinematics/test_results/moveit_kinematics/test_launch_panda-kdl-singular.test.py.xunit.xml" "--package-name=moveit_kinematics")
set_tests_properties([=[test_launch_panda-kdl-singular.test.py]=] PROPERTIES  TIMEOUT "60" WORKING_DIRECTORY "/home/moveit/build/moveit_kinematics/test" _BACKTRACE_TRIPLES "/opt/ros/humble/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/humble/share/ros_testing/cmake/add_ros_test.cmake;73;ament_add_test;/home/moveit/src/moveit2/moveit_kinematics/test/CMakeLists.txt;22;add_ros_test;/home/moveit/src/moveit2/moveit_kinematics/test/CMakeLists.txt;0;")
add_test([=[test_launch_panda-kdl.test.py]=] "/usr/bin/python3" "-u" "/opt/ros/humble/share/ament_cmake_test/cmake/run_test.py" "/home/moveit/build/moveit_kinematics/test_results/moveit_kinematics/test_launch_panda-kdl.test.py.xunit.xml" "--package-name" "moveit_kinematics" "--output-file" "/home/moveit/build/moveit_kinematics/ros_test/test_launch_panda-kdl.test.py.txt" "--command" "ros2" "test" "/home/moveit/src/moveit2/moveit_kinematics/test/launch/panda-kdl.test.py" "test_binary_dir:=/home/moveit/build/moveit_kinematics/test" "--junit-xml=/home/moveit/build/moveit_kinematics/test_results/moveit_kinematics/test_launch_panda-kdl.test.py.xunit.xml" "--package-name=moveit_kinematics")
set_tests_properties([=[test_launch_panda-kdl.test.py]=] PROPERTIES  TIMEOUT "60" WORKING_DIRECTORY "/home/moveit/build/moveit_kinematics/test" _BACKTRACE_TRIPLES "/opt/ros/humble/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/humble/share/ros_testing/cmake/add_ros_test.cmake;73;ament_add_test;/home/moveit/src/moveit2/moveit_kinematics/test/CMakeLists.txt;23;add_ros_test;/home/moveit/src/moveit2/moveit_kinematics/test/CMakeLists.txt;0;")
subdirs("../gtest")
