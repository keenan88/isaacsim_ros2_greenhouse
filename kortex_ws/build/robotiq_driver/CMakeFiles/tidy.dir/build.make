# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.22

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/kortex_ws/src/ros2_robotiq_gripper/robotiq_driver

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/kortex_ws/build/robotiq_driver

# Utility rule file for tidy.

# Include any custom commands dependencies for this target.
include CMakeFiles/tidy.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/tidy.dir/progress.make

CMakeFiles/tidy:
	cd /home/kortex_ws/src/ros2_robotiq_gripper/robotiq_driver && clang-tidy -p /home/kortex_ws/build/robotiq_driver `git ls-files *.cpp`

tidy: CMakeFiles/tidy
tidy: CMakeFiles/tidy.dir/build.make
.PHONY : tidy

# Rule to build all files generated by this target.
CMakeFiles/tidy.dir/build: tidy
.PHONY : CMakeFiles/tidy.dir/build

CMakeFiles/tidy.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/tidy.dir/cmake_clean.cmake
.PHONY : CMakeFiles/tidy.dir/clean

CMakeFiles/tidy.dir/depend:
	cd /home/kortex_ws/build/robotiq_driver && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/kortex_ws/src/ros2_robotiq_gripper/robotiq_driver /home/kortex_ws/src/ros2_robotiq_gripper/robotiq_driver /home/kortex_ws/build/robotiq_driver /home/kortex_ws/build/robotiq_driver /home/kortex_ws/build/robotiq_driver/CMakeFiles/tidy.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/tidy.dir/depend
