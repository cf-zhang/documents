# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
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
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/cfzhang/xcode/catkin_gtest/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/cfzhang/xcode/catkin_gtest/build

# Utility rule file for _run_tests_ros_gtest_gtest_addServiceAndClient-test.

# Include the progress variables for this target.
include ros_gtest/CMakeFiles/_run_tests_ros_gtest_gtest_addServiceAndClient-test.dir/progress.make

ros_gtest/CMakeFiles/_run_tests_ros_gtest_gtest_addServiceAndClient-test:
	cd /home/cfzhang/xcode/catkin_gtest/build/ros_gtest && ../catkin_generated/env_cached.sh /usr/bin/python /opt/ros/kinetic/share/catkin/cmake/test/run_tests.py /home/cfzhang/xcode/catkin_gtest/build/test_results/ros_gtest/gtest-addServiceAndClient-test.xml /home/cfzhang/xcode/catkin_gtest/devel/lib/ros_gtest/addServiceAndClient-test\ --gtest_output=xml:/home/cfzhang/xcode/catkin_gtest/build/test_results/ros_gtest/gtest-addServiceAndClient-test.xml

_run_tests_ros_gtest_gtest_addServiceAndClient-test: ros_gtest/CMakeFiles/_run_tests_ros_gtest_gtest_addServiceAndClient-test
_run_tests_ros_gtest_gtest_addServiceAndClient-test: ros_gtest/CMakeFiles/_run_tests_ros_gtest_gtest_addServiceAndClient-test.dir/build.make

.PHONY : _run_tests_ros_gtest_gtest_addServiceAndClient-test

# Rule to build all files generated by this target.
ros_gtest/CMakeFiles/_run_tests_ros_gtest_gtest_addServiceAndClient-test.dir/build: _run_tests_ros_gtest_gtest_addServiceAndClient-test

.PHONY : ros_gtest/CMakeFiles/_run_tests_ros_gtest_gtest_addServiceAndClient-test.dir/build

ros_gtest/CMakeFiles/_run_tests_ros_gtest_gtest_addServiceAndClient-test.dir/clean:
	cd /home/cfzhang/xcode/catkin_gtest/build/ros_gtest && $(CMAKE_COMMAND) -P CMakeFiles/_run_tests_ros_gtest_gtest_addServiceAndClient-test.dir/cmake_clean.cmake
.PHONY : ros_gtest/CMakeFiles/_run_tests_ros_gtest_gtest_addServiceAndClient-test.dir/clean

ros_gtest/CMakeFiles/_run_tests_ros_gtest_gtest_addServiceAndClient-test.dir/depend:
	cd /home/cfzhang/xcode/catkin_gtest/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/cfzhang/xcode/catkin_gtest/src /home/cfzhang/xcode/catkin_gtest/src/ros_gtest /home/cfzhang/xcode/catkin_gtest/build /home/cfzhang/xcode/catkin_gtest/build/ros_gtest /home/cfzhang/xcode/catkin_gtest/build/ros_gtest/CMakeFiles/_run_tests_ros_gtest_gtest_addServiceAndClient-test.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : ros_gtest/CMakeFiles/_run_tests_ros_gtest_gtest_addServiceAndClient-test.dir/depend

