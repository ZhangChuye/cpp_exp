# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

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
CMAKE_SOURCE_DIR = /home/tingxfan/cpp_exp/study_pointcloud

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/tingxfan/cpp_exp/study_pointcloud/build

# Include any dependencies generated for this target.
include CMakeFiles/pcl_debug.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/pcl_debug.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/pcl_debug.dir/flags.make

CMakeFiles/pcl_debug.dir/pcl_debug.cpp.o: CMakeFiles/pcl_debug.dir/flags.make
CMakeFiles/pcl_debug.dir/pcl_debug.cpp.o: ../pcl_debug.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/tingxfan/cpp_exp/study_pointcloud/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/pcl_debug.dir/pcl_debug.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/pcl_debug.dir/pcl_debug.cpp.o -c /home/tingxfan/cpp_exp/study_pointcloud/pcl_debug.cpp

CMakeFiles/pcl_debug.dir/pcl_debug.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/pcl_debug.dir/pcl_debug.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/tingxfan/cpp_exp/study_pointcloud/pcl_debug.cpp > CMakeFiles/pcl_debug.dir/pcl_debug.cpp.i

CMakeFiles/pcl_debug.dir/pcl_debug.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/pcl_debug.dir/pcl_debug.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/tingxfan/cpp_exp/study_pointcloud/pcl_debug.cpp -o CMakeFiles/pcl_debug.dir/pcl_debug.cpp.s

# Object files for target pcl_debug
pcl_debug_OBJECTS = \
"CMakeFiles/pcl_debug.dir/pcl_debug.cpp.o"

# External object files for target pcl_debug
pcl_debug_EXTERNAL_OBJECTS =

pcl_debug: CMakeFiles/pcl_debug.dir/pcl_debug.cpp.o
pcl_debug: CMakeFiles/pcl_debug.dir/build.make
pcl_debug: CMakeFiles/pcl_debug.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/tingxfan/cpp_exp/study_pointcloud/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable pcl_debug"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/pcl_debug.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/pcl_debug.dir/build: pcl_debug

.PHONY : CMakeFiles/pcl_debug.dir/build

CMakeFiles/pcl_debug.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/pcl_debug.dir/cmake_clean.cmake
.PHONY : CMakeFiles/pcl_debug.dir/clean

CMakeFiles/pcl_debug.dir/depend:
	cd /home/tingxfan/cpp_exp/study_pointcloud/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/tingxfan/cpp_exp/study_pointcloud /home/tingxfan/cpp_exp/study_pointcloud /home/tingxfan/cpp_exp/study_pointcloud/build /home/tingxfan/cpp_exp/study_pointcloud/build /home/tingxfan/cpp_exp/study_pointcloud/build/CMakeFiles/pcl_debug.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/pcl_debug.dir/depend

