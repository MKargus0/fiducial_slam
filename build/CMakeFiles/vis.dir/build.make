# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.18

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
CMAKE_COMMAND = /usr/local/lib/python3.6/dist-packages/cmake/data/bin/cmake

# The command to remove a file.
RM = /usr/local/lib/python3.6/dist-packages/cmake/data/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/argus/visual_navigation

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/argus/visual_navigation/build

# Include any dependencies generated for this target.
include CMakeFiles/vis.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/vis.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/vis.dir/flags.make

CMakeFiles/vis.dir/src/vision_systems.cpp.o: CMakeFiles/vis.dir/flags.make
CMakeFiles/vis.dir/src/vision_systems.cpp.o: ../src/vision_systems.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/argus/visual_navigation/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/vis.dir/src/vision_systems.cpp.o"
	/usr/bin/x86_64-linux-gnu-g++-7 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/vis.dir/src/vision_systems.cpp.o -c /home/argus/visual_navigation/src/vision_systems.cpp

CMakeFiles/vis.dir/src/vision_systems.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/vis.dir/src/vision_systems.cpp.i"
	/usr/bin/x86_64-linux-gnu-g++-7 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/argus/visual_navigation/src/vision_systems.cpp > CMakeFiles/vis.dir/src/vision_systems.cpp.i

CMakeFiles/vis.dir/src/vision_systems.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/vis.dir/src/vision_systems.cpp.s"
	/usr/bin/x86_64-linux-gnu-g++-7 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/argus/visual_navigation/src/vision_systems.cpp -o CMakeFiles/vis.dir/src/vision_systems.cpp.s

# Object files for target vis
vis_OBJECTS = \
"CMakeFiles/vis.dir/src/vision_systems.cpp.o"

# External object files for target vis
vis_EXTERNAL_OBJECTS =

libvis.so: CMakeFiles/vis.dir/src/vision_systems.cpp.o
libvis.so: CMakeFiles/vis.dir/build.make
libvis.so: CMakeFiles/vis.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/argus/visual_navigation/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library libvis.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/vis.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/vis.dir/build: libvis.so

.PHONY : CMakeFiles/vis.dir/build

CMakeFiles/vis.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/vis.dir/cmake_clean.cmake
.PHONY : CMakeFiles/vis.dir/clean

CMakeFiles/vis.dir/depend:
	cd /home/argus/visual_navigation/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/argus/visual_navigation /home/argus/visual_navigation /home/argus/visual_navigation/build /home/argus/visual_navigation/build /home/argus/visual_navigation/build/CMakeFiles/vis.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/vis.dir/depend
