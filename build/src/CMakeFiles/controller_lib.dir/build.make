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
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/steffen/EmbeddedHausarbeit

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/steffen/EmbeddedHausarbeit/build

# Include any dependencies generated for this target.
include src/CMakeFiles/controller_lib.dir/depend.make

# Include the progress variables for this target.
include src/CMakeFiles/controller_lib.dir/progress.make

# Include the compile flags for this target's objects.
include src/CMakeFiles/controller_lib.dir/flags.make

src/CMakeFiles/controller_lib.dir/controller.cpp.o: src/CMakeFiles/controller_lib.dir/flags.make
src/CMakeFiles/controller_lib.dir/controller.cpp.o: ../src/controller.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/steffen/EmbeddedHausarbeit/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/CMakeFiles/controller_lib.dir/controller.cpp.o"
	cd /home/steffen/EmbeddedHausarbeit/build/src && /usr/bin/arm-linux-gnueabihf-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/controller_lib.dir/controller.cpp.o -c /home/steffen/EmbeddedHausarbeit/src/controller.cpp

src/CMakeFiles/controller_lib.dir/controller.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/controller_lib.dir/controller.cpp.i"
	cd /home/steffen/EmbeddedHausarbeit/build/src && /usr/bin/arm-linux-gnueabihf-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/steffen/EmbeddedHausarbeit/src/controller.cpp > CMakeFiles/controller_lib.dir/controller.cpp.i

src/CMakeFiles/controller_lib.dir/controller.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/controller_lib.dir/controller.cpp.s"
	cd /home/steffen/EmbeddedHausarbeit/build/src && /usr/bin/arm-linux-gnueabihf-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/steffen/EmbeddedHausarbeit/src/controller.cpp -o CMakeFiles/controller_lib.dir/controller.cpp.s

# Object files for target controller_lib
controller_lib_OBJECTS = \
"CMakeFiles/controller_lib.dir/controller.cpp.o"

# External object files for target controller_lib
controller_lib_EXTERNAL_OBJECTS =

src/libcontroller_lib.a: src/CMakeFiles/controller_lib.dir/controller.cpp.o
src/libcontroller_lib.a: src/CMakeFiles/controller_lib.dir/build.make
src/libcontroller_lib.a: src/CMakeFiles/controller_lib.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/steffen/EmbeddedHausarbeit/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libcontroller_lib.a"
	cd /home/steffen/EmbeddedHausarbeit/build/src && $(CMAKE_COMMAND) -P CMakeFiles/controller_lib.dir/cmake_clean_target.cmake
	cd /home/steffen/EmbeddedHausarbeit/build/src && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/controller_lib.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/CMakeFiles/controller_lib.dir/build: src/libcontroller_lib.a

.PHONY : src/CMakeFiles/controller_lib.dir/build

src/CMakeFiles/controller_lib.dir/clean:
	cd /home/steffen/EmbeddedHausarbeit/build/src && $(CMAKE_COMMAND) -P CMakeFiles/controller_lib.dir/cmake_clean.cmake
.PHONY : src/CMakeFiles/controller_lib.dir/clean

src/CMakeFiles/controller_lib.dir/depend:
	cd /home/steffen/EmbeddedHausarbeit/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/steffen/EmbeddedHausarbeit /home/steffen/EmbeddedHausarbeit/src /home/steffen/EmbeddedHausarbeit/build /home/steffen/EmbeddedHausarbeit/build/src /home/steffen/EmbeddedHausarbeit/build/src/CMakeFiles/controller_lib.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/CMakeFiles/controller_lib.dir/depend
