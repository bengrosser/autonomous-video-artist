# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.6

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
CMAKE_COMMAND = /Applications/CLion.app/Contents/bin/cmake/bin/cmake

# The command to remove a file.
RM = /Applications/CLion.app/Contents/bin/cmake/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /Users/frankshammer42/Documents/CS/autonomous-video-artist/visual/exp

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/frankshammer42/Documents/CS/autonomous-video-artist/visual/exp/cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/exp.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/exp.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/exp.dir/flags.make

CMakeFiles/exp.dir/preliminary_analysis.cpp.o: CMakeFiles/exp.dir/flags.make
CMakeFiles/exp.dir/preliminary_analysis.cpp.o: ../preliminary_analysis.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/frankshammer42/Documents/CS/autonomous-video-artist/visual/exp/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/exp.dir/preliminary_analysis.cpp.o"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/exp.dir/preliminary_analysis.cpp.o -c /Users/frankshammer42/Documents/CS/autonomous-video-artist/visual/exp/preliminary_analysis.cpp

CMakeFiles/exp.dir/preliminary_analysis.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/exp.dir/preliminary_analysis.cpp.i"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/frankshammer42/Documents/CS/autonomous-video-artist/visual/exp/preliminary_analysis.cpp > CMakeFiles/exp.dir/preliminary_analysis.cpp.i

CMakeFiles/exp.dir/preliminary_analysis.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/exp.dir/preliminary_analysis.cpp.s"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/frankshammer42/Documents/CS/autonomous-video-artist/visual/exp/preliminary_analysis.cpp -o CMakeFiles/exp.dir/preliminary_analysis.cpp.s

CMakeFiles/exp.dir/preliminary_analysis.cpp.o.requires:

.PHONY : CMakeFiles/exp.dir/preliminary_analysis.cpp.o.requires

CMakeFiles/exp.dir/preliminary_analysis.cpp.o.provides: CMakeFiles/exp.dir/preliminary_analysis.cpp.o.requires
	$(MAKE) -f CMakeFiles/exp.dir/build.make CMakeFiles/exp.dir/preliminary_analysis.cpp.o.provides.build
.PHONY : CMakeFiles/exp.dir/preliminary_analysis.cpp.o.provides

CMakeFiles/exp.dir/preliminary_analysis.cpp.o.provides.build: CMakeFiles/exp.dir/preliminary_analysis.cpp.o


# Object files for target exp
exp_OBJECTS = \
"CMakeFiles/exp.dir/preliminary_analysis.cpp.o"

# External object files for target exp
exp_EXTERNAL_OBJECTS =

exp: CMakeFiles/exp.dir/preliminary_analysis.cpp.o
exp: CMakeFiles/exp.dir/build.make
exp: CMakeFiles/exp.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/Users/frankshammer42/Documents/CS/autonomous-video-artist/visual/exp/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable exp"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/exp.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/exp.dir/build: exp

.PHONY : CMakeFiles/exp.dir/build

CMakeFiles/exp.dir/requires: CMakeFiles/exp.dir/preliminary_analysis.cpp.o.requires

.PHONY : CMakeFiles/exp.dir/requires

CMakeFiles/exp.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/exp.dir/cmake_clean.cmake
.PHONY : CMakeFiles/exp.dir/clean

CMakeFiles/exp.dir/depend:
	cd /Users/frankshammer42/Documents/CS/autonomous-video-artist/visual/exp/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/frankshammer42/Documents/CS/autonomous-video-artist/visual/exp /Users/frankshammer42/Documents/CS/autonomous-video-artist/visual/exp /Users/frankshammer42/Documents/CS/autonomous-video-artist/visual/exp/cmake-build-debug /Users/frankshammer42/Documents/CS/autonomous-video-artist/visual/exp/cmake-build-debug /Users/frankshammer42/Documents/CS/autonomous-video-artist/visual/exp/cmake-build-debug/CMakeFiles/exp.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/exp.dir/depend

