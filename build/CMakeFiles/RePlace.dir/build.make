# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.26

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
CMAKE_COMMAND = /home/bing/.local/lib/python3.8/site-packages/cmake/data/bin/cmake

# The command to remove a file.
RM = /home/bing/.local/lib/python3.8/site-packages/cmake/data/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/bing/yd/slam-test/RePlace

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/bing/yd/slam-test/RePlace/build

# Include any dependencies generated for this target.
include CMakeFiles/RePlace.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/RePlace.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/RePlace.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/RePlace.dir/flags.make

CMakeFiles/RePlace.dir/src/ORBextractor.cpp.o: CMakeFiles/RePlace.dir/flags.make
CMakeFiles/RePlace.dir/src/ORBextractor.cpp.o: /home/bing/yd/slam-test/RePlace/src/ORBextractor.cpp
CMakeFiles/RePlace.dir/src/ORBextractor.cpp.o: CMakeFiles/RePlace.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/bing/yd/slam-test/RePlace/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/RePlace.dir/src/ORBextractor.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/RePlace.dir/src/ORBextractor.cpp.o -MF CMakeFiles/RePlace.dir/src/ORBextractor.cpp.o.d -o CMakeFiles/RePlace.dir/src/ORBextractor.cpp.o -c /home/bing/yd/slam-test/RePlace/src/ORBextractor.cpp

CMakeFiles/RePlace.dir/src/ORBextractor.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/RePlace.dir/src/ORBextractor.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/bing/yd/slam-test/RePlace/src/ORBextractor.cpp > CMakeFiles/RePlace.dir/src/ORBextractor.cpp.i

CMakeFiles/RePlace.dir/src/ORBextractor.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/RePlace.dir/src/ORBextractor.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/bing/yd/slam-test/RePlace/src/ORBextractor.cpp -o CMakeFiles/RePlace.dir/src/ORBextractor.cpp.s

CMakeFiles/RePlace.dir/src/Frame.cpp.o: CMakeFiles/RePlace.dir/flags.make
CMakeFiles/RePlace.dir/src/Frame.cpp.o: /home/bing/yd/slam-test/RePlace/src/Frame.cpp
CMakeFiles/RePlace.dir/src/Frame.cpp.o: CMakeFiles/RePlace.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/bing/yd/slam-test/RePlace/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/RePlace.dir/src/Frame.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/RePlace.dir/src/Frame.cpp.o -MF CMakeFiles/RePlace.dir/src/Frame.cpp.o.d -o CMakeFiles/RePlace.dir/src/Frame.cpp.o -c /home/bing/yd/slam-test/RePlace/src/Frame.cpp

CMakeFiles/RePlace.dir/src/Frame.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/RePlace.dir/src/Frame.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/bing/yd/slam-test/RePlace/src/Frame.cpp > CMakeFiles/RePlace.dir/src/Frame.cpp.i

CMakeFiles/RePlace.dir/src/Frame.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/RePlace.dir/src/Frame.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/bing/yd/slam-test/RePlace/src/Frame.cpp -o CMakeFiles/RePlace.dir/src/Frame.cpp.s

CMakeFiles/RePlace.dir/src/ORBmatcher.cpp.o: CMakeFiles/RePlace.dir/flags.make
CMakeFiles/RePlace.dir/src/ORBmatcher.cpp.o: /home/bing/yd/slam-test/RePlace/src/ORBmatcher.cpp
CMakeFiles/RePlace.dir/src/ORBmatcher.cpp.o: CMakeFiles/RePlace.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/bing/yd/slam-test/RePlace/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/RePlace.dir/src/ORBmatcher.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/RePlace.dir/src/ORBmatcher.cpp.o -MF CMakeFiles/RePlace.dir/src/ORBmatcher.cpp.o.d -o CMakeFiles/RePlace.dir/src/ORBmatcher.cpp.o -c /home/bing/yd/slam-test/RePlace/src/ORBmatcher.cpp

CMakeFiles/RePlace.dir/src/ORBmatcher.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/RePlace.dir/src/ORBmatcher.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/bing/yd/slam-test/RePlace/src/ORBmatcher.cpp > CMakeFiles/RePlace.dir/src/ORBmatcher.cpp.i

CMakeFiles/RePlace.dir/src/ORBmatcher.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/RePlace.dir/src/ORBmatcher.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/bing/yd/slam-test/RePlace/src/ORBmatcher.cpp -o CMakeFiles/RePlace.dir/src/ORBmatcher.cpp.s

CMakeFiles/RePlace.dir/src/MapPoint.cpp.o: CMakeFiles/RePlace.dir/flags.make
CMakeFiles/RePlace.dir/src/MapPoint.cpp.o: /home/bing/yd/slam-test/RePlace/src/MapPoint.cpp
CMakeFiles/RePlace.dir/src/MapPoint.cpp.o: CMakeFiles/RePlace.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/bing/yd/slam-test/RePlace/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/RePlace.dir/src/MapPoint.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/RePlace.dir/src/MapPoint.cpp.o -MF CMakeFiles/RePlace.dir/src/MapPoint.cpp.o.d -o CMakeFiles/RePlace.dir/src/MapPoint.cpp.o -c /home/bing/yd/slam-test/RePlace/src/MapPoint.cpp

CMakeFiles/RePlace.dir/src/MapPoint.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/RePlace.dir/src/MapPoint.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/bing/yd/slam-test/RePlace/src/MapPoint.cpp > CMakeFiles/RePlace.dir/src/MapPoint.cpp.i

CMakeFiles/RePlace.dir/src/MapPoint.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/RePlace.dir/src/MapPoint.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/bing/yd/slam-test/RePlace/src/MapPoint.cpp -o CMakeFiles/RePlace.dir/src/MapPoint.cpp.s

CMakeFiles/RePlace.dir/src/Optimizer.cpp.o: CMakeFiles/RePlace.dir/flags.make
CMakeFiles/RePlace.dir/src/Optimizer.cpp.o: /home/bing/yd/slam-test/RePlace/src/Optimizer.cpp
CMakeFiles/RePlace.dir/src/Optimizer.cpp.o: CMakeFiles/RePlace.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/bing/yd/slam-test/RePlace/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/RePlace.dir/src/Optimizer.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/RePlace.dir/src/Optimizer.cpp.o -MF CMakeFiles/RePlace.dir/src/Optimizer.cpp.o.d -o CMakeFiles/RePlace.dir/src/Optimizer.cpp.o -c /home/bing/yd/slam-test/RePlace/src/Optimizer.cpp

CMakeFiles/RePlace.dir/src/Optimizer.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/RePlace.dir/src/Optimizer.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/bing/yd/slam-test/RePlace/src/Optimizer.cpp > CMakeFiles/RePlace.dir/src/Optimizer.cpp.i

CMakeFiles/RePlace.dir/src/Optimizer.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/RePlace.dir/src/Optimizer.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/bing/yd/slam-test/RePlace/src/Optimizer.cpp -o CMakeFiles/RePlace.dir/src/Optimizer.cpp.s

CMakeFiles/RePlace.dir/src/Converter.cpp.o: CMakeFiles/RePlace.dir/flags.make
CMakeFiles/RePlace.dir/src/Converter.cpp.o: /home/bing/yd/slam-test/RePlace/src/Converter.cpp
CMakeFiles/RePlace.dir/src/Converter.cpp.o: CMakeFiles/RePlace.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/bing/yd/slam-test/RePlace/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object CMakeFiles/RePlace.dir/src/Converter.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/RePlace.dir/src/Converter.cpp.o -MF CMakeFiles/RePlace.dir/src/Converter.cpp.o.d -o CMakeFiles/RePlace.dir/src/Converter.cpp.o -c /home/bing/yd/slam-test/RePlace/src/Converter.cpp

CMakeFiles/RePlace.dir/src/Converter.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/RePlace.dir/src/Converter.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/bing/yd/slam-test/RePlace/src/Converter.cpp > CMakeFiles/RePlace.dir/src/Converter.cpp.i

CMakeFiles/RePlace.dir/src/Converter.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/RePlace.dir/src/Converter.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/bing/yd/slam-test/RePlace/src/Converter.cpp -o CMakeFiles/RePlace.dir/src/Converter.cpp.s

# Object files for target RePlace
RePlace_OBJECTS = \
"CMakeFiles/RePlace.dir/src/ORBextractor.cpp.o" \
"CMakeFiles/RePlace.dir/src/Frame.cpp.o" \
"CMakeFiles/RePlace.dir/src/ORBmatcher.cpp.o" \
"CMakeFiles/RePlace.dir/src/MapPoint.cpp.o" \
"CMakeFiles/RePlace.dir/src/Optimizer.cpp.o" \
"CMakeFiles/RePlace.dir/src/Converter.cpp.o"

# External object files for target RePlace
RePlace_EXTERNAL_OBJECTS =

libRePlace.so: CMakeFiles/RePlace.dir/src/ORBextractor.cpp.o
libRePlace.so: CMakeFiles/RePlace.dir/src/Frame.cpp.o
libRePlace.so: CMakeFiles/RePlace.dir/src/ORBmatcher.cpp.o
libRePlace.so: CMakeFiles/RePlace.dir/src/MapPoint.cpp.o
libRePlace.so: CMakeFiles/RePlace.dir/src/Optimizer.cpp.o
libRePlace.so: CMakeFiles/RePlace.dir/src/Converter.cpp.o
libRePlace.so: CMakeFiles/RePlace.dir/build.make
libRePlace.so: CMakeFiles/RePlace.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/bing/yd/slam-test/RePlace/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Linking CXX shared library libRePlace.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/RePlace.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/RePlace.dir/build: libRePlace.so
.PHONY : CMakeFiles/RePlace.dir/build

CMakeFiles/RePlace.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/RePlace.dir/cmake_clean.cmake
.PHONY : CMakeFiles/RePlace.dir/clean

CMakeFiles/RePlace.dir/depend:
	cd /home/bing/yd/slam-test/RePlace/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/bing/yd/slam-test/RePlace /home/bing/yd/slam-test/RePlace /home/bing/yd/slam-test/RePlace/build /home/bing/yd/slam-test/RePlace/build /home/bing/yd/slam-test/RePlace/build/CMakeFiles/RePlace.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/RePlace.dir/depend

