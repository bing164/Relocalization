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
CMAKE_SOURCE_DIR = /home/bing/yd/slam-test/Relocalization

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/bing/yd/slam-test/Relocalization/build

# Include any dependencies generated for this target.
include CMakeFiles/Relocalization.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/Relocalization.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/Relocalization.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/Relocalization.dir/flags.make

CMakeFiles/Relocalization.dir/src/ORBextractor.cpp.o: CMakeFiles/Relocalization.dir/flags.make
CMakeFiles/Relocalization.dir/src/ORBextractor.cpp.o: /home/bing/yd/slam-test/Relocalization/src/ORBextractor.cpp
CMakeFiles/Relocalization.dir/src/ORBextractor.cpp.o: CMakeFiles/Relocalization.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/bing/yd/slam-test/Relocalization/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/Relocalization.dir/src/ORBextractor.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/Relocalization.dir/src/ORBextractor.cpp.o -MF CMakeFiles/Relocalization.dir/src/ORBextractor.cpp.o.d -o CMakeFiles/Relocalization.dir/src/ORBextractor.cpp.o -c /home/bing/yd/slam-test/Relocalization/src/ORBextractor.cpp

CMakeFiles/Relocalization.dir/src/ORBextractor.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Relocalization.dir/src/ORBextractor.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/bing/yd/slam-test/Relocalization/src/ORBextractor.cpp > CMakeFiles/Relocalization.dir/src/ORBextractor.cpp.i

CMakeFiles/Relocalization.dir/src/ORBextractor.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Relocalization.dir/src/ORBextractor.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/bing/yd/slam-test/Relocalization/src/ORBextractor.cpp -o CMakeFiles/Relocalization.dir/src/ORBextractor.cpp.s

CMakeFiles/Relocalization.dir/src/Frame.cpp.o: CMakeFiles/Relocalization.dir/flags.make
CMakeFiles/Relocalization.dir/src/Frame.cpp.o: /home/bing/yd/slam-test/Relocalization/src/Frame.cpp
CMakeFiles/Relocalization.dir/src/Frame.cpp.o: CMakeFiles/Relocalization.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/bing/yd/slam-test/Relocalization/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/Relocalization.dir/src/Frame.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/Relocalization.dir/src/Frame.cpp.o -MF CMakeFiles/Relocalization.dir/src/Frame.cpp.o.d -o CMakeFiles/Relocalization.dir/src/Frame.cpp.o -c /home/bing/yd/slam-test/Relocalization/src/Frame.cpp

CMakeFiles/Relocalization.dir/src/Frame.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Relocalization.dir/src/Frame.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/bing/yd/slam-test/Relocalization/src/Frame.cpp > CMakeFiles/Relocalization.dir/src/Frame.cpp.i

CMakeFiles/Relocalization.dir/src/Frame.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Relocalization.dir/src/Frame.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/bing/yd/slam-test/Relocalization/src/Frame.cpp -o CMakeFiles/Relocalization.dir/src/Frame.cpp.s

CMakeFiles/Relocalization.dir/src/ORBmatcher.cpp.o: CMakeFiles/Relocalization.dir/flags.make
CMakeFiles/Relocalization.dir/src/ORBmatcher.cpp.o: /home/bing/yd/slam-test/Relocalization/src/ORBmatcher.cpp
CMakeFiles/Relocalization.dir/src/ORBmatcher.cpp.o: CMakeFiles/Relocalization.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/bing/yd/slam-test/Relocalization/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/Relocalization.dir/src/ORBmatcher.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/Relocalization.dir/src/ORBmatcher.cpp.o -MF CMakeFiles/Relocalization.dir/src/ORBmatcher.cpp.o.d -o CMakeFiles/Relocalization.dir/src/ORBmatcher.cpp.o -c /home/bing/yd/slam-test/Relocalization/src/ORBmatcher.cpp

CMakeFiles/Relocalization.dir/src/ORBmatcher.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Relocalization.dir/src/ORBmatcher.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/bing/yd/slam-test/Relocalization/src/ORBmatcher.cpp > CMakeFiles/Relocalization.dir/src/ORBmatcher.cpp.i

CMakeFiles/Relocalization.dir/src/ORBmatcher.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Relocalization.dir/src/ORBmatcher.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/bing/yd/slam-test/Relocalization/src/ORBmatcher.cpp -o CMakeFiles/Relocalization.dir/src/ORBmatcher.cpp.s

CMakeFiles/Relocalization.dir/src/MapPoint.cpp.o: CMakeFiles/Relocalization.dir/flags.make
CMakeFiles/Relocalization.dir/src/MapPoint.cpp.o: /home/bing/yd/slam-test/Relocalization/src/MapPoint.cpp
CMakeFiles/Relocalization.dir/src/MapPoint.cpp.o: CMakeFiles/Relocalization.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/bing/yd/slam-test/Relocalization/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/Relocalization.dir/src/MapPoint.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/Relocalization.dir/src/MapPoint.cpp.o -MF CMakeFiles/Relocalization.dir/src/MapPoint.cpp.o.d -o CMakeFiles/Relocalization.dir/src/MapPoint.cpp.o -c /home/bing/yd/slam-test/Relocalization/src/MapPoint.cpp

CMakeFiles/Relocalization.dir/src/MapPoint.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Relocalization.dir/src/MapPoint.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/bing/yd/slam-test/Relocalization/src/MapPoint.cpp > CMakeFiles/Relocalization.dir/src/MapPoint.cpp.i

CMakeFiles/Relocalization.dir/src/MapPoint.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Relocalization.dir/src/MapPoint.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/bing/yd/slam-test/Relocalization/src/MapPoint.cpp -o CMakeFiles/Relocalization.dir/src/MapPoint.cpp.s

CMakeFiles/Relocalization.dir/src/Optimizer.cpp.o: CMakeFiles/Relocalization.dir/flags.make
CMakeFiles/Relocalization.dir/src/Optimizer.cpp.o: /home/bing/yd/slam-test/Relocalization/src/Optimizer.cpp
CMakeFiles/Relocalization.dir/src/Optimizer.cpp.o: CMakeFiles/Relocalization.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/bing/yd/slam-test/Relocalization/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/Relocalization.dir/src/Optimizer.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/Relocalization.dir/src/Optimizer.cpp.o -MF CMakeFiles/Relocalization.dir/src/Optimizer.cpp.o.d -o CMakeFiles/Relocalization.dir/src/Optimizer.cpp.o -c /home/bing/yd/slam-test/Relocalization/src/Optimizer.cpp

CMakeFiles/Relocalization.dir/src/Optimizer.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Relocalization.dir/src/Optimizer.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/bing/yd/slam-test/Relocalization/src/Optimizer.cpp > CMakeFiles/Relocalization.dir/src/Optimizer.cpp.i

CMakeFiles/Relocalization.dir/src/Optimizer.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Relocalization.dir/src/Optimizer.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/bing/yd/slam-test/Relocalization/src/Optimizer.cpp -o CMakeFiles/Relocalization.dir/src/Optimizer.cpp.s

CMakeFiles/Relocalization.dir/src/Converter.cpp.o: CMakeFiles/Relocalization.dir/flags.make
CMakeFiles/Relocalization.dir/src/Converter.cpp.o: /home/bing/yd/slam-test/Relocalization/src/Converter.cpp
CMakeFiles/Relocalization.dir/src/Converter.cpp.o: CMakeFiles/Relocalization.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/bing/yd/slam-test/Relocalization/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object CMakeFiles/Relocalization.dir/src/Converter.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/Relocalization.dir/src/Converter.cpp.o -MF CMakeFiles/Relocalization.dir/src/Converter.cpp.o.d -o CMakeFiles/Relocalization.dir/src/Converter.cpp.o -c /home/bing/yd/slam-test/Relocalization/src/Converter.cpp

CMakeFiles/Relocalization.dir/src/Converter.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Relocalization.dir/src/Converter.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/bing/yd/slam-test/Relocalization/src/Converter.cpp > CMakeFiles/Relocalization.dir/src/Converter.cpp.i

CMakeFiles/Relocalization.dir/src/Converter.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Relocalization.dir/src/Converter.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/bing/yd/slam-test/Relocalization/src/Converter.cpp -o CMakeFiles/Relocalization.dir/src/Converter.cpp.s

# Object files for target Relocalization
Relocalization_OBJECTS = \
"CMakeFiles/Relocalization.dir/src/ORBextractor.cpp.o" \
"CMakeFiles/Relocalization.dir/src/Frame.cpp.o" \
"CMakeFiles/Relocalization.dir/src/ORBmatcher.cpp.o" \
"CMakeFiles/Relocalization.dir/src/MapPoint.cpp.o" \
"CMakeFiles/Relocalization.dir/src/Optimizer.cpp.o" \
"CMakeFiles/Relocalization.dir/src/Converter.cpp.o"

# External object files for target Relocalization
Relocalization_EXTERNAL_OBJECTS =

libRelocalization.so: CMakeFiles/Relocalization.dir/src/ORBextractor.cpp.o
libRelocalization.so: CMakeFiles/Relocalization.dir/src/Frame.cpp.o
libRelocalization.so: CMakeFiles/Relocalization.dir/src/ORBmatcher.cpp.o
libRelocalization.so: CMakeFiles/Relocalization.dir/src/MapPoint.cpp.o
libRelocalization.so: CMakeFiles/Relocalization.dir/src/Optimizer.cpp.o
libRelocalization.so: CMakeFiles/Relocalization.dir/src/Converter.cpp.o
libRelocalization.so: CMakeFiles/Relocalization.dir/build.make
libRelocalization.so: CMakeFiles/Relocalization.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/bing/yd/slam-test/Relocalization/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Linking CXX shared library libRelocalization.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/Relocalization.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/Relocalization.dir/build: libRelocalization.so
.PHONY : CMakeFiles/Relocalization.dir/build

CMakeFiles/Relocalization.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/Relocalization.dir/cmake_clean.cmake
.PHONY : CMakeFiles/Relocalization.dir/clean

CMakeFiles/Relocalization.dir/depend:
	cd /home/bing/yd/slam-test/Relocalization/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/bing/yd/slam-test/Relocalization /home/bing/yd/slam-test/Relocalization /home/bing/yd/slam-test/Relocalization/build /home/bing/yd/slam-test/Relocalization/build /home/bing/yd/slam-test/Relocalization/build/CMakeFiles/Relocalization.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/Relocalization.dir/depend
