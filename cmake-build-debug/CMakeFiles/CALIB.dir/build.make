# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.12

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
CMAKE_COMMAND = /home/liushiqi/clion-2018.2.1/bin/cmake/linux/bin/cmake

# The command to remove a file.
RM = /home/liushiqi/clion-2018.2.1/bin/cmake/linux/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/liushiqi/ClionProjects/zhang_calibration

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/liushiqi/ClionProjects/zhang_calibration/cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/CALIB.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/CALIB.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/CALIB.dir/flags.make

CMakeFiles/CALIB.dir/src/ZZYCalibration.cpp.o: CMakeFiles/CALIB.dir/flags.make
CMakeFiles/CALIB.dir/src/ZZYCalibration.cpp.o: ../src/ZZYCalibration.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/liushiqi/ClionProjects/zhang_calibration/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/CALIB.dir/src/ZZYCalibration.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/CALIB.dir/src/ZZYCalibration.cpp.o -c /home/liushiqi/ClionProjects/zhang_calibration/src/ZZYCalibration.cpp

CMakeFiles/CALIB.dir/src/ZZYCalibration.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/CALIB.dir/src/ZZYCalibration.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/liushiqi/ClionProjects/zhang_calibration/src/ZZYCalibration.cpp > CMakeFiles/CALIB.dir/src/ZZYCalibration.cpp.i

CMakeFiles/CALIB.dir/src/ZZYCalibration.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/CALIB.dir/src/ZZYCalibration.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/liushiqi/ClionProjects/zhang_calibration/src/ZZYCalibration.cpp -o CMakeFiles/CALIB.dir/src/ZZYCalibration.cpp.s

# Object files for target CALIB
CALIB_OBJECTS = \
"CMakeFiles/CALIB.dir/src/ZZYCalibration.cpp.o"

# External object files for target CALIB
CALIB_EXTERNAL_OBJECTS =

../lib/libCALIB.so: CMakeFiles/CALIB.dir/src/ZZYCalibration.cpp.o
../lib/libCALIB.so: CMakeFiles/CALIB.dir/build.make
../lib/libCALIB.so: CMakeFiles/CALIB.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/liushiqi/ClionProjects/zhang_calibration/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library ../lib/libCALIB.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/CALIB.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/CALIB.dir/build: ../lib/libCALIB.so

.PHONY : CMakeFiles/CALIB.dir/build

CMakeFiles/CALIB.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/CALIB.dir/cmake_clean.cmake
.PHONY : CMakeFiles/CALIB.dir/clean

CMakeFiles/CALIB.dir/depend:
	cd /home/liushiqi/ClionProjects/zhang_calibration/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/liushiqi/ClionProjects/zhang_calibration /home/liushiqi/ClionProjects/zhang_calibration /home/liushiqi/ClionProjects/zhang_calibration/cmake-build-debug /home/liushiqi/ClionProjects/zhang_calibration/cmake-build-debug /home/liushiqi/ClionProjects/zhang_calibration/cmake-build-debug/CMakeFiles/CALIB.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/CALIB.dir/depend

