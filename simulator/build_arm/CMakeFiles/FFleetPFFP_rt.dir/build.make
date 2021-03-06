# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.7

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
CMAKE_COMMAND = /opt/robomap3/1.7.3/core2-64/sysroots/x86_64-pokysdk-linux/usr/bin/cmake

# The command to remove a file.
RM = /opt/robomap3/1.7.3/core2-64/sysroots/x86_64-pokysdk-linux/usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/ppalmeros/flair/ppalmeros/FFormation_04/simulator

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ppalmeros/flair/ppalmeros/FFormation_04/simulator/build_arm

# Include any dependencies generated for this target.
include CMakeFiles/FFleetPFFP_rt.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/FFleetPFFP_rt.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/FFleetPFFP_rt.dir/flags.make

CMakeFiles/FFleetPFFP_rt.dir/src/mainSimulatorx4.cpp.o: CMakeFiles/FFleetPFFP_rt.dir/flags.make
CMakeFiles/FFleetPFFP_rt.dir/src/mainSimulatorx4.cpp.o: ../src/mainSimulatorx4.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ppalmeros/flair/ppalmeros/FFormation_04/simulator/build_arm/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/FFleetPFFP_rt.dir/src/mainSimulatorx4.cpp.o"
	/opt/robomap3/1.7.3/armv7a-neon/sysroots/x86_64-pokysdk-linux/usr/bin/arm-poky-linux-gnueabi/arm-poky-linux-gnueabi-g++  --sysroot=/opt/robomap3/1.7.3/armv7a-neon/sysroots/armv7a-vfp-neon-poky-linux-gnueabi  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/FFleetPFFP_rt.dir/src/mainSimulatorx4.cpp.o -c /home/ppalmeros/flair/ppalmeros/FFormation_04/simulator/src/mainSimulatorx4.cpp

CMakeFiles/FFleetPFFP_rt.dir/src/mainSimulatorx4.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/FFleetPFFP_rt.dir/src/mainSimulatorx4.cpp.i"
	/opt/robomap3/1.7.3/armv7a-neon/sysroots/x86_64-pokysdk-linux/usr/bin/arm-poky-linux-gnueabi/arm-poky-linux-gnueabi-g++  --sysroot=/opt/robomap3/1.7.3/armv7a-neon/sysroots/armv7a-vfp-neon-poky-linux-gnueabi $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ppalmeros/flair/ppalmeros/FFormation_04/simulator/src/mainSimulatorx4.cpp > CMakeFiles/FFleetPFFP_rt.dir/src/mainSimulatorx4.cpp.i

CMakeFiles/FFleetPFFP_rt.dir/src/mainSimulatorx4.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/FFleetPFFP_rt.dir/src/mainSimulatorx4.cpp.s"
	/opt/robomap3/1.7.3/armv7a-neon/sysroots/x86_64-pokysdk-linux/usr/bin/arm-poky-linux-gnueabi/arm-poky-linux-gnueabi-g++  --sysroot=/opt/robomap3/1.7.3/armv7a-neon/sysroots/armv7a-vfp-neon-poky-linux-gnueabi $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ppalmeros/flair/ppalmeros/FFormation_04/simulator/src/mainSimulatorx4.cpp -o CMakeFiles/FFleetPFFP_rt.dir/src/mainSimulatorx4.cpp.s

CMakeFiles/FFleetPFFP_rt.dir/src/mainSimulatorx4.cpp.o.requires:

.PHONY : CMakeFiles/FFleetPFFP_rt.dir/src/mainSimulatorx4.cpp.o.requires

CMakeFiles/FFleetPFFP_rt.dir/src/mainSimulatorx4.cpp.o.provides: CMakeFiles/FFleetPFFP_rt.dir/src/mainSimulatorx4.cpp.o.requires
	$(MAKE) -f CMakeFiles/FFleetPFFP_rt.dir/build.make CMakeFiles/FFleetPFFP_rt.dir/src/mainSimulatorx4.cpp.o.provides.build
.PHONY : CMakeFiles/FFleetPFFP_rt.dir/src/mainSimulatorx4.cpp.o.provides

CMakeFiles/FFleetPFFP_rt.dir/src/mainSimulatorx4.cpp.o.provides.build: CMakeFiles/FFleetPFFP_rt.dir/src/mainSimulatorx4.cpp.o


# Object files for target FFleetPFFP_rt
FFleetPFFP_rt_OBJECTS = \
"CMakeFiles/FFleetPFFP_rt.dir/src/mainSimulatorx4.cpp.o"

# External object files for target FFleetPFFP_rt
FFleetPFFP_rt_EXTERNAL_OBJECTS =

bin/FFleetPFFP_rt: CMakeFiles/FFleetPFFP_rt.dir/src/mainSimulatorx4.cpp.o
bin/FFleetPFFP_rt: CMakeFiles/FFleetPFFP_rt.dir/build.make
bin/FFleetPFFP_rt: /home/ppalmeros/flair/flair-bin/lib/arm/libFlairSimulator_nogl.a
bin/FFleetPFFP_rt: /home/ppalmeros/flair/flair-bin/lib/arm/libFlairSensorActuator.a
bin/FFleetPFFP_rt: /home/ppalmeros/flair/flair-bin/lib/arm/libFlairCore_rt.a
bin/FFleetPFFP_rt: /opt/robomap3/1.7.3/armv7a-neon/sysroots/armv7a-vfp-neon-poky-linux-gnueabi/usr/lib/libxml2.so
bin/FFleetPFFP_rt: /opt/robomap3/1.7.3/armv7a-neon/sysroots/armv7a-vfp-neon-poky-linux-gnueabi/usr/lib/librtdm.so
bin/FFleetPFFP_rt: /opt/robomap3/1.7.3/armv7a-neon/sysroots/armv7a-vfp-neon-poky-linux-gnueabi/usr/lib/libnative.so
bin/FFleetPFFP_rt: /opt/robomap3/1.7.3/armv7a-neon/sysroots/armv7a-vfp-neon-poky-linux-gnueabi/usr/lib/libxenomai.so
bin/FFleetPFFP_rt: CMakeFiles/FFleetPFFP_rt.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/ppalmeros/flair/ppalmeros/FFormation_04/simulator/build_arm/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable bin/FFleetPFFP_rt"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/FFleetPFFP_rt.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/FFleetPFFP_rt.dir/build: bin/FFleetPFFP_rt

.PHONY : CMakeFiles/FFleetPFFP_rt.dir/build

CMakeFiles/FFleetPFFP_rt.dir/requires: CMakeFiles/FFleetPFFP_rt.dir/src/mainSimulatorx4.cpp.o.requires

.PHONY : CMakeFiles/FFleetPFFP_rt.dir/requires

CMakeFiles/FFleetPFFP_rt.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/FFleetPFFP_rt.dir/cmake_clean.cmake
.PHONY : CMakeFiles/FFleetPFFP_rt.dir/clean

CMakeFiles/FFleetPFFP_rt.dir/depend:
	cd /home/ppalmeros/flair/ppalmeros/FFormation_04/simulator/build_arm && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ppalmeros/flair/ppalmeros/FFormation_04/simulator /home/ppalmeros/flair/ppalmeros/FFormation_04/simulator /home/ppalmeros/flair/ppalmeros/FFormation_04/simulator/build_arm /home/ppalmeros/flair/ppalmeros/FFormation_04/simulator/build_arm /home/ppalmeros/flair/ppalmeros/FFormation_04/simulator/build_arm/CMakeFiles/FFleetPFFP_rt.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/FFleetPFFP_rt.dir/depend

