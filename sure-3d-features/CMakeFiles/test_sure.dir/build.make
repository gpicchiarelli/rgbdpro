# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list

# Produce verbose output by default.
VERBOSE = 1

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
CMAKE_SOURCE_DIR = /home/giacomo/sviluppo/LoopCloserRGBD/sure-3d-features

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/giacomo/sviluppo/LoopCloserRGBD/sure-3d-features

# Include any dependencies generated for this target.
include CMakeFiles/test_sure.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/test_sure.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/test_sure.dir/flags.make

CMakeFiles/test_sure.dir/src/test_sure.cpp.o: CMakeFiles/test_sure.dir/flags.make
CMakeFiles/test_sure.dir/src/test_sure.cpp.o: src/test_sure.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/giacomo/sviluppo/LoopCloserRGBD/sure-3d-features/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/test_sure.dir/src/test_sure.cpp.o"
	/usr/bin/ccache  g++  $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/test_sure.dir/src/test_sure.cpp.o -c /home/giacomo/sviluppo/LoopCloserRGBD/sure-3d-features/src/test_sure.cpp

CMakeFiles/test_sure.dir/src/test_sure.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test_sure.dir/src/test_sure.cpp.i"
	/usr/bin/ccache  g++ $(CXX_DEFINES) $(CXX_FLAGS) -E /home/giacomo/sviluppo/LoopCloserRGBD/sure-3d-features/src/test_sure.cpp > CMakeFiles/test_sure.dir/src/test_sure.cpp.i

CMakeFiles/test_sure.dir/src/test_sure.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test_sure.dir/src/test_sure.cpp.s"
	/usr/bin/ccache  g++ $(CXX_DEFINES) $(CXX_FLAGS) -S /home/giacomo/sviluppo/LoopCloserRGBD/sure-3d-features/src/test_sure.cpp -o CMakeFiles/test_sure.dir/src/test_sure.cpp.s

CMakeFiles/test_sure.dir/src/test_sure.cpp.o.requires:
.PHONY : CMakeFiles/test_sure.dir/src/test_sure.cpp.o.requires

CMakeFiles/test_sure.dir/src/test_sure.cpp.o.provides: CMakeFiles/test_sure.dir/src/test_sure.cpp.o.requires
	$(MAKE) -f CMakeFiles/test_sure.dir/build.make CMakeFiles/test_sure.dir/src/test_sure.cpp.o.provides.build
.PHONY : CMakeFiles/test_sure.dir/src/test_sure.cpp.o.provides

CMakeFiles/test_sure.dir/src/test_sure.cpp.o.provides.build: CMakeFiles/test_sure.dir/src/test_sure.cpp.o

# Object files for target test_sure
test_sure_OBJECTS = \
"CMakeFiles/test_sure.dir/src/test_sure.cpp.o"

# External object files for target test_sure
test_sure_EXTERNAL_OBJECTS =

bin/test_sure: CMakeFiles/test_sure.dir/src/test_sure.cpp.o
bin/test_sure: CMakeFiles/test_sure.dir/build.make
bin/test_sure: lib/libsure3d.so.0.2.7
bin/test_sure: /usr/lib/libboost_system-mt.so
bin/test_sure: /usr/lib/libboost_filesystem-mt.so
bin/test_sure: /usr/lib/libboost_thread-mt.so
bin/test_sure: /usr/lib/libboost_date_time-mt.so
bin/test_sure: /usr/lib/libboost_iostreams-mt.so
bin/test_sure: /usr/lib/libboost_serialization-mt.so
bin/test_sure: /usr/local/lib/libpcl_common.so
bin/test_sure: /usr/lib/libflann_cpp_s.a
bin/test_sure: /usr/local/lib/libpcl_kdtree.so
bin/test_sure: /usr/local/lib/libpcl_octree.so
bin/test_sure: /usr/local/lib/libpcl_search.so
bin/test_sure: /usr/local/lib/libpcl_sample_consensus.so
bin/test_sure: /usr/local/lib/libpcl_filters.so
bin/test_sure: /usr/lib/libOpenNI.so
bin/test_sure: /usr/lib/libvtkCommon.so.5.8.0
bin/test_sure: /usr/lib/libvtkRendering.so.5.8.0
bin/test_sure: /usr/lib/libvtkHybrid.so.5.8.0
bin/test_sure: /usr/local/lib/libpcl_io.so
bin/test_sure: /usr/local/lib/libpcl_features.so
bin/test_sure: /usr/local/lib/libpcl_keypoints.so
bin/test_sure: /usr/lib/libqhull.so
bin/test_sure: /usr/local/lib/libpcl_surface.so
bin/test_sure: /usr/local/lib/libpcl_registration.so
bin/test_sure: /usr/local/lib/libpcl_ml.so
bin/test_sure: /usr/local/lib/libpcl_segmentation.so
bin/test_sure: /usr/local/lib/libpcl_recognition.so
bin/test_sure: /usr/local/lib/libpcl_visualization.so
bin/test_sure: /usr/local/lib/libpcl_outofcore.so
bin/test_sure: /usr/local/lib/libpcl_tracking.so
bin/test_sure: /usr/local/lib/libpcl_stereo.so
bin/test_sure: /usr/lib/libvtkParallel.so.5.8.0
bin/test_sure: /usr/lib/libvtkRendering.so.5.8.0
bin/test_sure: /usr/lib/libvtkGraphics.so.5.8.0
bin/test_sure: /usr/lib/libvtkImaging.so.5.8.0
bin/test_sure: /usr/lib/libvtkIO.so.5.8.0
bin/test_sure: /usr/lib/libvtkFiltering.so.5.8.0
bin/test_sure: /usr/lib/libvtkCommon.so.5.8.0
bin/test_sure: /usr/lib/libvtksys.so.5.8.0
bin/test_sure: CMakeFiles/test_sure.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable bin/test_sure"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/test_sure.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/test_sure.dir/build: bin/test_sure
.PHONY : CMakeFiles/test_sure.dir/build

CMakeFiles/test_sure.dir/requires: CMakeFiles/test_sure.dir/src/test_sure.cpp.o.requires
.PHONY : CMakeFiles/test_sure.dir/requires

CMakeFiles/test_sure.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/test_sure.dir/cmake_clean.cmake
.PHONY : CMakeFiles/test_sure.dir/clean

CMakeFiles/test_sure.dir/depend:
	cd /home/giacomo/sviluppo/LoopCloserRGBD/sure-3d-features && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/giacomo/sviluppo/LoopCloserRGBD/sure-3d-features /home/giacomo/sviluppo/LoopCloserRGBD/sure-3d-features /home/giacomo/sviluppo/LoopCloserRGBD/sure-3d-features /home/giacomo/sviluppo/LoopCloserRGBD/sure-3d-features /home/giacomo/sviluppo/LoopCloserRGBD/sure-3d-features/CMakeFiles/test_sure.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/test_sure.dir/depend
