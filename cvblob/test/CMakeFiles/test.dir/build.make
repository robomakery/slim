# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

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
CMAKE_SOURCE_DIR = /home/robomakery/Code/slim/cvblob

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/robomakery/Code/slim/cvblob

# Include any dependencies generated for this target.
include test/CMakeFiles/test.dir/depend.make

# Include the progress variables for this target.
include test/CMakeFiles/test.dir/progress.make

# Include the compile flags for this target's objects.
include test/CMakeFiles/test.dir/flags.make

test/CMakeFiles/test.dir/test.cpp.o: test/CMakeFiles/test.dir/flags.make
test/CMakeFiles/test.dir/test.cpp.o: test/test.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/robomakery/Code/slim/cvblob/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object test/CMakeFiles/test.dir/test.cpp.o"
	cd /home/robomakery/Code/slim/cvblob/test && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -O3 -o CMakeFiles/test.dir/test.cpp.o -c /home/robomakery/Code/slim/cvblob/test/test.cpp

test/CMakeFiles/test.dir/test.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test.dir/test.cpp.i"
	cd /home/robomakery/Code/slim/cvblob/test && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -O3 -E /home/robomakery/Code/slim/cvblob/test/test.cpp > CMakeFiles/test.dir/test.cpp.i

test/CMakeFiles/test.dir/test.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test.dir/test.cpp.s"
	cd /home/robomakery/Code/slim/cvblob/test && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -O3 -S /home/robomakery/Code/slim/cvblob/test/test.cpp -o CMakeFiles/test.dir/test.cpp.s

test/CMakeFiles/test.dir/test.cpp.o.requires:
.PHONY : test/CMakeFiles/test.dir/test.cpp.o.requires

test/CMakeFiles/test.dir/test.cpp.o.provides: test/CMakeFiles/test.dir/test.cpp.o.requires
	$(MAKE) -f test/CMakeFiles/test.dir/build.make test/CMakeFiles/test.dir/test.cpp.o.provides.build
.PHONY : test/CMakeFiles/test.dir/test.cpp.o.provides

test/CMakeFiles/test.dir/test.cpp.o.provides.build: test/CMakeFiles/test.dir/test.cpp.o

# Object files for target test
test_OBJECTS = \
"CMakeFiles/test.dir/test.cpp.o"

# External object files for target test
test_EXTERNAL_OBJECTS =

bin/test: test/CMakeFiles/test.dir/test.cpp.o
bin/test: test/CMakeFiles/test.dir/build.make
bin/test: lib/libcvblob.so
bin/test: /usr/lib/x86_64-linux-gnu/libopencv_videostab.so.2.4.8
bin/test: /usr/lib/x86_64-linux-gnu/libopencv_video.so.2.4.8
bin/test: /usr/lib/x86_64-linux-gnu/libopencv_ts.so.2.4.8
bin/test: /usr/lib/x86_64-linux-gnu/libopencv_superres.so.2.4.8
bin/test: /usr/lib/x86_64-linux-gnu/libopencv_stitching.so.2.4.8
bin/test: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.2.4.8
bin/test: /usr/lib/x86_64-linux-gnu/libopencv_ocl.so.2.4.8
bin/test: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.2.4.8
bin/test: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.2.4.8
bin/test: /usr/lib/x86_64-linux-gnu/libopencv_legacy.so.2.4.8
bin/test: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.2.4.8
bin/test: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.2.4.8
bin/test: /usr/lib/x86_64-linux-gnu/libopencv_gpu.so.2.4.8
bin/test: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.2.4.8
bin/test: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.2.4.8
bin/test: /usr/lib/x86_64-linux-gnu/libopencv_core.so.2.4.8
bin/test: /usr/lib/x86_64-linux-gnu/libopencv_contrib.so.2.4.8
bin/test: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.2.4.8
bin/test: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.2.4.8
bin/test: /usr/lib/x86_64-linux-gnu/libopencv_legacy.so.2.4.8
bin/test: /usr/lib/x86_64-linux-gnu/libopencv_video.so.2.4.8
bin/test: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.2.4.8
bin/test: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.2.4.8
bin/test: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.2.4.8
bin/test: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.2.4.8
bin/test: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.2.4.8
bin/test: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.2.4.8
bin/test: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.2.4.8
bin/test: /usr/lib/x86_64-linux-gnu/libopencv_core.so.2.4.8
bin/test: test/CMakeFiles/test.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable ../bin/test"
	cd /home/robomakery/Code/slim/cvblob/test && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/test.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
test/CMakeFiles/test.dir/build: bin/test
.PHONY : test/CMakeFiles/test.dir/build

test/CMakeFiles/test.dir/requires: test/CMakeFiles/test.dir/test.cpp.o.requires
.PHONY : test/CMakeFiles/test.dir/requires

test/CMakeFiles/test.dir/clean:
	cd /home/robomakery/Code/slim/cvblob/test && $(CMAKE_COMMAND) -P CMakeFiles/test.dir/cmake_clean.cmake
.PHONY : test/CMakeFiles/test.dir/clean

test/CMakeFiles/test.dir/depend:
	cd /home/robomakery/Code/slim/cvblob && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/robomakery/Code/slim/cvblob /home/robomakery/Code/slim/cvblob/test /home/robomakery/Code/slim/cvblob /home/robomakery/Code/slim/cvblob/test /home/robomakery/Code/slim/cvblob/test/CMakeFiles/test.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : test/CMakeFiles/test.dir/depend

