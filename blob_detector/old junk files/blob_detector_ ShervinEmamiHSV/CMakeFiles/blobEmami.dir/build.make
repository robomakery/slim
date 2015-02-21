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
CMAKE_SOURCE_DIR = "/home/robomakery/Code/slim/blob_detector/blob_detector_ ShervinEmamiHSV"

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = "/home/robomakery/Code/slim/blob_detector/blob_detector_ ShervinEmamiHSV"

# Include any dependencies generated for this target.
include CMakeFiles/blobEmami.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/blobEmami.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/blobEmami.dir/flags.make

CMakeFiles/blobEmami.dir/blobDetectorEmami.cpp.o: CMakeFiles/blobEmami.dir/flags.make
CMakeFiles/blobEmami.dir/blobDetectorEmami.cpp.o: blobDetectorEmami.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report "/home/robomakery/Code/slim/blob_detector/blob_detector_ ShervinEmamiHSV/CMakeFiles" $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/blobEmami.dir/blobDetectorEmami.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/blobEmami.dir/blobDetectorEmami.cpp.o -c "/home/robomakery/Code/slim/blob_detector/blob_detector_ ShervinEmamiHSV/blobDetectorEmami.cpp"

CMakeFiles/blobEmami.dir/blobDetectorEmami.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/blobEmami.dir/blobDetectorEmami.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E "/home/robomakery/Code/slim/blob_detector/blob_detector_ ShervinEmamiHSV/blobDetectorEmami.cpp" > CMakeFiles/blobEmami.dir/blobDetectorEmami.cpp.i

CMakeFiles/blobEmami.dir/blobDetectorEmami.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/blobEmami.dir/blobDetectorEmami.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S "/home/robomakery/Code/slim/blob_detector/blob_detector_ ShervinEmamiHSV/blobDetectorEmami.cpp" -o CMakeFiles/blobEmami.dir/blobDetectorEmami.cpp.s

CMakeFiles/blobEmami.dir/blobDetectorEmami.cpp.o.requires:
.PHONY : CMakeFiles/blobEmami.dir/blobDetectorEmami.cpp.o.requires

CMakeFiles/blobEmami.dir/blobDetectorEmami.cpp.o.provides: CMakeFiles/blobEmami.dir/blobDetectorEmami.cpp.o.requires
	$(MAKE) -f CMakeFiles/blobEmami.dir/build.make CMakeFiles/blobEmami.dir/blobDetectorEmami.cpp.o.provides.build
.PHONY : CMakeFiles/blobEmami.dir/blobDetectorEmami.cpp.o.provides

CMakeFiles/blobEmami.dir/blobDetectorEmami.cpp.o.provides.build: CMakeFiles/blobEmami.dir/blobDetectorEmami.cpp.o

# Object files for target blobEmami
blobEmami_OBJECTS = \
"CMakeFiles/blobEmami.dir/blobDetectorEmami.cpp.o"

# External object files for target blobEmami
blobEmami_EXTERNAL_OBJECTS =

blobEmami: CMakeFiles/blobEmami.dir/blobDetectorEmami.cpp.o
blobEmami: CMakeFiles/blobEmami.dir/build.make
blobEmami: /usr/lib/x86_64-linux-gnu/libopencv_videostab.so.2.4.8
blobEmami: /usr/lib/x86_64-linux-gnu/libopencv_video.so.2.4.8
blobEmami: /usr/lib/x86_64-linux-gnu/libopencv_ts.so.2.4.8
blobEmami: /usr/lib/x86_64-linux-gnu/libopencv_superres.so.2.4.8
blobEmami: /usr/lib/x86_64-linux-gnu/libopencv_stitching.so.2.4.8
blobEmami: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.2.4.8
blobEmami: /usr/lib/x86_64-linux-gnu/libopencv_ocl.so.2.4.8
blobEmami: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.2.4.8
blobEmami: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.2.4.8
blobEmami: /usr/lib/x86_64-linux-gnu/libopencv_legacy.so.2.4.8
blobEmami: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.2.4.8
blobEmami: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.2.4.8
blobEmami: /usr/lib/x86_64-linux-gnu/libopencv_gpu.so.2.4.8
blobEmami: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.2.4.8
blobEmami: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.2.4.8
blobEmami: /usr/lib/x86_64-linux-gnu/libopencv_core.so.2.4.8
blobEmami: /usr/lib/x86_64-linux-gnu/libopencv_contrib.so.2.4.8
blobEmami: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.2.4.8
blobEmami: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.2.4.8
blobEmami: /usr/lib/x86_64-linux-gnu/libopencv_legacy.so.2.4.8
blobEmami: /usr/lib/x86_64-linux-gnu/libopencv_video.so.2.4.8
blobEmami: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.2.4.8
blobEmami: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.2.4.8
blobEmami: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.2.4.8
blobEmami: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.2.4.8
blobEmami: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.2.4.8
blobEmami: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.2.4.8
blobEmami: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.2.4.8
blobEmami: /usr/lib/x86_64-linux-gnu/libopencv_core.so.2.4.8
blobEmami: CMakeFiles/blobEmami.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable blobEmami"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/blobEmami.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/blobEmami.dir/build: blobEmami
.PHONY : CMakeFiles/blobEmami.dir/build

CMakeFiles/blobEmami.dir/requires: CMakeFiles/blobEmami.dir/blobDetectorEmami.cpp.o.requires
.PHONY : CMakeFiles/blobEmami.dir/requires

CMakeFiles/blobEmami.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/blobEmami.dir/cmake_clean.cmake
.PHONY : CMakeFiles/blobEmami.dir/clean

CMakeFiles/blobEmami.dir/depend:
	cd "/home/robomakery/Code/slim/blob_detector/blob_detector_ ShervinEmamiHSV" && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" "/home/robomakery/Code/slim/blob_detector/blob_detector_ ShervinEmamiHSV" "/home/robomakery/Code/slim/blob_detector/blob_detector_ ShervinEmamiHSV" "/home/robomakery/Code/slim/blob_detector/blob_detector_ ShervinEmamiHSV" "/home/robomakery/Code/slim/blob_detector/blob_detector_ ShervinEmamiHSV" "/home/robomakery/Code/slim/blob_detector/blob_detector_ ShervinEmamiHSV/CMakeFiles/blobEmami.dir/DependInfo.cmake" --color=$(COLOR)
.PHONY : CMakeFiles/blobEmami.dir/depend
