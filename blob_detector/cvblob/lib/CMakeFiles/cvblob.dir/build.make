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
CMAKE_SOURCE_DIR = /home/robomakery/Downloads/cvblob

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/robomakery/Downloads/cvblob

# Include any dependencies generated for this target.
include lib/CMakeFiles/cvblob.dir/depend.make

# Include the progress variables for this target.
include lib/CMakeFiles/cvblob.dir/progress.make

# Include the compile flags for this target's objects.
include lib/CMakeFiles/cvblob.dir/flags.make

lib/CMakeFiles/cvblob.dir/cvblob.o: lib/CMakeFiles/cvblob.dir/flags.make
lib/CMakeFiles/cvblob.dir/cvblob.o: cvBlob/cvblob.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/robomakery/Downloads/cvblob/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object lib/CMakeFiles/cvblob.dir/cvblob.o"
	cd /home/robomakery/Downloads/cvblob/lib && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/cvblob.dir/cvblob.o -c /home/robomakery/Downloads/cvblob/cvBlob/cvblob.cpp

lib/CMakeFiles/cvblob.dir/cvblob.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/cvblob.dir/cvblob.i"
	cd /home/robomakery/Downloads/cvblob/lib && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/robomakery/Downloads/cvblob/cvBlob/cvblob.cpp > CMakeFiles/cvblob.dir/cvblob.i

lib/CMakeFiles/cvblob.dir/cvblob.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/cvblob.dir/cvblob.s"
	cd /home/robomakery/Downloads/cvblob/lib && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/robomakery/Downloads/cvblob/cvBlob/cvblob.cpp -o CMakeFiles/cvblob.dir/cvblob.s

lib/CMakeFiles/cvblob.dir/cvblob.o.requires:
.PHONY : lib/CMakeFiles/cvblob.dir/cvblob.o.requires

lib/CMakeFiles/cvblob.dir/cvblob.o.provides: lib/CMakeFiles/cvblob.dir/cvblob.o.requires
	$(MAKE) -f lib/CMakeFiles/cvblob.dir/build.make lib/CMakeFiles/cvblob.dir/cvblob.o.provides.build
.PHONY : lib/CMakeFiles/cvblob.dir/cvblob.o.provides

lib/CMakeFiles/cvblob.dir/cvblob.o.provides.build: lib/CMakeFiles/cvblob.dir/cvblob.o

lib/CMakeFiles/cvblob.dir/cvlabel.o: lib/CMakeFiles/cvblob.dir/flags.make
lib/CMakeFiles/cvblob.dir/cvlabel.o: cvBlob/cvlabel.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/robomakery/Downloads/cvblob/CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object lib/CMakeFiles/cvblob.dir/cvlabel.o"
	cd /home/robomakery/Downloads/cvblob/lib && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/cvblob.dir/cvlabel.o -c /home/robomakery/Downloads/cvblob/cvBlob/cvlabel.cpp

lib/CMakeFiles/cvblob.dir/cvlabel.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/cvblob.dir/cvlabel.i"
	cd /home/robomakery/Downloads/cvblob/lib && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/robomakery/Downloads/cvblob/cvBlob/cvlabel.cpp > CMakeFiles/cvblob.dir/cvlabel.i

lib/CMakeFiles/cvblob.dir/cvlabel.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/cvblob.dir/cvlabel.s"
	cd /home/robomakery/Downloads/cvblob/lib && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/robomakery/Downloads/cvblob/cvBlob/cvlabel.cpp -o CMakeFiles/cvblob.dir/cvlabel.s

lib/CMakeFiles/cvblob.dir/cvlabel.o.requires:
.PHONY : lib/CMakeFiles/cvblob.dir/cvlabel.o.requires

lib/CMakeFiles/cvblob.dir/cvlabel.o.provides: lib/CMakeFiles/cvblob.dir/cvlabel.o.requires
	$(MAKE) -f lib/CMakeFiles/cvblob.dir/build.make lib/CMakeFiles/cvblob.dir/cvlabel.o.provides.build
.PHONY : lib/CMakeFiles/cvblob.dir/cvlabel.o.provides

lib/CMakeFiles/cvblob.dir/cvlabel.o.provides.build: lib/CMakeFiles/cvblob.dir/cvlabel.o

lib/CMakeFiles/cvblob.dir/cvaux.o: lib/CMakeFiles/cvblob.dir/flags.make
lib/CMakeFiles/cvblob.dir/cvaux.o: cvBlob/cvaux.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/robomakery/Downloads/cvblob/CMakeFiles $(CMAKE_PROGRESS_3)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object lib/CMakeFiles/cvblob.dir/cvaux.o"
	cd /home/robomakery/Downloads/cvblob/lib && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/cvblob.dir/cvaux.o -c /home/robomakery/Downloads/cvblob/cvBlob/cvaux.cpp

lib/CMakeFiles/cvblob.dir/cvaux.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/cvblob.dir/cvaux.i"
	cd /home/robomakery/Downloads/cvblob/lib && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/robomakery/Downloads/cvblob/cvBlob/cvaux.cpp > CMakeFiles/cvblob.dir/cvaux.i

lib/CMakeFiles/cvblob.dir/cvaux.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/cvblob.dir/cvaux.s"
	cd /home/robomakery/Downloads/cvblob/lib && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/robomakery/Downloads/cvblob/cvBlob/cvaux.cpp -o CMakeFiles/cvblob.dir/cvaux.s

lib/CMakeFiles/cvblob.dir/cvaux.o.requires:
.PHONY : lib/CMakeFiles/cvblob.dir/cvaux.o.requires

lib/CMakeFiles/cvblob.dir/cvaux.o.provides: lib/CMakeFiles/cvblob.dir/cvaux.o.requires
	$(MAKE) -f lib/CMakeFiles/cvblob.dir/build.make lib/CMakeFiles/cvblob.dir/cvaux.o.provides.build
.PHONY : lib/CMakeFiles/cvblob.dir/cvaux.o.provides

lib/CMakeFiles/cvblob.dir/cvaux.o.provides.build: lib/CMakeFiles/cvblob.dir/cvaux.o

lib/CMakeFiles/cvblob.dir/cvcontour.o: lib/CMakeFiles/cvblob.dir/flags.make
lib/CMakeFiles/cvblob.dir/cvcontour.o: cvBlob/cvcontour.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/robomakery/Downloads/cvblob/CMakeFiles $(CMAKE_PROGRESS_4)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object lib/CMakeFiles/cvblob.dir/cvcontour.o"
	cd /home/robomakery/Downloads/cvblob/lib && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/cvblob.dir/cvcontour.o -c /home/robomakery/Downloads/cvblob/cvBlob/cvcontour.cpp

lib/CMakeFiles/cvblob.dir/cvcontour.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/cvblob.dir/cvcontour.i"
	cd /home/robomakery/Downloads/cvblob/lib && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/robomakery/Downloads/cvblob/cvBlob/cvcontour.cpp > CMakeFiles/cvblob.dir/cvcontour.i

lib/CMakeFiles/cvblob.dir/cvcontour.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/cvblob.dir/cvcontour.s"
	cd /home/robomakery/Downloads/cvblob/lib && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/robomakery/Downloads/cvblob/cvBlob/cvcontour.cpp -o CMakeFiles/cvblob.dir/cvcontour.s

lib/CMakeFiles/cvblob.dir/cvcontour.o.requires:
.PHONY : lib/CMakeFiles/cvblob.dir/cvcontour.o.requires

lib/CMakeFiles/cvblob.dir/cvcontour.o.provides: lib/CMakeFiles/cvblob.dir/cvcontour.o.requires
	$(MAKE) -f lib/CMakeFiles/cvblob.dir/build.make lib/CMakeFiles/cvblob.dir/cvcontour.o.provides.build
.PHONY : lib/CMakeFiles/cvblob.dir/cvcontour.o.provides

lib/CMakeFiles/cvblob.dir/cvcontour.o.provides.build: lib/CMakeFiles/cvblob.dir/cvcontour.o

lib/CMakeFiles/cvblob.dir/cvtrack.o: lib/CMakeFiles/cvblob.dir/flags.make
lib/CMakeFiles/cvblob.dir/cvtrack.o: cvBlob/cvtrack.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/robomakery/Downloads/cvblob/CMakeFiles $(CMAKE_PROGRESS_5)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object lib/CMakeFiles/cvblob.dir/cvtrack.o"
	cd /home/robomakery/Downloads/cvblob/lib && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/cvblob.dir/cvtrack.o -c /home/robomakery/Downloads/cvblob/cvBlob/cvtrack.cpp

lib/CMakeFiles/cvblob.dir/cvtrack.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/cvblob.dir/cvtrack.i"
	cd /home/robomakery/Downloads/cvblob/lib && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/robomakery/Downloads/cvblob/cvBlob/cvtrack.cpp > CMakeFiles/cvblob.dir/cvtrack.i

lib/CMakeFiles/cvblob.dir/cvtrack.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/cvblob.dir/cvtrack.s"
	cd /home/robomakery/Downloads/cvblob/lib && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/robomakery/Downloads/cvblob/cvBlob/cvtrack.cpp -o CMakeFiles/cvblob.dir/cvtrack.s

lib/CMakeFiles/cvblob.dir/cvtrack.o.requires:
.PHONY : lib/CMakeFiles/cvblob.dir/cvtrack.o.requires

lib/CMakeFiles/cvblob.dir/cvtrack.o.provides: lib/CMakeFiles/cvblob.dir/cvtrack.o.requires
	$(MAKE) -f lib/CMakeFiles/cvblob.dir/build.make lib/CMakeFiles/cvblob.dir/cvtrack.o.provides.build
.PHONY : lib/CMakeFiles/cvblob.dir/cvtrack.o.provides

lib/CMakeFiles/cvblob.dir/cvtrack.o.provides.build: lib/CMakeFiles/cvblob.dir/cvtrack.o

lib/CMakeFiles/cvblob.dir/cvcolor.o: lib/CMakeFiles/cvblob.dir/flags.make
lib/CMakeFiles/cvblob.dir/cvcolor.o: cvBlob/cvcolor.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/robomakery/Downloads/cvblob/CMakeFiles $(CMAKE_PROGRESS_6)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object lib/CMakeFiles/cvblob.dir/cvcolor.o"
	cd /home/robomakery/Downloads/cvblob/lib && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/cvblob.dir/cvcolor.o -c /home/robomakery/Downloads/cvblob/cvBlob/cvcolor.cpp

lib/CMakeFiles/cvblob.dir/cvcolor.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/cvblob.dir/cvcolor.i"
	cd /home/robomakery/Downloads/cvblob/lib && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/robomakery/Downloads/cvblob/cvBlob/cvcolor.cpp > CMakeFiles/cvblob.dir/cvcolor.i

lib/CMakeFiles/cvblob.dir/cvcolor.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/cvblob.dir/cvcolor.s"
	cd /home/robomakery/Downloads/cvblob/lib && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/robomakery/Downloads/cvblob/cvBlob/cvcolor.cpp -o CMakeFiles/cvblob.dir/cvcolor.s

lib/CMakeFiles/cvblob.dir/cvcolor.o.requires:
.PHONY : lib/CMakeFiles/cvblob.dir/cvcolor.o.requires

lib/CMakeFiles/cvblob.dir/cvcolor.o.provides: lib/CMakeFiles/cvblob.dir/cvcolor.o.requires
	$(MAKE) -f lib/CMakeFiles/cvblob.dir/build.make lib/CMakeFiles/cvblob.dir/cvcolor.o.provides.build
.PHONY : lib/CMakeFiles/cvblob.dir/cvcolor.o.provides

lib/CMakeFiles/cvblob.dir/cvcolor.o.provides.build: lib/CMakeFiles/cvblob.dir/cvcolor.o

# Object files for target cvblob
cvblob_OBJECTS = \
"CMakeFiles/cvblob.dir/cvblob.o" \
"CMakeFiles/cvblob.dir/cvlabel.o" \
"CMakeFiles/cvblob.dir/cvaux.o" \
"CMakeFiles/cvblob.dir/cvcontour.o" \
"CMakeFiles/cvblob.dir/cvtrack.o" \
"CMakeFiles/cvblob.dir/cvcolor.o"

# External object files for target cvblob
cvblob_EXTERNAL_OBJECTS =

lib/libcvblob.so: lib/CMakeFiles/cvblob.dir/cvblob.o
lib/libcvblob.so: lib/CMakeFiles/cvblob.dir/cvlabel.o
lib/libcvblob.so: lib/CMakeFiles/cvblob.dir/cvaux.o
lib/libcvblob.so: lib/CMakeFiles/cvblob.dir/cvcontour.o
lib/libcvblob.so: lib/CMakeFiles/cvblob.dir/cvtrack.o
lib/libcvblob.so: lib/CMakeFiles/cvblob.dir/cvcolor.o
lib/libcvblob.so: lib/CMakeFiles/cvblob.dir/build.make
lib/libcvblob.so: /usr/lib/x86_64-linux-gnu/libopencv_videostab.so.2.4.8
lib/libcvblob.so: /usr/lib/x86_64-linux-gnu/libopencv_video.so.2.4.8
lib/libcvblob.so: /usr/lib/x86_64-linux-gnu/libopencv_ts.so.2.4.8
lib/libcvblob.so: /usr/lib/x86_64-linux-gnu/libopencv_superres.so.2.4.8
lib/libcvblob.so: /usr/lib/x86_64-linux-gnu/libopencv_stitching.so.2.4.8
lib/libcvblob.so: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.2.4.8
lib/libcvblob.so: /usr/lib/x86_64-linux-gnu/libopencv_ocl.so.2.4.8
lib/libcvblob.so: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.2.4.8
lib/libcvblob.so: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.2.4.8
lib/libcvblob.so: /usr/lib/x86_64-linux-gnu/libopencv_legacy.so.2.4.8
lib/libcvblob.so: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.2.4.8
lib/libcvblob.so: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.2.4.8
lib/libcvblob.so: /usr/lib/x86_64-linux-gnu/libopencv_gpu.so.2.4.8
lib/libcvblob.so: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.2.4.8
lib/libcvblob.so: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.2.4.8
lib/libcvblob.so: /usr/lib/x86_64-linux-gnu/libopencv_core.so.2.4.8
lib/libcvblob.so: /usr/lib/x86_64-linux-gnu/libopencv_contrib.so.2.4.8
lib/libcvblob.so: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.2.4.8
lib/libcvblob.so: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.2.4.8
lib/libcvblob.so: /usr/lib/x86_64-linux-gnu/libopencv_legacy.so.2.4.8
lib/libcvblob.so: /usr/lib/x86_64-linux-gnu/libopencv_video.so.2.4.8
lib/libcvblob.so: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.2.4.8
lib/libcvblob.so: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.2.4.8
lib/libcvblob.so: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.2.4.8
lib/libcvblob.so: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.2.4.8
lib/libcvblob.so: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.2.4.8
lib/libcvblob.so: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.2.4.8
lib/libcvblob.so: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.2.4.8
lib/libcvblob.so: /usr/lib/x86_64-linux-gnu/libopencv_core.so.2.4.8
lib/libcvblob.so: lib/CMakeFiles/cvblob.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX shared library libcvblob.so"
	cd /home/robomakery/Downloads/cvblob/lib && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/cvblob.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
lib/CMakeFiles/cvblob.dir/build: lib/libcvblob.so
.PHONY : lib/CMakeFiles/cvblob.dir/build

lib/CMakeFiles/cvblob.dir/requires: lib/CMakeFiles/cvblob.dir/cvblob.o.requires
lib/CMakeFiles/cvblob.dir/requires: lib/CMakeFiles/cvblob.dir/cvlabel.o.requires
lib/CMakeFiles/cvblob.dir/requires: lib/CMakeFiles/cvblob.dir/cvaux.o.requires
lib/CMakeFiles/cvblob.dir/requires: lib/CMakeFiles/cvblob.dir/cvcontour.o.requires
lib/CMakeFiles/cvblob.dir/requires: lib/CMakeFiles/cvblob.dir/cvtrack.o.requires
lib/CMakeFiles/cvblob.dir/requires: lib/CMakeFiles/cvblob.dir/cvcolor.o.requires
.PHONY : lib/CMakeFiles/cvblob.dir/requires

lib/CMakeFiles/cvblob.dir/clean:
	cd /home/robomakery/Downloads/cvblob/lib && $(CMAKE_COMMAND) -P CMakeFiles/cvblob.dir/cmake_clean.cmake
.PHONY : lib/CMakeFiles/cvblob.dir/clean

lib/CMakeFiles/cvblob.dir/depend:
	cd /home/robomakery/Downloads/cvblob && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/robomakery/Downloads/cvblob /home/robomakery/Downloads/cvblob/cvBlob /home/robomakery/Downloads/cvblob /home/robomakery/Downloads/cvblob/lib /home/robomakery/Downloads/cvblob/lib/CMakeFiles/cvblob.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : lib/CMakeFiles/cvblob.dir/depend

