# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

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
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/qiyu/VSCodeOs/VSCoding_2

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/qiyu/VSCodeOs/VSCoding_2/build

# Include any dependencies generated for this target.
include CMakeFiles/armordetection_result_exe.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/armordetection_result_exe.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/armordetection_result_exe.dir/flags.make

CMakeFiles/armordetection_result_exe.dir/armordetection_boostio.cpp.o: CMakeFiles/armordetection_result_exe.dir/flags.make
CMakeFiles/armordetection_result_exe.dir/armordetection_boostio.cpp.o: ../armordetection_boostio.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/qiyu/VSCodeOs/VSCoding_2/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/armordetection_result_exe.dir/armordetection_boostio.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/armordetection_result_exe.dir/armordetection_boostio.cpp.o -c /home/qiyu/VSCodeOs/VSCoding_2/armordetection_boostio.cpp

CMakeFiles/armordetection_result_exe.dir/armordetection_boostio.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/armordetection_result_exe.dir/armordetection_boostio.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/qiyu/VSCodeOs/VSCoding_2/armordetection_boostio.cpp > CMakeFiles/armordetection_result_exe.dir/armordetection_boostio.cpp.i

CMakeFiles/armordetection_result_exe.dir/armordetection_boostio.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/armordetection_result_exe.dir/armordetection_boostio.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/qiyu/VSCodeOs/VSCoding_2/armordetection_boostio.cpp -o CMakeFiles/armordetection_result_exe.dir/armordetection_boostio.cpp.s

CMakeFiles/armordetection_result_exe.dir/armordetection_boostio.cpp.o.requires:

.PHONY : CMakeFiles/armordetection_result_exe.dir/armordetection_boostio.cpp.o.requires

CMakeFiles/armordetection_result_exe.dir/armordetection_boostio.cpp.o.provides: CMakeFiles/armordetection_result_exe.dir/armordetection_boostio.cpp.o.requires
	$(MAKE) -f CMakeFiles/armordetection_result_exe.dir/build.make CMakeFiles/armordetection_result_exe.dir/armordetection_boostio.cpp.o.provides.build
.PHONY : CMakeFiles/armordetection_result_exe.dir/armordetection_boostio.cpp.o.provides

CMakeFiles/armordetection_result_exe.dir/armordetection_boostio.cpp.o.provides.build: CMakeFiles/armordetection_result_exe.dir/armordetection_boostio.cpp.o


# Object files for target armordetection_result_exe
armordetection_result_exe_OBJECTS = \
"CMakeFiles/armordetection_result_exe.dir/armordetection_boostio.cpp.o"

# External object files for target armordetection_result_exe
armordetection_result_exe_EXTERNAL_OBJECTS =

armordetection_result_exe: CMakeFiles/armordetection_result_exe.dir/armordetection_boostio.cpp.o
armordetection_result_exe: CMakeFiles/armordetection_result_exe.dir/build.make
armordetection_result_exe: /usr/local/lib/libopencv_stitching.so.3.2.0
armordetection_result_exe: /usr/local/lib/libopencv_superres.so.3.2.0
armordetection_result_exe: /usr/local/lib/libopencv_videostab.so.3.2.0
armordetection_result_exe: /usr/local/lib/libopencv_aruco.so.3.2.0
armordetection_result_exe: /usr/local/lib/libopencv_bgsegm.so.3.2.0
armordetection_result_exe: /usr/local/lib/libopencv_bioinspired.so.3.2.0
armordetection_result_exe: /usr/local/lib/libopencv_ccalib.so.3.2.0
armordetection_result_exe: /usr/local/lib/libopencv_dpm.so.3.2.0
armordetection_result_exe: /usr/local/lib/libopencv_freetype.so.3.2.0
armordetection_result_exe: /usr/local/lib/libopencv_fuzzy.so.3.2.0
armordetection_result_exe: /usr/local/lib/libopencv_line_descriptor.so.3.2.0
armordetection_result_exe: /usr/local/lib/libopencv_optflow.so.3.2.0
armordetection_result_exe: /usr/local/lib/libopencv_reg.so.3.2.0
armordetection_result_exe: /usr/local/lib/libopencv_saliency.so.3.2.0
armordetection_result_exe: /usr/local/lib/libopencv_stereo.so.3.2.0
armordetection_result_exe: /usr/local/lib/libopencv_structured_light.so.3.2.0
armordetection_result_exe: /usr/local/lib/libopencv_surface_matching.so.3.2.0
armordetection_result_exe: /usr/local/lib/libopencv_tracking.so.3.2.0
armordetection_result_exe: /usr/local/lib/libopencv_xfeatures2d.so.3.2.0
armordetection_result_exe: /usr/local/lib/libopencv_ximgproc.so.3.2.0
armordetection_result_exe: /usr/local/lib/libopencv_xobjdetect.so.3.2.0
armordetection_result_exe: /usr/local/lib/libopencv_xphoto.so.3.2.0
armordetection_result_exe: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
armordetection_result_exe: /usr/lib/x86_64-linux-gnu/libboost_regex.so
armordetection_result_exe: /usr/lib/x86_64-linux-gnu/libboost_system.so
armordetection_result_exe: /usr/local/lib/libopencv_shape.so.3.2.0
armordetection_result_exe: /usr/local/lib/libopencv_phase_unwrapping.so.3.2.0
armordetection_result_exe: /usr/local/lib/libopencv_rgbd.so.3.2.0
armordetection_result_exe: /usr/local/lib/libopencv_calib3d.so.3.2.0
armordetection_result_exe: /usr/local/lib/libopencv_video.so.3.2.0
armordetection_result_exe: /usr/local/lib/libopencv_datasets.so.3.2.0
armordetection_result_exe: /usr/local/lib/libopencv_dnn.so.3.2.0
armordetection_result_exe: /usr/local/lib/libopencv_face.so.3.2.0
armordetection_result_exe: /usr/local/lib/libopencv_plot.so.3.2.0
armordetection_result_exe: /usr/local/lib/libopencv_text.so.3.2.0
armordetection_result_exe: /usr/local/lib/libopencv_features2d.so.3.2.0
armordetection_result_exe: /usr/local/lib/libopencv_flann.so.3.2.0
armordetection_result_exe: /usr/local/lib/libopencv_objdetect.so.3.2.0
armordetection_result_exe: /usr/local/lib/libopencv_ml.so.3.2.0
armordetection_result_exe: /usr/local/lib/libopencv_highgui.so.3.2.0
armordetection_result_exe: /usr/local/lib/libopencv_photo.so.3.2.0
armordetection_result_exe: /usr/local/lib/libopencv_videoio.so.3.2.0
armordetection_result_exe: /usr/local/lib/libopencv_imgcodecs.so.3.2.0
armordetection_result_exe: /usr/local/lib/libopencv_imgproc.so.3.2.0
armordetection_result_exe: /usr/local/lib/libopencv_core.so.3.2.0
armordetection_result_exe: CMakeFiles/armordetection_result_exe.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/qiyu/VSCodeOs/VSCoding_2/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable armordetection_result_exe"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/armordetection_result_exe.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/armordetection_result_exe.dir/build: armordetection_result_exe

.PHONY : CMakeFiles/armordetection_result_exe.dir/build

CMakeFiles/armordetection_result_exe.dir/requires: CMakeFiles/armordetection_result_exe.dir/armordetection_boostio.cpp.o.requires

.PHONY : CMakeFiles/armordetection_result_exe.dir/requires

CMakeFiles/armordetection_result_exe.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/armordetection_result_exe.dir/cmake_clean.cmake
.PHONY : CMakeFiles/armordetection_result_exe.dir/clean

CMakeFiles/armordetection_result_exe.dir/depend:
	cd /home/qiyu/VSCodeOs/VSCoding_2/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/qiyu/VSCodeOs/VSCoding_2 /home/qiyu/VSCodeOs/VSCoding_2 /home/qiyu/VSCodeOs/VSCoding_2/build /home/qiyu/VSCodeOs/VSCoding_2/build /home/qiyu/VSCodeOs/VSCoding_2/build/CMakeFiles/armordetection_result_exe.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/armordetection_result_exe.dir/depend

