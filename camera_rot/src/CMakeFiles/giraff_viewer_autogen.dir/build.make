# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

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
CMAKE_SOURCE_DIR = /home/robocomp/robocomp/components/beta-robotica-class/RoboticaAvanzada/camera_rot

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/alumno/robocomp/components/beta-robotica-class/RoboticaAvanzada/camera_rot

# Utility rule file for giraff_viewer_autogen.

# Include the progress variables for this target.
include src/CMakeFiles/giraff_viewer_autogen.dir/progress.make

src/CMakeFiles/giraff_viewer_autogen:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/alumno/robocomp/components/beta-robotica-class/RoboticaAvanzada/camera_rot/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Automatic MOC and UIC for target giraff_viewer"
	cd /home/alumno/robocomp/components/beta-robotica-class/RoboticaAvanzada/camera_rot/src && /usr/bin/cmake -E cmake_autogen /home/alumno/robocomp/components/beta-robotica-class/RoboticaAvanzada/camera_rot/src/CMakeFiles/giraff_viewer_autogen.dir/AutogenInfo.json Debug

giraff_viewer_autogen: src/CMakeFiles/giraff_viewer_autogen
giraff_viewer_autogen: src/CMakeFiles/giraff_viewer_autogen.dir/build.make

.PHONY : giraff_viewer_autogen

# Rule to build all files generated by this target.
src/CMakeFiles/giraff_viewer_autogen.dir/build: giraff_viewer_autogen

.PHONY : src/CMakeFiles/giraff_viewer_autogen.dir/build

src/CMakeFiles/giraff_viewer_autogen.dir/clean:
	cd /home/alumno/robocomp/components/beta-robotica-class/RoboticaAvanzada/camera_rot/src && $(CMAKE_COMMAND) -P CMakeFiles/giraff_viewer_autogen.dir/cmake_clean.cmake
.PHONY : src/CMakeFiles/giraff_viewer_autogen.dir/clean

src/CMakeFiles/giraff_viewer_autogen.dir/depend:
	cd /home/alumno/robocomp/components/beta-robotica-class/RoboticaAvanzada/camera_rot && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/robocomp/robocomp/components/beta-robotica-class/RoboticaAvanzada/camera_rot /home/robocomp/robocomp/components/beta-robotica-class/RoboticaAvanzada/camera_rot/src /home/alumno/robocomp/components/beta-robotica-class/RoboticaAvanzada/camera_rot /home/alumno/robocomp/components/beta-robotica-class/RoboticaAvanzada/camera_rot/src /home/alumno/robocomp/components/beta-robotica-class/RoboticaAvanzada/camera_rot/src/CMakeFiles/giraff_viewer_autogen.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/CMakeFiles/giraff_viewer_autogen.dir/depend

