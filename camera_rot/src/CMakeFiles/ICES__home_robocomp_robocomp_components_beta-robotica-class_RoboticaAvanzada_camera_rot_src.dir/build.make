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

# Utility rule file for ICES__home_robocomp_robocomp_components_beta-robotica-class_RoboticaAvanzada_camera_rot_src.

# Include the progress variables for this target.
include src/CMakeFiles/ICES__home_robocomp_robocomp_components_beta-robotica-class_RoboticaAvanzada_camera_rot_src.dir/progress.make

ICES__home_robocomp_robocomp_components_beta-robotica-class_RoboticaAvanzada_camera_rot_src: src/CMakeFiles/ICES__home_robocomp_robocomp_components_beta-robotica-class_RoboticaAvanzada_camera_rot_src.dir/build.make
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "BU robocompdsl /home/alumno/robocomp/interfaces/IDSLs/CommonBehavior.idsl /home/robocomp/robocomp/components/beta-robotica-class/RoboticaAvanzada/camera_rot/src/CommonBehavior.ice"
	cd /home/alumno/robocomp/components/beta-robotica-class/RoboticaAvanzada/camera_rot/src && robocompdsl /home/alumno/robocomp/interfaces/IDSLs/CommonBehavior.idsl /home/robocomp/robocomp/components/beta-robotica-class/RoboticaAvanzada/camera_rot/src/CommonBehavior.ice
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "BU robocompdsl /home/alumno/robocomp/interfaces/IDSLs/CameraRGBDSimple.idsl /home/robocomp/robocomp/components/beta-robotica-class/RoboticaAvanzada/camera_rot/src/CameraRGBDSimple.ice"
	cd /home/alumno/robocomp/components/beta-robotica-class/RoboticaAvanzada/camera_rot/src && robocompdsl /home/alumno/robocomp/interfaces/IDSLs/CameraRGBDSimple.idsl /home/robocomp/robocomp/components/beta-robotica-class/RoboticaAvanzada/camera_rot/src/CameraRGBDSimple.ice
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "BU robocompdsl /home/alumno/robocomp/interfaces/IDSLs/CameraSimple.idsl /home/robocomp/robocomp/components/beta-robotica-class/RoboticaAvanzada/camera_rot/src/CameraSimple.ice"
	cd /home/alumno/robocomp/components/beta-robotica-class/RoboticaAvanzada/camera_rot/src && robocompdsl /home/alumno/robocomp/interfaces/IDSLs/CameraSimple.idsl /home/robocomp/robocomp/components/beta-robotica-class/RoboticaAvanzada/camera_rot/src/CameraSimple.ice
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "BU robocompdsl /home/alumno/robocomp/interfaces/IDSLs/DifferentialRobot.idsl /home/robocomp/robocomp/components/beta-robotica-class/RoboticaAvanzada/camera_rot/src/DifferentialRobot.ice"
	cd /home/alumno/robocomp/components/beta-robotica-class/RoboticaAvanzada/camera_rot/src && robocompdsl /home/alumno/robocomp/interfaces/IDSLs/DifferentialRobot.idsl /home/robocomp/robocomp/components/beta-robotica-class/RoboticaAvanzada/camera_rot/src/DifferentialRobot.ice
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "BU robocompdsl /home/alumno/robocomp/interfaces/IDSLs/FullPoseEstimation.idsl /home/robocomp/robocomp/components/beta-robotica-class/RoboticaAvanzada/camera_rot/src/FullPoseEstimation.ice"
	cd /home/alumno/robocomp/components/beta-robotica-class/RoboticaAvanzada/camera_rot/src && robocompdsl /home/alumno/robocomp/interfaces/IDSLs/FullPoseEstimation.idsl /home/robocomp/robocomp/components/beta-robotica-class/RoboticaAvanzada/camera_rot/src/FullPoseEstimation.ice
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "BU robocompdsl /home/alumno/robocomp/interfaces/IDSLs/GenericBase.idsl /home/robocomp/robocomp/components/beta-robotica-class/RoboticaAvanzada/camera_rot/src/GenericBase.ice"
	cd /home/alumno/robocomp/components/beta-robotica-class/RoboticaAvanzada/camera_rot/src && robocompdsl /home/alumno/robocomp/interfaces/IDSLs/GenericBase.idsl /home/robocomp/robocomp/components/beta-robotica-class/RoboticaAvanzada/camera_rot/src/GenericBase.ice
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "BU robocompdsl /home/alumno/robocomp/interfaces/IDSLs/HumanCameraBody.idsl /home/robocomp/robocomp/components/beta-robotica-class/RoboticaAvanzada/camera_rot/src/HumanCameraBody.ice"
	cd /home/alumno/robocomp/components/beta-robotica-class/RoboticaAvanzada/camera_rot/src && robocompdsl /home/alumno/robocomp/interfaces/IDSLs/HumanCameraBody.idsl /home/robocomp/robocomp/components/beta-robotica-class/RoboticaAvanzada/camera_rot/src/HumanCameraBody.ice
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "BU robocompdsl /home/alumno/robocomp/interfaces/IDSLs/JointMotorSimple.idsl /home/robocomp/robocomp/components/beta-robotica-class/RoboticaAvanzada/camera_rot/src/JointMotorSimple.ice"
	cd /home/alumno/robocomp/components/beta-robotica-class/RoboticaAvanzada/camera_rot/src && robocompdsl /home/alumno/robocomp/interfaces/IDSLs/JointMotorSimple.idsl /home/robocomp/robocomp/components/beta-robotica-class/RoboticaAvanzada/camera_rot/src/JointMotorSimple.ice
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "BU robocompdsl /home/alumno/robocomp/interfaces/IDSLs/Laser.idsl /home/robocomp/robocomp/components/beta-robotica-class/RoboticaAvanzada/camera_rot/src/Laser.ice"
	cd /home/alumno/robocomp/components/beta-robotica-class/RoboticaAvanzada/camera_rot/src && robocompdsl /home/alumno/robocomp/interfaces/IDSLs/Laser.idsl /home/robocomp/robocomp/components/beta-robotica-class/RoboticaAvanzada/camera_rot/src/Laser.ice
.PHONY : ICES__home_robocomp_robocomp_components_beta-robotica-class_RoboticaAvanzada_camera_rot_src

# Rule to build all files generated by this target.
src/CMakeFiles/ICES__home_robocomp_robocomp_components_beta-robotica-class_RoboticaAvanzada_camera_rot_src.dir/build: ICES__home_robocomp_robocomp_components_beta-robotica-class_RoboticaAvanzada_camera_rot_src

.PHONY : src/CMakeFiles/ICES__home_robocomp_robocomp_components_beta-robotica-class_RoboticaAvanzada_camera_rot_src.dir/build

src/CMakeFiles/ICES__home_robocomp_robocomp_components_beta-robotica-class_RoboticaAvanzada_camera_rot_src.dir/clean:
	cd /home/alumno/robocomp/components/beta-robotica-class/RoboticaAvanzada/camera_rot/src && $(CMAKE_COMMAND) -P CMakeFiles/ICES__home_robocomp_robocomp_components_beta-robotica-class_RoboticaAvanzada_camera_rot_src.dir/cmake_clean.cmake
.PHONY : src/CMakeFiles/ICES__home_robocomp_robocomp_components_beta-robotica-class_RoboticaAvanzada_camera_rot_src.dir/clean

src/CMakeFiles/ICES__home_robocomp_robocomp_components_beta-robotica-class_RoboticaAvanzada_camera_rot_src.dir/depend:
	cd /home/alumno/robocomp/components/beta-robotica-class/RoboticaAvanzada/camera_rot && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/robocomp/robocomp/components/beta-robotica-class/RoboticaAvanzada/camera_rot /home/robocomp/robocomp/components/beta-robotica-class/RoboticaAvanzada/camera_rot/src /home/alumno/robocomp/components/beta-robotica-class/RoboticaAvanzada/camera_rot /home/alumno/robocomp/components/beta-robotica-class/RoboticaAvanzada/camera_rot/src /home/alumno/robocomp/components/beta-robotica-class/RoboticaAvanzada/camera_rot/src/CMakeFiles/ICES__home_robocomp_robocomp_components_beta-robotica-class_RoboticaAvanzada_camera_rot_src.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/CMakeFiles/ICES__home_robocomp_robocomp_components_beta-robotica-class_RoboticaAvanzada_camera_rot_src.dir/depend

