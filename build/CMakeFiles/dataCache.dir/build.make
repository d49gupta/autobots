# CMAKE generated file: DO NOT EDIT!
# Generated by "MinGW Makefiles" Generator, CMake Version 3.31

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

SHELL = cmd.exe

# The CMake executable.
CMAKE_COMMAND = C:\Users\dharm\Downloads\cmake-3.31.2-windows-x86_64\cmake-3.31.2-windows-x86_64\bin\cmake.exe

# The command to remove a file.
RM = C:\Users\dharm\Downloads\cmake-3.31.2-windows-x86_64\cmake-3.31.2-windows-x86_64\bin\cmake.exe -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = C:\Users\dharm\OneDrive\Documents\Learning\Software\Projects\Autonomous_car_project

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = C:\Users\dharm\OneDrive\Documents\Learning\Software\Projects\Autonomous_car_project\build

# Include any dependencies generated for this target.
include CMakeFiles/dataCache.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/dataCache.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/dataCache.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/dataCache.dir/flags.make

CMakeFiles/dataCache.dir/codegen:
.PHONY : CMakeFiles/dataCache.dir/codegen

CMakeFiles/dataCache.dir/src/output_nodes/dataCache.cpp.obj: CMakeFiles/dataCache.dir/flags.make
CMakeFiles/dataCache.dir/src/output_nodes/dataCache.cpp.obj: CMakeFiles/dataCache.dir/includes_CXX.rsp
CMakeFiles/dataCache.dir/src/output_nodes/dataCache.cpp.obj: C:/Users/dharm/OneDrive/Documents/Learning/Software/Projects/Autonomous_car_project/src/output_nodes/dataCache.cpp
CMakeFiles/dataCache.dir/src/output_nodes/dataCache.cpp.obj: CMakeFiles/dataCache.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=C:\Users\dharm\OneDrive\Documents\Learning\Software\Projects\Autonomous_car_project\build\CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/dataCache.dir/src/output_nodes/dataCache.cpp.obj"
	C:\msys64\mingw64\bin\c++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/dataCache.dir/src/output_nodes/dataCache.cpp.obj -MF CMakeFiles\dataCache.dir\src\output_nodes\dataCache.cpp.obj.d -o CMakeFiles\dataCache.dir\src\output_nodes\dataCache.cpp.obj -c C:\Users\dharm\OneDrive\Documents\Learning\Software\Projects\Autonomous_car_project\src\output_nodes\dataCache.cpp

CMakeFiles/dataCache.dir/src/output_nodes/dataCache.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/dataCache.dir/src/output_nodes/dataCache.cpp.i"
	C:\msys64\mingw64\bin\c++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E C:\Users\dharm\OneDrive\Documents\Learning\Software\Projects\Autonomous_car_project\src\output_nodes\dataCache.cpp > CMakeFiles\dataCache.dir\src\output_nodes\dataCache.cpp.i

CMakeFiles/dataCache.dir/src/output_nodes/dataCache.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/dataCache.dir/src/output_nodes/dataCache.cpp.s"
	C:\msys64\mingw64\bin\c++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S C:\Users\dharm\OneDrive\Documents\Learning\Software\Projects\Autonomous_car_project\src\output_nodes\dataCache.cpp -o CMakeFiles\dataCache.dir\src\output_nodes\dataCache.cpp.s

# Object files for target dataCache
dataCache_OBJECTS = \
"CMakeFiles/dataCache.dir/src/output_nodes/dataCache.cpp.obj"

# External object files for target dataCache
dataCache_EXTERNAL_OBJECTS =

dataCache.exe: CMakeFiles/dataCache.dir/src/output_nodes/dataCache.cpp.obj
dataCache.exe: CMakeFiles/dataCache.dir/build.make
dataCache.exe: CMakeFiles/dataCache.dir/linkLibs.rsp
dataCache.exe: CMakeFiles/dataCache.dir/objects1.rsp
dataCache.exe: CMakeFiles/dataCache.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --bold --progress-dir=C:\Users\dharm\OneDrive\Documents\Learning\Software\Projects\Autonomous_car_project\build\CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable dataCache.exe"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles\dataCache.dir\link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/dataCache.dir/build: dataCache.exe
.PHONY : CMakeFiles/dataCache.dir/build

CMakeFiles/dataCache.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles\dataCache.dir\cmake_clean.cmake
.PHONY : CMakeFiles/dataCache.dir/clean

CMakeFiles/dataCache.dir/depend:
	$(CMAKE_COMMAND) -E cmake_depends "MinGW Makefiles" C:\Users\dharm\OneDrive\Documents\Learning\Software\Projects\Autonomous_car_project C:\Users\dharm\OneDrive\Documents\Learning\Software\Projects\Autonomous_car_project C:\Users\dharm\OneDrive\Documents\Learning\Software\Projects\Autonomous_car_project\build C:\Users\dharm\OneDrive\Documents\Learning\Software\Projects\Autonomous_car_project\build C:\Users\dharm\OneDrive\Documents\Learning\Software\Projects\Autonomous_car_project\build\CMakeFiles\dataCache.dir\DependInfo.cmake "--color=$(COLOR)"
.PHONY : CMakeFiles/dataCache.dir/depend

