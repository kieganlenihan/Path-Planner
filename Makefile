# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.20

# Default target executed when no arguments are given to make.
default_target: all
.PHONY : default_target

# Allow only one "make -f Makefile2" at a time, but pass parallelism.
.NOTPARALLEL:

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

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/local/Cellar/cmake/3.20.1/bin/cmake

# The command to remove a file.
RM = /usr/local/Cellar/cmake/3.20.1/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = "/Users/kieganlenihan/Documents/GitHub/Path Planner"

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = "/Users/kieganlenihan/Documents/GitHub/Path Planner"

#=============================================================================
# Targets provided globally by CMake.

# Special rule for the target rebuild_cache
rebuild_cache:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "Running CMake to regenerate build system..."
	/usr/local/Cellar/cmake/3.20.1/bin/cmake --regenerate-during-build -S$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR)
.PHONY : rebuild_cache

# Special rule for the target rebuild_cache
rebuild_cache/fast: rebuild_cache
.PHONY : rebuild_cache/fast

# Special rule for the target edit_cache
edit_cache:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "Running CMake cache editor..."
	/usr/local/Cellar/cmake/3.20.1/bin/ccmake -S$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR)
.PHONY : edit_cache

# Special rule for the target edit_cache
edit_cache/fast: edit_cache
.PHONY : edit_cache/fast

# The main all target
all: cmake_check_build_system
	$(CMAKE_COMMAND) -E cmake_progress_start "/Users/kieganlenihan/Documents/GitHub/Path Planner/CMakeFiles" "/Users/kieganlenihan/Documents/GitHub/Path Planner//CMakeFiles/progress.marks"
	$(MAKE) $(MAKESILENT) -f CMakeFiles/Makefile2 all
	$(CMAKE_COMMAND) -E cmake_progress_start "/Users/kieganlenihan/Documents/GitHub/Path Planner/CMakeFiles" 0
.PHONY : all

# The main clean target
clean:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/Makefile2 clean
.PHONY : clean

# The main clean target
clean/fast: clean
.PHONY : clean/fast

# Prepare targets for installation.
preinstall: all
	$(MAKE) $(MAKESILENT) -f CMakeFiles/Makefile2 preinstall
.PHONY : preinstall

# Prepare targets for installation.
preinstall/fast:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/Makefile2 preinstall
.PHONY : preinstall/fast

# clear depends
depend:
	$(CMAKE_COMMAND) -S$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR) --check-build-system CMakeFiles/Makefile.cmake 1
.PHONY : depend

#=============================================================================
# Target rules for targets named path_planner

# Build rule for target.
path_planner: cmake_check_build_system
	$(MAKE) $(MAKESILENT) -f CMakeFiles/Makefile2 path_planner
.PHONY : path_planner

# fast build rule for target.
path_planner/fast:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/path_planner.dir/build.make CMakeFiles/path_planner.dir/build
.PHONY : path_planner/fast

#=============================================================================
# Target rules for targets named CGAL_Qt5_moc_and_resources

# Build rule for target.
CGAL_Qt5_moc_and_resources: cmake_check_build_system
	$(MAKE) $(MAKESILENT) -f CMakeFiles/Makefile2 CGAL_Qt5_moc_and_resources
.PHONY : CGAL_Qt5_moc_and_resources

# fast build rule for target.
CGAL_Qt5_moc_and_resources/fast:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/CGAL_Qt5_moc_and_resources.dir/build.make CMakeFiles/CGAL_Qt5_moc_and_resources.dir/build
.PHONY : CGAL_Qt5_moc_and_resources/fast

#=============================================================================
# Target rules for targets named CGAL_Qt5_moc_and_resources_autogen

# Build rule for target.
CGAL_Qt5_moc_and_resources_autogen: cmake_check_build_system
	$(MAKE) $(MAKESILENT) -f CMakeFiles/Makefile2 CGAL_Qt5_moc_and_resources_autogen
.PHONY : CGAL_Qt5_moc_and_resources_autogen

# fast build rule for target.
CGAL_Qt5_moc_and_resources_autogen/fast:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/CGAL_Qt5_moc_and_resources_autogen.dir/build.make CMakeFiles/CGAL_Qt5_moc_and_resources_autogen.dir/build
.PHONY : CGAL_Qt5_moc_and_resources_autogen/fast

CGAL_Qt5_moc_and_resources_autogen/mocs_compilation.o: CGAL_Qt5_moc_and_resources_autogen/mocs_compilation.cpp.o
.PHONY : CGAL_Qt5_moc_and_resources_autogen/mocs_compilation.o

# target to build an object file
CGAL_Qt5_moc_and_resources_autogen/mocs_compilation.cpp.o:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/CGAL_Qt5_moc_and_resources.dir/build.make CMakeFiles/CGAL_Qt5_moc_and_resources.dir/CGAL_Qt5_moc_and_resources_autogen/mocs_compilation.cpp.o
.PHONY : CGAL_Qt5_moc_and_resources_autogen/mocs_compilation.cpp.o

CGAL_Qt5_moc_and_resources_autogen/mocs_compilation.i: CGAL_Qt5_moc_and_resources_autogen/mocs_compilation.cpp.i
.PHONY : CGAL_Qt5_moc_and_resources_autogen/mocs_compilation.i

# target to preprocess a source file
CGAL_Qt5_moc_and_resources_autogen/mocs_compilation.cpp.i:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/CGAL_Qt5_moc_and_resources.dir/build.make CMakeFiles/CGAL_Qt5_moc_and_resources.dir/CGAL_Qt5_moc_and_resources_autogen/mocs_compilation.cpp.i
.PHONY : CGAL_Qt5_moc_and_resources_autogen/mocs_compilation.cpp.i

CGAL_Qt5_moc_and_resources_autogen/mocs_compilation.s: CGAL_Qt5_moc_and_resources_autogen/mocs_compilation.cpp.s
.PHONY : CGAL_Qt5_moc_and_resources_autogen/mocs_compilation.s

# target to generate assembly for a file
CGAL_Qt5_moc_and_resources_autogen/mocs_compilation.cpp.s:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/CGAL_Qt5_moc_and_resources.dir/build.make CMakeFiles/CGAL_Qt5_moc_and_resources.dir/CGAL_Qt5_moc_and_resources_autogen/mocs_compilation.cpp.s
.PHONY : CGAL_Qt5_moc_and_resources_autogen/mocs_compilation.cpp.s

qrc_CGAL.o: qrc_CGAL.cpp.o
.PHONY : qrc_CGAL.o

# target to build an object file
qrc_CGAL.cpp.o:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/CGAL_Qt5_moc_and_resources.dir/build.make CMakeFiles/CGAL_Qt5_moc_and_resources.dir/qrc_CGAL.cpp.o
.PHONY : qrc_CGAL.cpp.o

qrc_CGAL.i: qrc_CGAL.cpp.i
.PHONY : qrc_CGAL.i

# target to preprocess a source file
qrc_CGAL.cpp.i:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/CGAL_Qt5_moc_and_resources.dir/build.make CMakeFiles/CGAL_Qt5_moc_and_resources.dir/qrc_CGAL.cpp.i
.PHONY : qrc_CGAL.cpp.i

qrc_CGAL.s: qrc_CGAL.cpp.s
.PHONY : qrc_CGAL.s

# target to generate assembly for a file
qrc_CGAL.cpp.s:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/CGAL_Qt5_moc_and_resources.dir/build.make CMakeFiles/CGAL_Qt5_moc_and_resources.dir/qrc_CGAL.cpp.s
.PHONY : qrc_CGAL.cpp.s

qrc_File.o: qrc_File.cpp.o
.PHONY : qrc_File.o

# target to build an object file
qrc_File.cpp.o:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/CGAL_Qt5_moc_and_resources.dir/build.make CMakeFiles/CGAL_Qt5_moc_and_resources.dir/qrc_File.cpp.o
.PHONY : qrc_File.cpp.o

qrc_File.i: qrc_File.cpp.i
.PHONY : qrc_File.i

# target to preprocess a source file
qrc_File.cpp.i:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/CGAL_Qt5_moc_and_resources.dir/build.make CMakeFiles/CGAL_Qt5_moc_and_resources.dir/qrc_File.cpp.i
.PHONY : qrc_File.cpp.i

qrc_File.s: qrc_File.cpp.s
.PHONY : qrc_File.s

# target to generate assembly for a file
qrc_File.cpp.s:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/CGAL_Qt5_moc_and_resources.dir/build.make CMakeFiles/CGAL_Qt5_moc_and_resources.dir/qrc_File.cpp.s
.PHONY : qrc_File.cpp.s

qrc_Input.o: qrc_Input.cpp.o
.PHONY : qrc_Input.o

# target to build an object file
qrc_Input.cpp.o:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/CGAL_Qt5_moc_and_resources.dir/build.make CMakeFiles/CGAL_Qt5_moc_and_resources.dir/qrc_Input.cpp.o
.PHONY : qrc_Input.cpp.o

qrc_Input.i: qrc_Input.cpp.i
.PHONY : qrc_Input.i

# target to preprocess a source file
qrc_Input.cpp.i:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/CGAL_Qt5_moc_and_resources.dir/build.make CMakeFiles/CGAL_Qt5_moc_and_resources.dir/qrc_Input.cpp.i
.PHONY : qrc_Input.cpp.i

qrc_Input.s: qrc_Input.cpp.s
.PHONY : qrc_Input.s

# target to generate assembly for a file
qrc_Input.cpp.s:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/CGAL_Qt5_moc_and_resources.dir/build.make CMakeFiles/CGAL_Qt5_moc_and_resources.dir/qrc_Input.cpp.s
.PHONY : qrc_Input.cpp.s

qrc_Triangulation_2.o: qrc_Triangulation_2.cpp.o
.PHONY : qrc_Triangulation_2.o

# target to build an object file
qrc_Triangulation_2.cpp.o:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/CGAL_Qt5_moc_and_resources.dir/build.make CMakeFiles/CGAL_Qt5_moc_and_resources.dir/qrc_Triangulation_2.cpp.o
.PHONY : qrc_Triangulation_2.cpp.o

qrc_Triangulation_2.i: qrc_Triangulation_2.cpp.i
.PHONY : qrc_Triangulation_2.i

# target to preprocess a source file
qrc_Triangulation_2.cpp.i:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/CGAL_Qt5_moc_and_resources.dir/build.make CMakeFiles/CGAL_Qt5_moc_and_resources.dir/qrc_Triangulation_2.cpp.i
.PHONY : qrc_Triangulation_2.cpp.i

qrc_Triangulation_2.s: qrc_Triangulation_2.cpp.s
.PHONY : qrc_Triangulation_2.s

# target to generate assembly for a file
qrc_Triangulation_2.cpp.s:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/CGAL_Qt5_moc_and_resources.dir/build.make CMakeFiles/CGAL_Qt5_moc_and_resources.dir/qrc_Triangulation_2.cpp.s
.PHONY : qrc_Triangulation_2.cpp.s

src/mover.o: src/mover.cpp.o
.PHONY : src/mover.o

# target to build an object file
src/mover.cpp.o:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/path_planner.dir/build.make CMakeFiles/path_planner.dir/src/mover.cpp.o
.PHONY : src/mover.cpp.o

src/mover.i: src/mover.cpp.i
.PHONY : src/mover.i

# target to preprocess a source file
src/mover.cpp.i:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/path_planner.dir/build.make CMakeFiles/path_planner.dir/src/mover.cpp.i
.PHONY : src/mover.cpp.i

src/mover.s: src/mover.cpp.s
.PHONY : src/mover.s

# target to generate assembly for a file
src/mover.cpp.s:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/path_planner.dir/build.make CMakeFiles/path_planner.dir/src/mover.cpp.s
.PHONY : src/mover.cpp.s

src/optimize.o: src/optimize.cpp.o
.PHONY : src/optimize.o

# target to build an object file
src/optimize.cpp.o:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/path_planner.dir/build.make CMakeFiles/path_planner.dir/src/optimize.cpp.o
.PHONY : src/optimize.cpp.o

src/optimize.i: src/optimize.cpp.i
.PHONY : src/optimize.i

# target to preprocess a source file
src/optimize.cpp.i:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/path_planner.dir/build.make CMakeFiles/path_planner.dir/src/optimize.cpp.i
.PHONY : src/optimize.cpp.i

src/optimize.s: src/optimize.cpp.s
.PHONY : src/optimize.s

# target to generate assembly for a file
src/optimize.cpp.s:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/path_planner.dir/build.make CMakeFiles/path_planner.dir/src/optimize.cpp.s
.PHONY : src/optimize.cpp.s

src/path_planner.o: src/path_planner.cpp.o
.PHONY : src/path_planner.o

# target to build an object file
src/path_planner.cpp.o:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/path_planner.dir/build.make CMakeFiles/path_planner.dir/src/path_planner.cpp.o
.PHONY : src/path_planner.cpp.o

src/path_planner.i: src/path_planner.cpp.i
.PHONY : src/path_planner.i

# target to preprocess a source file
src/path_planner.cpp.i:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/path_planner.dir/build.make CMakeFiles/path_planner.dir/src/path_planner.cpp.i
.PHONY : src/path_planner.cpp.i

src/path_planner.s: src/path_planner.cpp.s
.PHONY : src/path_planner.s

# target to generate assembly for a file
src/path_planner.cpp.s:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/path_planner.dir/build.make CMakeFiles/path_planner.dir/src/path_planner.cpp.s
.PHONY : src/path_planner.cpp.s

src/visualize.o: src/visualize.cpp.o
.PHONY : src/visualize.o

# target to build an object file
src/visualize.cpp.o:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/path_planner.dir/build.make CMakeFiles/path_planner.dir/src/visualize.cpp.o
.PHONY : src/visualize.cpp.o

src/visualize.i: src/visualize.cpp.i
.PHONY : src/visualize.i

# target to preprocess a source file
src/visualize.cpp.i:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/path_planner.dir/build.make CMakeFiles/path_planner.dir/src/visualize.cpp.i
.PHONY : src/visualize.cpp.i

src/visualize.s: src/visualize.cpp.s
.PHONY : src/visualize.s

# target to generate assembly for a file
src/visualize.cpp.s:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/path_planner.dir/build.make CMakeFiles/path_planner.dir/src/visualize.cpp.s
.PHONY : src/visualize.cpp.s

# Help Target
help:
	@echo "The following are some of the valid targets for this Makefile:"
	@echo "... all (the default if no target is provided)"
	@echo "... clean"
	@echo "... depend"
	@echo "... edit_cache"
	@echo "... rebuild_cache"
	@echo "... CGAL_Qt5_moc_and_resources_autogen"
	@echo "... CGAL_Qt5_moc_and_resources"
	@echo "... path_planner"
	@echo "... CGAL_Qt5_moc_and_resources_autogen/mocs_compilation.o"
	@echo "... CGAL_Qt5_moc_and_resources_autogen/mocs_compilation.i"
	@echo "... CGAL_Qt5_moc_and_resources_autogen/mocs_compilation.s"
	@echo "... qrc_CGAL.o"
	@echo "... qrc_CGAL.i"
	@echo "... qrc_CGAL.s"
	@echo "... qrc_File.o"
	@echo "... qrc_File.i"
	@echo "... qrc_File.s"
	@echo "... qrc_Input.o"
	@echo "... qrc_Input.i"
	@echo "... qrc_Input.s"
	@echo "... qrc_Triangulation_2.o"
	@echo "... qrc_Triangulation_2.i"
	@echo "... qrc_Triangulation_2.s"
	@echo "... src/mover.o"
	@echo "... src/mover.i"
	@echo "... src/mover.s"
	@echo "... src/optimize.o"
	@echo "... src/optimize.i"
	@echo "... src/optimize.s"
	@echo "... src/path_planner.o"
	@echo "... src/path_planner.i"
	@echo "... src/path_planner.s"
	@echo "... src/visualize.o"
	@echo "... src/visualize.i"
	@echo "... src/visualize.s"
.PHONY : help



#=============================================================================
# Special targets to cleanup operation of make.

# Special rule to run CMake to check the build system integrity.
# No rule that depends on this can have commands that come from listfiles
# because they might be regenerated.
cmake_check_build_system:
	$(CMAKE_COMMAND) -S$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR) --check-build-system CMakeFiles/Makefile.cmake 0
.PHONY : cmake_check_build_system
