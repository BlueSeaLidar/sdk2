# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

# Default target executed when no arguments are given to make.
default_target: all

.PHONY : default_target

# Allow only one "make -f Makefile2" at a time, but pass parallelism.
.NOTPARALLEL:


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
CMAKE_SOURCE_DIR = /home/pacecat/wangzn/lanhai-driver-branch

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/pacecat/wangzn/lanhai-driver-branch

#=============================================================================
# Targets provided globally by CMake.

# Special rule for the target install/strip
install/strip: preinstall
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "Installing the project stripped..."
	/usr/bin/cmake -DCMAKE_INSTALL_DO_STRIP=1 -P cmake_install.cmake
.PHONY : install/strip

# Special rule for the target install/strip
install/strip/fast: preinstall/fast
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "Installing the project stripped..."
	/usr/bin/cmake -DCMAKE_INSTALL_DO_STRIP=1 -P cmake_install.cmake
.PHONY : install/strip/fast

# Special rule for the target install
install: preinstall
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "Install the project..."
	/usr/bin/cmake -P cmake_install.cmake
.PHONY : install

# Special rule for the target install
install/fast: preinstall/fast
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "Install the project..."
	/usr/bin/cmake -P cmake_install.cmake
.PHONY : install/fast

# Special rule for the target list_install_components
list_install_components:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "Available install components are: \"Unspecified\""
.PHONY : list_install_components

# Special rule for the target list_install_components
list_install_components/fast: list_install_components

.PHONY : list_install_components/fast

# Special rule for the target rebuild_cache
rebuild_cache:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "Running CMake to regenerate build system..."
	/usr/bin/cmake -S$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR)
.PHONY : rebuild_cache

# Special rule for the target rebuild_cache
rebuild_cache/fast: rebuild_cache

.PHONY : rebuild_cache/fast

# Special rule for the target edit_cache
edit_cache:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "No interactive CMake dialog available..."
	/usr/bin/cmake -E echo No\ interactive\ CMake\ dialog\ available.
.PHONY : edit_cache

# Special rule for the target edit_cache
edit_cache/fast: edit_cache

.PHONY : edit_cache/fast

# Special rule for the target install/local
install/local: preinstall
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "Installing only the local directory..."
	/usr/bin/cmake -DCMAKE_INSTALL_LOCAL_ONLY=1 -P cmake_install.cmake
.PHONY : install/local

# Special rule for the target install/local
install/local/fast: preinstall/fast
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "Installing only the local directory..."
	/usr/bin/cmake -DCMAKE_INSTALL_LOCAL_ONLY=1 -P cmake_install.cmake
.PHONY : install/local/fast

# The main all target
all: cmake_check_build_system
	$(CMAKE_COMMAND) -E cmake_progress_start /home/pacecat/wangzn/lanhai-driver-branch/CMakeFiles /home/pacecat/wangzn/lanhai-driver-branch/CMakeFiles/progress.marks
	$(MAKE) -f CMakeFiles/Makefile2 all
	$(CMAKE_COMMAND) -E cmake_progress_start /home/pacecat/wangzn/lanhai-driver-branch/CMakeFiles 0
.PHONY : all

# The main clean target
clean:
	$(MAKE) -f CMakeFiles/Makefile2 clean
.PHONY : clean

# The main clean target
clean/fast: clean

.PHONY : clean/fast

# Prepare targets for installation.
preinstall: all
	$(MAKE) -f CMakeFiles/Makefile2 preinstall
.PHONY : preinstall

# Prepare targets for installation.
preinstall/fast:
	$(MAKE) -f CMakeFiles/Makefile2 preinstall
.PHONY : preinstall/fast

# clear depends
depend:
	$(CMAKE_COMMAND) -S$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR) --check-build-system CMakeFiles/Makefile.cmake 1
.PHONY : depend

#=============================================================================
# Target rules for targets named demo

# Build rule for target.
demo: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 demo
.PHONY : demo

# fast build rule for target.
demo/fast:
	$(MAKE) -f CMakeFiles/demo.dir/build.make CMakeFiles/demo.dir/build
.PHONY : demo/fast

example/main.o: example/main.cpp.o

.PHONY : example/main.o

# target to build an object file
example/main.cpp.o:
	$(MAKE) -f CMakeFiles/demo.dir/build.make CMakeFiles/demo.dir/example/main.cpp.o
.PHONY : example/main.cpp.o

example/main.i: example/main.cpp.i

.PHONY : example/main.i

# target to preprocess a source file
example/main.cpp.i:
	$(MAKE) -f CMakeFiles/demo.dir/build.make CMakeFiles/demo.dir/example/main.cpp.i
.PHONY : example/main.cpp.i

example/main.s: example/main.cpp.s

.PHONY : example/main.s

# target to generate assembly for a file
example/main.cpp.s:
	$(MAKE) -f CMakeFiles/demo.dir/build.make CMakeFiles/demo.dir/example/main.cpp.s
.PHONY : example/main.cpp.s

sdk/Global.o: sdk/Global.cpp.o

.PHONY : sdk/Global.o

# target to build an object file
sdk/Global.cpp.o:
	$(MAKE) -f CMakeFiles/demo.dir/build.make CMakeFiles/demo.dir/sdk/Global.cpp.o
.PHONY : sdk/Global.cpp.o

sdk/Global.i: sdk/Global.cpp.i

.PHONY : sdk/Global.i

# target to preprocess a source file
sdk/Global.cpp.i:
	$(MAKE) -f CMakeFiles/demo.dir/build.make CMakeFiles/demo.dir/sdk/Global.cpp.i
.PHONY : sdk/Global.cpp.i

sdk/Global.s: sdk/Global.cpp.s

.PHONY : sdk/Global.s

# target to generate assembly for a file
sdk/Global.cpp.s:
	$(MAKE) -f CMakeFiles/demo.dir/build.make CMakeFiles/demo.dir/sdk/Global.cpp.s
.PHONY : sdk/Global.cpp.s

sdk/LidarDataProcess.o: sdk/LidarDataProcess.cpp.o

.PHONY : sdk/LidarDataProcess.o

# target to build an object file
sdk/LidarDataProcess.cpp.o:
	$(MAKE) -f CMakeFiles/demo.dir/build.make CMakeFiles/demo.dir/sdk/LidarDataProcess.cpp.o
.PHONY : sdk/LidarDataProcess.cpp.o

sdk/LidarDataProcess.i: sdk/LidarDataProcess.cpp.i

.PHONY : sdk/LidarDataProcess.i

# target to preprocess a source file
sdk/LidarDataProcess.cpp.i:
	$(MAKE) -f CMakeFiles/demo.dir/build.make CMakeFiles/demo.dir/sdk/LidarDataProcess.cpp.i
.PHONY : sdk/LidarDataProcess.cpp.i

sdk/LidarDataProcess.s: sdk/LidarDataProcess.cpp.s

.PHONY : sdk/LidarDataProcess.s

# target to generate assembly for a file
sdk/LidarDataProcess.cpp.s:
	$(MAKE) -f CMakeFiles/demo.dir/build.make CMakeFiles/demo.dir/sdk/LidarDataProcess.cpp.s
.PHONY : sdk/LidarDataProcess.cpp.s

sdk/service/LidarCheckService_linux.o: sdk/service/LidarCheckService_linux.cpp.o

.PHONY : sdk/service/LidarCheckService_linux.o

# target to build an object file
sdk/service/LidarCheckService_linux.cpp.o:
	$(MAKE) -f CMakeFiles/demo.dir/build.make CMakeFiles/demo.dir/sdk/service/LidarCheckService_linux.cpp.o
.PHONY : sdk/service/LidarCheckService_linux.cpp.o

sdk/service/LidarCheckService_linux.i: sdk/service/LidarCheckService_linux.cpp.i

.PHONY : sdk/service/LidarCheckService_linux.i

# target to preprocess a source file
sdk/service/LidarCheckService_linux.cpp.i:
	$(MAKE) -f CMakeFiles/demo.dir/build.make CMakeFiles/demo.dir/sdk/service/LidarCheckService_linux.cpp.i
.PHONY : sdk/service/LidarCheckService_linux.cpp.i

sdk/service/LidarCheckService_linux.s: sdk/service/LidarCheckService_linux.cpp.s

.PHONY : sdk/service/LidarCheckService_linux.s

# target to generate assembly for a file
sdk/service/LidarCheckService_linux.cpp.s:
	$(MAKE) -f CMakeFiles/demo.dir/build.make CMakeFiles/demo.dir/sdk/service/LidarCheckService_linux.cpp.s
.PHONY : sdk/service/LidarCheckService_linux.cpp.s

sdk/service/LidarWebService.o: sdk/service/LidarWebService.cpp.o

.PHONY : sdk/service/LidarWebService.o

# target to build an object file
sdk/service/LidarWebService.cpp.o:
	$(MAKE) -f CMakeFiles/demo.dir/build.make CMakeFiles/demo.dir/sdk/service/LidarWebService.cpp.o
.PHONY : sdk/service/LidarWebService.cpp.o

sdk/service/LidarWebService.i: sdk/service/LidarWebService.cpp.i

.PHONY : sdk/service/LidarWebService.i

# target to preprocess a source file
sdk/service/LidarWebService.cpp.i:
	$(MAKE) -f CMakeFiles/demo.dir/build.make CMakeFiles/demo.dir/sdk/service/LidarWebService.cpp.i
.PHONY : sdk/service/LidarWebService.cpp.i

sdk/service/LidarWebService.s: sdk/service/LidarWebService.cpp.s

.PHONY : sdk/service/LidarWebService.s

# target to generate assembly for a file
sdk/service/LidarWebService.cpp.s:
	$(MAKE) -f CMakeFiles/demo.dir/build.make CMakeFiles/demo.dir/sdk/service/LidarWebService.cpp.s
.PHONY : sdk/service/LidarWebService.cpp.s

sdk/standard_interface_linux.o: sdk/standard_interface_linux.cpp.o

.PHONY : sdk/standard_interface_linux.o

# target to build an object file
sdk/standard_interface_linux.cpp.o:
	$(MAKE) -f CMakeFiles/demo.dir/build.make CMakeFiles/demo.dir/sdk/standard_interface_linux.cpp.o
.PHONY : sdk/standard_interface_linux.cpp.o

sdk/standard_interface_linux.i: sdk/standard_interface_linux.cpp.i

.PHONY : sdk/standard_interface_linux.i

# target to preprocess a source file
sdk/standard_interface_linux.cpp.i:
	$(MAKE) -f CMakeFiles/demo.dir/build.make CMakeFiles/demo.dir/sdk/standard_interface_linux.cpp.i
.PHONY : sdk/standard_interface_linux.cpp.i

sdk/standard_interface_linux.s: sdk/standard_interface_linux.cpp.s

.PHONY : sdk/standard_interface_linux.s

# target to generate assembly for a file
sdk/standard_interface_linux.cpp.s:
	$(MAKE) -f CMakeFiles/demo.dir/build.make CMakeFiles/demo.dir/sdk/standard_interface_linux.cpp.s
.PHONY : sdk/standard_interface_linux.cpp.s

sdk/third_party/cJson/cJSON.o: sdk/third_party/cJson/cJSON.c.o

.PHONY : sdk/third_party/cJson/cJSON.o

# target to build an object file
sdk/third_party/cJson/cJSON.c.o:
	$(MAKE) -f CMakeFiles/demo.dir/build.make CMakeFiles/demo.dir/sdk/third_party/cJson/cJSON.c.o
.PHONY : sdk/third_party/cJson/cJSON.c.o

sdk/third_party/cJson/cJSON.i: sdk/third_party/cJson/cJSON.c.i

.PHONY : sdk/third_party/cJson/cJSON.i

# target to preprocess a source file
sdk/third_party/cJson/cJSON.c.i:
	$(MAKE) -f CMakeFiles/demo.dir/build.make CMakeFiles/demo.dir/sdk/third_party/cJson/cJSON.c.i
.PHONY : sdk/third_party/cJson/cJSON.c.i

sdk/third_party/cJson/cJSON.s: sdk/third_party/cJson/cJSON.c.s

.PHONY : sdk/third_party/cJson/cJSON.s

# target to generate assembly for a file
sdk/third_party/cJson/cJSON.c.s:
	$(MAKE) -f CMakeFiles/demo.dir/build.make CMakeFiles/demo.dir/sdk/third_party/cJson/cJSON.c.s
.PHONY : sdk/third_party/cJson/cJSON.c.s

sdk/third_party/mongoose/mongoose.o: sdk/third_party/mongoose/mongoose.c.o

.PHONY : sdk/third_party/mongoose/mongoose.o

# target to build an object file
sdk/third_party/mongoose/mongoose.c.o:
	$(MAKE) -f CMakeFiles/demo.dir/build.make CMakeFiles/demo.dir/sdk/third_party/mongoose/mongoose.c.o
.PHONY : sdk/third_party/mongoose/mongoose.c.o

sdk/third_party/mongoose/mongoose.i: sdk/third_party/mongoose/mongoose.c.i

.PHONY : sdk/third_party/mongoose/mongoose.i

# target to preprocess a source file
sdk/third_party/mongoose/mongoose.c.i:
	$(MAKE) -f CMakeFiles/demo.dir/build.make CMakeFiles/demo.dir/sdk/third_party/mongoose/mongoose.c.i
.PHONY : sdk/third_party/mongoose/mongoose.c.i

sdk/third_party/mongoose/mongoose.s: sdk/third_party/mongoose/mongoose.c.s

.PHONY : sdk/third_party/mongoose/mongoose.s

# target to generate assembly for a file
sdk/third_party/mongoose/mongoose.c.s:
	$(MAKE) -f CMakeFiles/demo.dir/build.make CMakeFiles/demo.dir/sdk/third_party/mongoose/mongoose.c.s
.PHONY : sdk/third_party/mongoose/mongoose.c.s

sdk/uart.o: sdk/uart.c.o

.PHONY : sdk/uart.o

# target to build an object file
sdk/uart.c.o:
	$(MAKE) -f CMakeFiles/demo.dir/build.make CMakeFiles/demo.dir/sdk/uart.c.o
.PHONY : sdk/uart.c.o

sdk/uart.i: sdk/uart.c.i

.PHONY : sdk/uart.i

# target to preprocess a source file
sdk/uart.c.i:
	$(MAKE) -f CMakeFiles/demo.dir/build.make CMakeFiles/demo.dir/sdk/uart.c.i
.PHONY : sdk/uart.c.i

sdk/uart.s: sdk/uart.c.s

.PHONY : sdk/uart.s

# target to generate assembly for a file
sdk/uart.c.s:
	$(MAKE) -f CMakeFiles/demo.dir/build.make CMakeFiles/demo.dir/sdk/uart.c.s
.PHONY : sdk/uart.c.s

# Help Target
help:
	@echo "The following are some of the valid targets for this Makefile:"
	@echo "... all (the default if no target is provided)"
	@echo "... clean"
	@echo "... depend"
	@echo "... install/strip"
	@echo "... install"
	@echo "... list_install_components"
	@echo "... rebuild_cache"
	@echo "... edit_cache"
	@echo "... install/local"
	@echo "... demo"
	@echo "... example/main.o"
	@echo "... example/main.i"
	@echo "... example/main.s"
	@echo "... sdk/Global.o"
	@echo "... sdk/Global.i"
	@echo "... sdk/Global.s"
	@echo "... sdk/LidarDataProcess.o"
	@echo "... sdk/LidarDataProcess.i"
	@echo "... sdk/LidarDataProcess.s"
	@echo "... sdk/service/LidarCheckService_linux.o"
	@echo "... sdk/service/LidarCheckService_linux.i"
	@echo "... sdk/service/LidarCheckService_linux.s"
	@echo "... sdk/service/LidarWebService.o"
	@echo "... sdk/service/LidarWebService.i"
	@echo "... sdk/service/LidarWebService.s"
	@echo "... sdk/standard_interface_linux.o"
	@echo "... sdk/standard_interface_linux.i"
	@echo "... sdk/standard_interface_linux.s"
	@echo "... sdk/third_party/cJson/cJSON.o"
	@echo "... sdk/third_party/cJson/cJSON.i"
	@echo "... sdk/third_party/cJson/cJSON.s"
	@echo "... sdk/third_party/mongoose/mongoose.o"
	@echo "... sdk/third_party/mongoose/mongoose.i"
	@echo "... sdk/third_party/mongoose/mongoose.s"
	@echo "... sdk/uart.o"
	@echo "... sdk/uart.i"
	@echo "... sdk/uart.s"
.PHONY : help



#=============================================================================
# Special targets to cleanup operation of make.

# Special rule to run CMake to check the build system integrity.
# No rule that depends on this can have commands that come from listfiles
# because they might be regenerated.
cmake_check_build_system:
	$(CMAKE_COMMAND) -S$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR) --check-build-system CMakeFiles/Makefile.cmake 0
.PHONY : cmake_check_build_system

