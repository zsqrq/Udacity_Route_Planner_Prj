# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.15

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
CMAKE_COMMAND = /home/wz/Downloads/clion-2019.3.2/bin/cmake/linux/bin/cmake

# The command to remove a file.
RM = /home/wz/Downloads/clion-2019.3.2/bin/cmake/linux/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/wz/CppND-Route-Planning-Project

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/wz/CppND-Route-Planning-Project/cmake-build-debug

# Include any dependencies generated for this target.
include thirdparty/googletest/googlemock/CMakeFiles/gmock_main.dir/depend.make

# Include the progress variables for this target.
include thirdparty/googletest/googlemock/CMakeFiles/gmock_main.dir/progress.make

# Include the compile flags for this target's objects.
include thirdparty/googletest/googlemock/CMakeFiles/gmock_main.dir/flags.make

thirdparty/googletest/googlemock/CMakeFiles/gmock_main.dir/src/gmock_main.cc.o: thirdparty/googletest/googlemock/CMakeFiles/gmock_main.dir/flags.make
thirdparty/googletest/googlemock/CMakeFiles/gmock_main.dir/src/gmock_main.cc.o: ../thirdparty/googletest/googlemock/src/gmock_main.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/wz/CppND-Route-Planning-Project/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object thirdparty/googletest/googlemock/CMakeFiles/gmock_main.dir/src/gmock_main.cc.o"
	cd /home/wz/CppND-Route-Planning-Project/cmake-build-debug/thirdparty/googletest/googlemock && /usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/gmock_main.dir/src/gmock_main.cc.o -c /home/wz/CppND-Route-Planning-Project/thirdparty/googletest/googlemock/src/gmock_main.cc

thirdparty/googletest/googlemock/CMakeFiles/gmock_main.dir/src/gmock_main.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/gmock_main.dir/src/gmock_main.cc.i"
	cd /home/wz/CppND-Route-Planning-Project/cmake-build-debug/thirdparty/googletest/googlemock && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/wz/CppND-Route-Planning-Project/thirdparty/googletest/googlemock/src/gmock_main.cc > CMakeFiles/gmock_main.dir/src/gmock_main.cc.i

thirdparty/googletest/googlemock/CMakeFiles/gmock_main.dir/src/gmock_main.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/gmock_main.dir/src/gmock_main.cc.s"
	cd /home/wz/CppND-Route-Planning-Project/cmake-build-debug/thirdparty/googletest/googlemock && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/wz/CppND-Route-Planning-Project/thirdparty/googletest/googlemock/src/gmock_main.cc -o CMakeFiles/gmock_main.dir/src/gmock_main.cc.s

# Object files for target gmock_main
gmock_main_OBJECTS = \
"CMakeFiles/gmock_main.dir/src/gmock_main.cc.o"

# External object files for target gmock_main
gmock_main_EXTERNAL_OBJECTS =

lib/libgmock_maind.a: thirdparty/googletest/googlemock/CMakeFiles/gmock_main.dir/src/gmock_main.cc.o
lib/libgmock_maind.a: thirdparty/googletest/googlemock/CMakeFiles/gmock_main.dir/build.make
lib/libgmock_maind.a: thirdparty/googletest/googlemock/CMakeFiles/gmock_main.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/wz/CppND-Route-Planning-Project/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library ../../../lib/libgmock_maind.a"
	cd /home/wz/CppND-Route-Planning-Project/cmake-build-debug/thirdparty/googletest/googlemock && $(CMAKE_COMMAND) -P CMakeFiles/gmock_main.dir/cmake_clean_target.cmake
	cd /home/wz/CppND-Route-Planning-Project/cmake-build-debug/thirdparty/googletest/googlemock && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/gmock_main.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
thirdparty/googletest/googlemock/CMakeFiles/gmock_main.dir/build: lib/libgmock_maind.a

.PHONY : thirdparty/googletest/googlemock/CMakeFiles/gmock_main.dir/build

thirdparty/googletest/googlemock/CMakeFiles/gmock_main.dir/clean:
	cd /home/wz/CppND-Route-Planning-Project/cmake-build-debug/thirdparty/googletest/googlemock && $(CMAKE_COMMAND) -P CMakeFiles/gmock_main.dir/cmake_clean.cmake
.PHONY : thirdparty/googletest/googlemock/CMakeFiles/gmock_main.dir/clean

thirdparty/googletest/googlemock/CMakeFiles/gmock_main.dir/depend:
	cd /home/wz/CppND-Route-Planning-Project/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/wz/CppND-Route-Planning-Project /home/wz/CppND-Route-Planning-Project/thirdparty/googletest/googlemock /home/wz/CppND-Route-Planning-Project/cmake-build-debug /home/wz/CppND-Route-Planning-Project/cmake-build-debug/thirdparty/googletest/googlemock /home/wz/CppND-Route-Planning-Project/cmake-build-debug/thirdparty/googletest/googlemock/CMakeFiles/gmock_main.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : thirdparty/googletest/googlemock/CMakeFiles/gmock_main.dir/depend

