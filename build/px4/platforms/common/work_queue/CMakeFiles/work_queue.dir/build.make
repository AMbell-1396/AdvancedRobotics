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
CMAKE_SOURCE_DIR = /home/advrob/elfin_ws/src/Firmware

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/advrob/elfin_ws/build/px4

# Include any dependencies generated for this target.
include platforms/common/work_queue/CMakeFiles/work_queue.dir/depend.make

# Include the progress variables for this target.
include platforms/common/work_queue/CMakeFiles/work_queue.dir/progress.make

# Include the compile flags for this target's objects.
include platforms/common/work_queue/CMakeFiles/work_queue.dir/flags.make

platforms/common/work_queue/CMakeFiles/work_queue.dir/dq_addlast.c.o: platforms/common/work_queue/CMakeFiles/work_queue.dir/flags.make
platforms/common/work_queue/CMakeFiles/work_queue.dir/dq_addlast.c.o: /home/advrob/elfin_ws/src/Firmware/platforms/common/work_queue/dq_addlast.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/advrob/elfin_ws/build/px4/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object platforms/common/work_queue/CMakeFiles/work_queue.dir/dq_addlast.c.o"
	cd /home/advrob/elfin_ws/build/px4/platforms/common/work_queue && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/work_queue.dir/dq_addlast.c.o   -c /home/advrob/elfin_ws/src/Firmware/platforms/common/work_queue/dq_addlast.c

platforms/common/work_queue/CMakeFiles/work_queue.dir/dq_addlast.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/work_queue.dir/dq_addlast.c.i"
	cd /home/advrob/elfin_ws/build/px4/platforms/common/work_queue && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/advrob/elfin_ws/src/Firmware/platforms/common/work_queue/dq_addlast.c > CMakeFiles/work_queue.dir/dq_addlast.c.i

platforms/common/work_queue/CMakeFiles/work_queue.dir/dq_addlast.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/work_queue.dir/dq_addlast.c.s"
	cd /home/advrob/elfin_ws/build/px4/platforms/common/work_queue && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/advrob/elfin_ws/src/Firmware/platforms/common/work_queue/dq_addlast.c -o CMakeFiles/work_queue.dir/dq_addlast.c.s

platforms/common/work_queue/CMakeFiles/work_queue.dir/dq_rem.c.o: platforms/common/work_queue/CMakeFiles/work_queue.dir/flags.make
platforms/common/work_queue/CMakeFiles/work_queue.dir/dq_rem.c.o: /home/advrob/elfin_ws/src/Firmware/platforms/common/work_queue/dq_rem.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/advrob/elfin_ws/build/px4/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building C object platforms/common/work_queue/CMakeFiles/work_queue.dir/dq_rem.c.o"
	cd /home/advrob/elfin_ws/build/px4/platforms/common/work_queue && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/work_queue.dir/dq_rem.c.o   -c /home/advrob/elfin_ws/src/Firmware/platforms/common/work_queue/dq_rem.c

platforms/common/work_queue/CMakeFiles/work_queue.dir/dq_rem.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/work_queue.dir/dq_rem.c.i"
	cd /home/advrob/elfin_ws/build/px4/platforms/common/work_queue && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/advrob/elfin_ws/src/Firmware/platforms/common/work_queue/dq_rem.c > CMakeFiles/work_queue.dir/dq_rem.c.i

platforms/common/work_queue/CMakeFiles/work_queue.dir/dq_rem.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/work_queue.dir/dq_rem.c.s"
	cd /home/advrob/elfin_ws/build/px4/platforms/common/work_queue && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/advrob/elfin_ws/src/Firmware/platforms/common/work_queue/dq_rem.c -o CMakeFiles/work_queue.dir/dq_rem.c.s

platforms/common/work_queue/CMakeFiles/work_queue.dir/dq_remfirst.c.o: platforms/common/work_queue/CMakeFiles/work_queue.dir/flags.make
platforms/common/work_queue/CMakeFiles/work_queue.dir/dq_remfirst.c.o: /home/advrob/elfin_ws/src/Firmware/platforms/common/work_queue/dq_remfirst.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/advrob/elfin_ws/build/px4/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building C object platforms/common/work_queue/CMakeFiles/work_queue.dir/dq_remfirst.c.o"
	cd /home/advrob/elfin_ws/build/px4/platforms/common/work_queue && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/work_queue.dir/dq_remfirst.c.o   -c /home/advrob/elfin_ws/src/Firmware/platforms/common/work_queue/dq_remfirst.c

platforms/common/work_queue/CMakeFiles/work_queue.dir/dq_remfirst.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/work_queue.dir/dq_remfirst.c.i"
	cd /home/advrob/elfin_ws/build/px4/platforms/common/work_queue && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/advrob/elfin_ws/src/Firmware/platforms/common/work_queue/dq_remfirst.c > CMakeFiles/work_queue.dir/dq_remfirst.c.i

platforms/common/work_queue/CMakeFiles/work_queue.dir/dq_remfirst.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/work_queue.dir/dq_remfirst.c.s"
	cd /home/advrob/elfin_ws/build/px4/platforms/common/work_queue && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/advrob/elfin_ws/src/Firmware/platforms/common/work_queue/dq_remfirst.c -o CMakeFiles/work_queue.dir/dq_remfirst.c.s

platforms/common/work_queue/CMakeFiles/work_queue.dir/hrt_queue.c.o: platforms/common/work_queue/CMakeFiles/work_queue.dir/flags.make
platforms/common/work_queue/CMakeFiles/work_queue.dir/hrt_queue.c.o: /home/advrob/elfin_ws/src/Firmware/platforms/common/work_queue/hrt_queue.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/advrob/elfin_ws/build/px4/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building C object platforms/common/work_queue/CMakeFiles/work_queue.dir/hrt_queue.c.o"
	cd /home/advrob/elfin_ws/build/px4/platforms/common/work_queue && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/work_queue.dir/hrt_queue.c.o   -c /home/advrob/elfin_ws/src/Firmware/platforms/common/work_queue/hrt_queue.c

platforms/common/work_queue/CMakeFiles/work_queue.dir/hrt_queue.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/work_queue.dir/hrt_queue.c.i"
	cd /home/advrob/elfin_ws/build/px4/platforms/common/work_queue && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/advrob/elfin_ws/src/Firmware/platforms/common/work_queue/hrt_queue.c > CMakeFiles/work_queue.dir/hrt_queue.c.i

platforms/common/work_queue/CMakeFiles/work_queue.dir/hrt_queue.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/work_queue.dir/hrt_queue.c.s"
	cd /home/advrob/elfin_ws/build/px4/platforms/common/work_queue && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/advrob/elfin_ws/src/Firmware/platforms/common/work_queue/hrt_queue.c -o CMakeFiles/work_queue.dir/hrt_queue.c.s

platforms/common/work_queue/CMakeFiles/work_queue.dir/hrt_thread.c.o: platforms/common/work_queue/CMakeFiles/work_queue.dir/flags.make
platforms/common/work_queue/CMakeFiles/work_queue.dir/hrt_thread.c.o: /home/advrob/elfin_ws/src/Firmware/platforms/common/work_queue/hrt_thread.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/advrob/elfin_ws/build/px4/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building C object platforms/common/work_queue/CMakeFiles/work_queue.dir/hrt_thread.c.o"
	cd /home/advrob/elfin_ws/build/px4/platforms/common/work_queue && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/work_queue.dir/hrt_thread.c.o   -c /home/advrob/elfin_ws/src/Firmware/platforms/common/work_queue/hrt_thread.c

platforms/common/work_queue/CMakeFiles/work_queue.dir/hrt_thread.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/work_queue.dir/hrt_thread.c.i"
	cd /home/advrob/elfin_ws/build/px4/platforms/common/work_queue && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/advrob/elfin_ws/src/Firmware/platforms/common/work_queue/hrt_thread.c > CMakeFiles/work_queue.dir/hrt_thread.c.i

platforms/common/work_queue/CMakeFiles/work_queue.dir/hrt_thread.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/work_queue.dir/hrt_thread.c.s"
	cd /home/advrob/elfin_ws/build/px4/platforms/common/work_queue && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/advrob/elfin_ws/src/Firmware/platforms/common/work_queue/hrt_thread.c -o CMakeFiles/work_queue.dir/hrt_thread.c.s

platforms/common/work_queue/CMakeFiles/work_queue.dir/hrt_work_cancel.c.o: platforms/common/work_queue/CMakeFiles/work_queue.dir/flags.make
platforms/common/work_queue/CMakeFiles/work_queue.dir/hrt_work_cancel.c.o: /home/advrob/elfin_ws/src/Firmware/platforms/common/work_queue/hrt_work_cancel.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/advrob/elfin_ws/build/px4/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building C object platforms/common/work_queue/CMakeFiles/work_queue.dir/hrt_work_cancel.c.o"
	cd /home/advrob/elfin_ws/build/px4/platforms/common/work_queue && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/work_queue.dir/hrt_work_cancel.c.o   -c /home/advrob/elfin_ws/src/Firmware/platforms/common/work_queue/hrt_work_cancel.c

platforms/common/work_queue/CMakeFiles/work_queue.dir/hrt_work_cancel.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/work_queue.dir/hrt_work_cancel.c.i"
	cd /home/advrob/elfin_ws/build/px4/platforms/common/work_queue && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/advrob/elfin_ws/src/Firmware/platforms/common/work_queue/hrt_work_cancel.c > CMakeFiles/work_queue.dir/hrt_work_cancel.c.i

platforms/common/work_queue/CMakeFiles/work_queue.dir/hrt_work_cancel.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/work_queue.dir/hrt_work_cancel.c.s"
	cd /home/advrob/elfin_ws/build/px4/platforms/common/work_queue && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/advrob/elfin_ws/src/Firmware/platforms/common/work_queue/hrt_work_cancel.c -o CMakeFiles/work_queue.dir/hrt_work_cancel.c.s

platforms/common/work_queue/CMakeFiles/work_queue.dir/queue.c.o: platforms/common/work_queue/CMakeFiles/work_queue.dir/flags.make
platforms/common/work_queue/CMakeFiles/work_queue.dir/queue.c.o: /home/advrob/elfin_ws/src/Firmware/platforms/common/work_queue/queue.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/advrob/elfin_ws/build/px4/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building C object platforms/common/work_queue/CMakeFiles/work_queue.dir/queue.c.o"
	cd /home/advrob/elfin_ws/build/px4/platforms/common/work_queue && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/work_queue.dir/queue.c.o   -c /home/advrob/elfin_ws/src/Firmware/platforms/common/work_queue/queue.c

platforms/common/work_queue/CMakeFiles/work_queue.dir/queue.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/work_queue.dir/queue.c.i"
	cd /home/advrob/elfin_ws/build/px4/platforms/common/work_queue && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/advrob/elfin_ws/src/Firmware/platforms/common/work_queue/queue.c > CMakeFiles/work_queue.dir/queue.c.i

platforms/common/work_queue/CMakeFiles/work_queue.dir/queue.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/work_queue.dir/queue.c.s"
	cd /home/advrob/elfin_ws/build/px4/platforms/common/work_queue && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/advrob/elfin_ws/src/Firmware/platforms/common/work_queue/queue.c -o CMakeFiles/work_queue.dir/queue.c.s

platforms/common/work_queue/CMakeFiles/work_queue.dir/sq_addafter.c.o: platforms/common/work_queue/CMakeFiles/work_queue.dir/flags.make
platforms/common/work_queue/CMakeFiles/work_queue.dir/sq_addafter.c.o: /home/advrob/elfin_ws/src/Firmware/platforms/common/work_queue/sq_addafter.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/advrob/elfin_ws/build/px4/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Building C object platforms/common/work_queue/CMakeFiles/work_queue.dir/sq_addafter.c.o"
	cd /home/advrob/elfin_ws/build/px4/platforms/common/work_queue && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/work_queue.dir/sq_addafter.c.o   -c /home/advrob/elfin_ws/src/Firmware/platforms/common/work_queue/sq_addafter.c

platforms/common/work_queue/CMakeFiles/work_queue.dir/sq_addafter.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/work_queue.dir/sq_addafter.c.i"
	cd /home/advrob/elfin_ws/build/px4/platforms/common/work_queue && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/advrob/elfin_ws/src/Firmware/platforms/common/work_queue/sq_addafter.c > CMakeFiles/work_queue.dir/sq_addafter.c.i

platforms/common/work_queue/CMakeFiles/work_queue.dir/sq_addafter.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/work_queue.dir/sq_addafter.c.s"
	cd /home/advrob/elfin_ws/build/px4/platforms/common/work_queue && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/advrob/elfin_ws/src/Firmware/platforms/common/work_queue/sq_addafter.c -o CMakeFiles/work_queue.dir/sq_addafter.c.s

platforms/common/work_queue/CMakeFiles/work_queue.dir/sq_addlast.c.o: platforms/common/work_queue/CMakeFiles/work_queue.dir/flags.make
platforms/common/work_queue/CMakeFiles/work_queue.dir/sq_addlast.c.o: /home/advrob/elfin_ws/src/Firmware/platforms/common/work_queue/sq_addlast.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/advrob/elfin_ws/build/px4/CMakeFiles --progress-num=$(CMAKE_PROGRESS_9) "Building C object platforms/common/work_queue/CMakeFiles/work_queue.dir/sq_addlast.c.o"
	cd /home/advrob/elfin_ws/build/px4/platforms/common/work_queue && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/work_queue.dir/sq_addlast.c.o   -c /home/advrob/elfin_ws/src/Firmware/platforms/common/work_queue/sq_addlast.c

platforms/common/work_queue/CMakeFiles/work_queue.dir/sq_addlast.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/work_queue.dir/sq_addlast.c.i"
	cd /home/advrob/elfin_ws/build/px4/platforms/common/work_queue && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/advrob/elfin_ws/src/Firmware/platforms/common/work_queue/sq_addlast.c > CMakeFiles/work_queue.dir/sq_addlast.c.i

platforms/common/work_queue/CMakeFiles/work_queue.dir/sq_addlast.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/work_queue.dir/sq_addlast.c.s"
	cd /home/advrob/elfin_ws/build/px4/platforms/common/work_queue && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/advrob/elfin_ws/src/Firmware/platforms/common/work_queue/sq_addlast.c -o CMakeFiles/work_queue.dir/sq_addlast.c.s

platforms/common/work_queue/CMakeFiles/work_queue.dir/sq_remfirst.c.o: platforms/common/work_queue/CMakeFiles/work_queue.dir/flags.make
platforms/common/work_queue/CMakeFiles/work_queue.dir/sq_remfirst.c.o: /home/advrob/elfin_ws/src/Firmware/platforms/common/work_queue/sq_remfirst.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/advrob/elfin_ws/build/px4/CMakeFiles --progress-num=$(CMAKE_PROGRESS_10) "Building C object platforms/common/work_queue/CMakeFiles/work_queue.dir/sq_remfirst.c.o"
	cd /home/advrob/elfin_ws/build/px4/platforms/common/work_queue && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/work_queue.dir/sq_remfirst.c.o   -c /home/advrob/elfin_ws/src/Firmware/platforms/common/work_queue/sq_remfirst.c

platforms/common/work_queue/CMakeFiles/work_queue.dir/sq_remfirst.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/work_queue.dir/sq_remfirst.c.i"
	cd /home/advrob/elfin_ws/build/px4/platforms/common/work_queue && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/advrob/elfin_ws/src/Firmware/platforms/common/work_queue/sq_remfirst.c > CMakeFiles/work_queue.dir/sq_remfirst.c.i

platforms/common/work_queue/CMakeFiles/work_queue.dir/sq_remfirst.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/work_queue.dir/sq_remfirst.c.s"
	cd /home/advrob/elfin_ws/build/px4/platforms/common/work_queue && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/advrob/elfin_ws/src/Firmware/platforms/common/work_queue/sq_remfirst.c -o CMakeFiles/work_queue.dir/sq_remfirst.c.s

platforms/common/work_queue/CMakeFiles/work_queue.dir/work_cancel.c.o: platforms/common/work_queue/CMakeFiles/work_queue.dir/flags.make
platforms/common/work_queue/CMakeFiles/work_queue.dir/work_cancel.c.o: /home/advrob/elfin_ws/src/Firmware/platforms/common/work_queue/work_cancel.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/advrob/elfin_ws/build/px4/CMakeFiles --progress-num=$(CMAKE_PROGRESS_11) "Building C object platforms/common/work_queue/CMakeFiles/work_queue.dir/work_cancel.c.o"
	cd /home/advrob/elfin_ws/build/px4/platforms/common/work_queue && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/work_queue.dir/work_cancel.c.o   -c /home/advrob/elfin_ws/src/Firmware/platforms/common/work_queue/work_cancel.c

platforms/common/work_queue/CMakeFiles/work_queue.dir/work_cancel.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/work_queue.dir/work_cancel.c.i"
	cd /home/advrob/elfin_ws/build/px4/platforms/common/work_queue && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/advrob/elfin_ws/src/Firmware/platforms/common/work_queue/work_cancel.c > CMakeFiles/work_queue.dir/work_cancel.c.i

platforms/common/work_queue/CMakeFiles/work_queue.dir/work_cancel.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/work_queue.dir/work_cancel.c.s"
	cd /home/advrob/elfin_ws/build/px4/platforms/common/work_queue && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/advrob/elfin_ws/src/Firmware/platforms/common/work_queue/work_cancel.c -o CMakeFiles/work_queue.dir/work_cancel.c.s

platforms/common/work_queue/CMakeFiles/work_queue.dir/work_lock.c.o: platforms/common/work_queue/CMakeFiles/work_queue.dir/flags.make
platforms/common/work_queue/CMakeFiles/work_queue.dir/work_lock.c.o: /home/advrob/elfin_ws/src/Firmware/platforms/common/work_queue/work_lock.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/advrob/elfin_ws/build/px4/CMakeFiles --progress-num=$(CMAKE_PROGRESS_12) "Building C object platforms/common/work_queue/CMakeFiles/work_queue.dir/work_lock.c.o"
	cd /home/advrob/elfin_ws/build/px4/platforms/common/work_queue && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/work_queue.dir/work_lock.c.o   -c /home/advrob/elfin_ws/src/Firmware/platforms/common/work_queue/work_lock.c

platforms/common/work_queue/CMakeFiles/work_queue.dir/work_lock.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/work_queue.dir/work_lock.c.i"
	cd /home/advrob/elfin_ws/build/px4/platforms/common/work_queue && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/advrob/elfin_ws/src/Firmware/platforms/common/work_queue/work_lock.c > CMakeFiles/work_queue.dir/work_lock.c.i

platforms/common/work_queue/CMakeFiles/work_queue.dir/work_lock.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/work_queue.dir/work_lock.c.s"
	cd /home/advrob/elfin_ws/build/px4/platforms/common/work_queue && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/advrob/elfin_ws/src/Firmware/platforms/common/work_queue/work_lock.c -o CMakeFiles/work_queue.dir/work_lock.c.s

platforms/common/work_queue/CMakeFiles/work_queue.dir/work_queue.c.o: platforms/common/work_queue/CMakeFiles/work_queue.dir/flags.make
platforms/common/work_queue/CMakeFiles/work_queue.dir/work_queue.c.o: /home/advrob/elfin_ws/src/Firmware/platforms/common/work_queue/work_queue.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/advrob/elfin_ws/build/px4/CMakeFiles --progress-num=$(CMAKE_PROGRESS_13) "Building C object platforms/common/work_queue/CMakeFiles/work_queue.dir/work_queue.c.o"
	cd /home/advrob/elfin_ws/build/px4/platforms/common/work_queue && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/work_queue.dir/work_queue.c.o   -c /home/advrob/elfin_ws/src/Firmware/platforms/common/work_queue/work_queue.c

platforms/common/work_queue/CMakeFiles/work_queue.dir/work_queue.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/work_queue.dir/work_queue.c.i"
	cd /home/advrob/elfin_ws/build/px4/platforms/common/work_queue && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/advrob/elfin_ws/src/Firmware/platforms/common/work_queue/work_queue.c > CMakeFiles/work_queue.dir/work_queue.c.i

platforms/common/work_queue/CMakeFiles/work_queue.dir/work_queue.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/work_queue.dir/work_queue.c.s"
	cd /home/advrob/elfin_ws/build/px4/platforms/common/work_queue && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/advrob/elfin_ws/src/Firmware/platforms/common/work_queue/work_queue.c -o CMakeFiles/work_queue.dir/work_queue.c.s

platforms/common/work_queue/CMakeFiles/work_queue.dir/work_thread.c.o: platforms/common/work_queue/CMakeFiles/work_queue.dir/flags.make
platforms/common/work_queue/CMakeFiles/work_queue.dir/work_thread.c.o: /home/advrob/elfin_ws/src/Firmware/platforms/common/work_queue/work_thread.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/advrob/elfin_ws/build/px4/CMakeFiles --progress-num=$(CMAKE_PROGRESS_14) "Building C object platforms/common/work_queue/CMakeFiles/work_queue.dir/work_thread.c.o"
	cd /home/advrob/elfin_ws/build/px4/platforms/common/work_queue && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/work_queue.dir/work_thread.c.o   -c /home/advrob/elfin_ws/src/Firmware/platforms/common/work_queue/work_thread.c

platforms/common/work_queue/CMakeFiles/work_queue.dir/work_thread.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/work_queue.dir/work_thread.c.i"
	cd /home/advrob/elfin_ws/build/px4/platforms/common/work_queue && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/advrob/elfin_ws/src/Firmware/platforms/common/work_queue/work_thread.c > CMakeFiles/work_queue.dir/work_thread.c.i

platforms/common/work_queue/CMakeFiles/work_queue.dir/work_thread.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/work_queue.dir/work_thread.c.s"
	cd /home/advrob/elfin_ws/build/px4/platforms/common/work_queue && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/advrob/elfin_ws/src/Firmware/platforms/common/work_queue/work_thread.c -o CMakeFiles/work_queue.dir/work_thread.c.s

# Object files for target work_queue
work_queue_OBJECTS = \
"CMakeFiles/work_queue.dir/dq_addlast.c.o" \
"CMakeFiles/work_queue.dir/dq_rem.c.o" \
"CMakeFiles/work_queue.dir/dq_remfirst.c.o" \
"CMakeFiles/work_queue.dir/hrt_queue.c.o" \
"CMakeFiles/work_queue.dir/hrt_thread.c.o" \
"CMakeFiles/work_queue.dir/hrt_work_cancel.c.o" \
"CMakeFiles/work_queue.dir/queue.c.o" \
"CMakeFiles/work_queue.dir/sq_addafter.c.o" \
"CMakeFiles/work_queue.dir/sq_addlast.c.o" \
"CMakeFiles/work_queue.dir/sq_remfirst.c.o" \
"CMakeFiles/work_queue.dir/work_cancel.c.o" \
"CMakeFiles/work_queue.dir/work_lock.c.o" \
"CMakeFiles/work_queue.dir/work_queue.c.o" \
"CMakeFiles/work_queue.dir/work_thread.c.o"

# External object files for target work_queue
work_queue_EXTERNAL_OBJECTS =

/home/advrob/elfin_ws/devel/.private/px4/lib/libwork_queue.a: platforms/common/work_queue/CMakeFiles/work_queue.dir/dq_addlast.c.o
/home/advrob/elfin_ws/devel/.private/px4/lib/libwork_queue.a: platforms/common/work_queue/CMakeFiles/work_queue.dir/dq_rem.c.o
/home/advrob/elfin_ws/devel/.private/px4/lib/libwork_queue.a: platforms/common/work_queue/CMakeFiles/work_queue.dir/dq_remfirst.c.o
/home/advrob/elfin_ws/devel/.private/px4/lib/libwork_queue.a: platforms/common/work_queue/CMakeFiles/work_queue.dir/hrt_queue.c.o
/home/advrob/elfin_ws/devel/.private/px4/lib/libwork_queue.a: platforms/common/work_queue/CMakeFiles/work_queue.dir/hrt_thread.c.o
/home/advrob/elfin_ws/devel/.private/px4/lib/libwork_queue.a: platforms/common/work_queue/CMakeFiles/work_queue.dir/hrt_work_cancel.c.o
/home/advrob/elfin_ws/devel/.private/px4/lib/libwork_queue.a: platforms/common/work_queue/CMakeFiles/work_queue.dir/queue.c.o
/home/advrob/elfin_ws/devel/.private/px4/lib/libwork_queue.a: platforms/common/work_queue/CMakeFiles/work_queue.dir/sq_addafter.c.o
/home/advrob/elfin_ws/devel/.private/px4/lib/libwork_queue.a: platforms/common/work_queue/CMakeFiles/work_queue.dir/sq_addlast.c.o
/home/advrob/elfin_ws/devel/.private/px4/lib/libwork_queue.a: platforms/common/work_queue/CMakeFiles/work_queue.dir/sq_remfirst.c.o
/home/advrob/elfin_ws/devel/.private/px4/lib/libwork_queue.a: platforms/common/work_queue/CMakeFiles/work_queue.dir/work_cancel.c.o
/home/advrob/elfin_ws/devel/.private/px4/lib/libwork_queue.a: platforms/common/work_queue/CMakeFiles/work_queue.dir/work_lock.c.o
/home/advrob/elfin_ws/devel/.private/px4/lib/libwork_queue.a: platforms/common/work_queue/CMakeFiles/work_queue.dir/work_queue.c.o
/home/advrob/elfin_ws/devel/.private/px4/lib/libwork_queue.a: platforms/common/work_queue/CMakeFiles/work_queue.dir/work_thread.c.o
/home/advrob/elfin_ws/devel/.private/px4/lib/libwork_queue.a: platforms/common/work_queue/CMakeFiles/work_queue.dir/build.make
/home/advrob/elfin_ws/devel/.private/px4/lib/libwork_queue.a: platforms/common/work_queue/CMakeFiles/work_queue.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/advrob/elfin_ws/build/px4/CMakeFiles --progress-num=$(CMAKE_PROGRESS_15) "Linking C static library /home/advrob/elfin_ws/devel/.private/px4/lib/libwork_queue.a"
	cd /home/advrob/elfin_ws/build/px4/platforms/common/work_queue && $(CMAKE_COMMAND) -P CMakeFiles/work_queue.dir/cmake_clean_target.cmake
	cd /home/advrob/elfin_ws/build/px4/platforms/common/work_queue && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/work_queue.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
platforms/common/work_queue/CMakeFiles/work_queue.dir/build: /home/advrob/elfin_ws/devel/.private/px4/lib/libwork_queue.a

.PHONY : platforms/common/work_queue/CMakeFiles/work_queue.dir/build

platforms/common/work_queue/CMakeFiles/work_queue.dir/clean:
	cd /home/advrob/elfin_ws/build/px4/platforms/common/work_queue && $(CMAKE_COMMAND) -P CMakeFiles/work_queue.dir/cmake_clean.cmake
.PHONY : platforms/common/work_queue/CMakeFiles/work_queue.dir/clean

platforms/common/work_queue/CMakeFiles/work_queue.dir/depend:
	cd /home/advrob/elfin_ws/build/px4 && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/advrob/elfin_ws/src/Firmware /home/advrob/elfin_ws/src/Firmware/platforms/common/work_queue /home/advrob/elfin_ws/build/px4 /home/advrob/elfin_ws/build/px4/platforms/common/work_queue /home/advrob/elfin_ws/build/px4/platforms/common/work_queue/CMakeFiles/work_queue.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : platforms/common/work_queue/CMakeFiles/work_queue.dir/depend
