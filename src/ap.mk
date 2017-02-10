local_name := ap
local_dir := src
local_run_build_dir := run/$(local_name)
local_run_program := $(build_dir)/run/$(local_name).elf
local_run_program_d := $(build_dir_d)/run/$(local_name).elf

local_src_dirs := . win_arm hal util examples
local_include_dirs := -I src
local_include_dirs += -I src/win_arm -I deps/asctec_uav_msgs/include
local_defines := -D__VERSION_MAJOR=4 -D__VERSION_MINOR=0 -D__BUILD_CONFIG=0x00 -DROM_RUN
local_flags := $(local_include_dirs) $(local_defines)
local_flags_d := $(local_include_dirs) $(local_defines) -DDEBUG
local_run_flags := 
local_libs := 
local_libs_d :=
local_ld_flags :=  
#local_ld_flags := -u _printf_float -u _scanf_float

include make/prog.mk