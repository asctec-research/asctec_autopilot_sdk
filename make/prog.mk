local_build_dirs := $(addprefix $(build_dir)/$(local_run_build_dir)/,$(local_src_dirs))
local_build_dirs_d := $(addprefix $(build_dir_d)/$(local_run_build_dir)/,$(local_src_dirs))
local_src_s := $(wildcard $(addprefix $(local_dir)/,$(addsuffix /*.s,$(local_src_dirs))))
local_src_c := $(wildcard $(addprefix $(local_dir)/,$(addsuffix /*.c,$(local_src_dirs))))
local_src_cxx := $(wildcard $(addprefix $(local_dir)/,$(addsuffix /*.cpp,$(local_src_dirs))))
local_objs := $(addprefix $(build_dir)/,$(patsubst %.s,%.o,$(local_src_s)))
local_objs += $(addprefix $(build_dir)/,$(patsubst %.c,%.o,$(local_src_c)))
local_objs += $(addprefix $(build_dir)/,$(patsubst %.cpp,%.o,$(local_src_cxx)))
local_objs_d := $(addprefix $(build_dir_d)/,$(patsubst %.s,%.o,$(local_src_s)))
local_objs_d += $(addprefix $(build_dir_d)/,$(patsubst %.c,%.o,$(local_src_c)))
local_objs_d += $(addprefix $(build_dir_d)/,$(patsubst %.cpp,%.o,$(local_src_cxx)))

local_objs := $(subst $(local_dir),$(local_run_build_dir),$(local_objs))
local_objs_d := $(subst $(local_dir),$(local_run_build_dir),$(local_objs_d))

-include $(local_objs:.o=.d)
-include $(local_objs_d:.o=.d)

run_programs += $(local_run_program)
run_programs_d += $(local_run_program_d)
sources += $(local_src)
build_folders += $(local_build_dirs)
build_folders_d += $(local_build_dirs_d)

$(foreach dir, $(local_src_dirs), $(eval $(call make-objects,$(local_run_build_dir),$(local_dir),$(dir),$(local_run_flags))))

$(eval $(call make-prog,$(local_dir)))