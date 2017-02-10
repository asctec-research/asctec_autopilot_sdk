run_programs :=
libraries :=
run_programs_d :=
libraries_d :=
sources :=
build_folders :=
build_folders_d :=

build_dir := release
build_dir_d := debug

.PHONY: all folders folders_d release debug release-run \
debug-run dist-clean release-dist-clean debug-dist-clean flash

all:

export

ifeq ($(OS),Windows_NT)
MKDIR := bin/mkdir.exe
RM := bin/rm.exe
OPENOCD := openocd.exe
else
MKDIR := mkdir
RM := rm
OPENOCD := openocd
endif

define make-prog
$(local_run_program): $(local_objs) $(local_libs) 
	$(LD) -Wl,-Map=$$(patsubst %.elf,%.map,$$@),--gc-sections -T$(1)/run.ld \
	 --specs=nano.specs $(LDFLAGS) $(local_objs) -Wl,--start-group -lc -lm $(local_libs) -Wl,--end-group $(local_ld_flags) -o $$@
	$(OBJCOPY) $(OBJCOPYFLAGS) $$@ $$(patsubst %.elf,%.bin,$$@)
#	$(OBJDUMP) $(OBJDUMPFLAGS) $$@ > $$(patsubst %.elf,%.lss,$$@)

$(local_run_program_d): $(local_objs_d) $(local_libs_d) 
	$(LD) -Wl,-Map=$$(patsubst %.elf,%.map,$$@),--gc-sections -T$(1)/run.ld \
	 --specs=nano.specs $(LDFLAGS) $(local_objs_d) -Wl,--start-group -lc -lm $(local_libs_d) -Wl,--end-group $(local_ld_flags) -o $$@
	$(OBJCOPY) $(OBJCOPYFLAGS) $$@ $$(patsubst %.elf,%.bin,$$@)
endef

# $(call make-objects-c, local_build_dir, local_dir, src_dir, params)
define make-objects
$(build_dir)/$(1)/$(3)/%.o: $(2)/$(3)/%.s
	$(AS) $(ASFLAGS) $(local_flags) $(4) -c $$< -o $$@
	@$(AS) -MM -MP -MT $$@ -MF $(build_dir)/$(1)/$(3)/$$*.d $(ASFLAGS) $(local_flags) $(4) $$<
	
$(build_dir)/$(1)/$(3)/%.o: $(2)/$(3)/%.c
	$(CC) $(CFLAGS) $(local_flags) $(4) -c $$< -o $$@
	@$(CC) -MM -MP -MT $$@ -MF $(build_dir)/$(1)/$(3)/$$*.d $(CFLAGS) $(local_flags) $(4) $$<

$(build_dir)/$(1)/$(3)/%.o: $(2)/$(3)/%.cpp
	$(CXX) $(CXXFLAGS) $(local_flags) $(4) -c $$< -o $$@
	@$(CXX) -MM -MP -MT $$@ -MF $(build_dir)/$(1)/$(3)/$$*.d $(CXXFLAGS) $(local_flags) $(4) $$<

$(build_dir_d)/$(1)/$(3)/%.o: $(2)/$(3)/%.s
	$(AS) $(ASFLAGS_D) $(local_flags_d) $(4) -c $$< -o $$@
	@$(AS) -MM -MP -MT $$@ -MF $(build_dir_d)/$(1)/$(3)/$$*.d $(ASFLAGS_D) $(local_flags_d) $(4) $$<

$(build_dir_d)/$(1)/$(3)/%.o: $(2)/$(3)/%.c
	$(CC) $(CFLAGS_D) $(local_flags_d) $(4) -c $$< -o $$@
	@$(CC) -MM -MP -MT $$@ -MF $(build_dir_d)/$(1)/$(3)/$$*.d $(CFLAGS_D) $(local_flags_d) $(4) $$<

$(build_dir_d)/$(1)/$(3)/%.o: $(2)/$(3)/%.cpp
	$(CXX) $(CXXFLAGS_D) $(local_flags_d) $(4) -c $$< -o $$@
	@$(CXX) -MM -MP -MT $$@ -MF $(build_dir_d)/$(1)/$(3)/$$*.d $(CXXFLAGS_D) $(local_flags_d) $(4) $$<
endef

include make/compiler.mk
include src/ap.mk

all: release debug

folders:
	$(MKDIR) -p $(build_folders)

folders_d:
	$(MKDIR) -p $(build_folders_d)

release: folders $(libraries) $(run_programs)
	$(SIZE) $(run_programs)

debug: folders_d $(libraries_d) $(run_programs_d)
	$(SIZE) $(run_programs_d)

clean: dist-clean

dist-clean: release-dist-clean debug-dist-clean

release-dist-clean:
	-$(RM) -rf $(build_dir)
	-$(RM) -rf $(libraries) $(run_programs)
	
debug-dist-clean:
	-$(RM) -rf $(build_dir_d)
	-$(RM) -rf $(libraries_d) $(run_programs_d)
	
flash: release
	$(OPENOCD) -f openocd/lpc2xxx_asctec.cfg -f openocd/flash-release.cfg
