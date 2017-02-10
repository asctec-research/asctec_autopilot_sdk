TARGET_ARCH := -mcpu=arm7tdmi-s
TARGET_ARCH += -mlittle-endian -mthumb -mthumb-interwork -mfloat-abi=soft 

DEFINES := -D__FPU_PRESENT=0 -D__FPU_USED=0 -DCORTEX_USE_FPU=0
TARGET_OPTS := -Os -fno-common -fno-strict-aliasing -finline -finline-functions-called-once -fmessage-length=0 -ffunction-sections -fdata-sections $(DEFINES)
TARGET_OPTS_D := -O0 -g3 -fno-common -fno-strict-aliasing -ffunction-sections -fdata-sections $(DEFINES)

WARNINGS = -Wall -W -Wno-sign-compare -Wwrite-strings -Wno-format #-Werror-implicit-function-declaration 
WARNINGS_CXX = -Wno-non-virtual-dtor
F_NO_EXCEPTIONS = -fno-exceptions -fno-rtti	# disabling exceptions saves code space

CC = arm-none-eabi-gcc $(TARGET_ARCH)
CFLAGS = -std=gnu11 $(WARNINGS) $(TARGET_OPTS) $(INCLUDE_DIRS)
CFLAGS_D = -std=gnu11 $(WARNINGS) $(TARGET_OPTS_D) $(INCLUDE_DIRS)

CXX = arm-none-eabi-g++ $(TARGET_ARCH)
CXXFLAGS = -std=gnu++11 $(WARNINGS) $(WARNINGS_CXX) $(TARGET_OPTS) $(INCLUDE_DIRS) $(F_NO_EXCEPTIONS)
CXXFLAGS_D = -std=gnu++11 $(WARNINGS) $(WARNINGS_CXX) $(TARGET_OPTS_D) $(INCLUDE_DIRS) $(F_NO_EXCEPTIONS)

AS = $(CC) -x assembler-with-cpp -c
ASFLAGS = $(WARNINGS) $(TARGET_OPTS) $(INCLUDE_DIRS)
ASFLAGS_D = $(WARNINGS) $(TARGET_OPTS_D) $(INCLUDE_DIRS)

LD = $(CC)	#C++ variant: LD = $(CXX)
LDFLAGS = -nostartfiles $(TARGET_ARCH) 

AR = arm-none-eabi-ar
ARFLAGS = cr

OBJCOPY = arm-none-eabi-objcopy
OBJCOPYFLAGS = -O binary

OBJDUMP = arm-none-eabi-objdump
OBJDUMPFLAGS = -h -t -S -C -D -l $(addprefix -I$(local_dir)/, $(local_src_dirs))

SIZE = arm-none-eabi-size
