#
# Makefile
#
# Copyright (c) 2012 - 2014 Robert Giseburt
# Copyright (c) 2013 - 2014 Alden S. Hart Jr.
#
#	This file is part of the Motate Library.
#
#	This file ("the software") is free software: you can redistribute it and/or modify
#	it under the terms of the GNU General Public License, version 2 as published by the
#	Free Software Foundation. You should have received a copy of the GNU General Public
#	License, version 2 along with the software. If not, see <http://www.gnu.org/licenses/>.
#
#	As a special exception, you may use this file as part of a software library without
#	restriction. Specifically, if other files instantiate templates or use macros or
#	inline functions from this file, or you compile this file and link it with  other
#	files to produce an executable, this file does not by itself cause the resulting
#	executable to be covered by the GNU General Public License. This exception does not
#	however invalidate any other reasons why the executable file might be covered by the
#	GNU General Public License.
#
#	THE SOFTWARE IS DISTRIBUTED IN THE HOPE THAT IT WILL BE USEFUL, BUT WITHOUT ANY
#	WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
#	OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT
#	SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
#	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF
#	OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
#

##############################################################################################
# Start of default section
#

PROJECT  = TinyG2

MOTATE_PATH ?= motate
export MOTATE_PATH

TOOLS_PATH ?= ../Tools

SOURCE_DIRS ?= . fatfs
SOURCE_DIRS += ${MOTATE_PATH}

FIRST_LINK_SOURCES +=

# List all user directories here
USER_INCLUDE_DIRS += $(SOURCE_DIRS)

# List the user directory to look for the libraries here
USER_LIB_DIRS +=

# List all user libraries here
USER_LIBS +=

# Define optimisation level here
#OPTIMIZATION ?= 0
OPTIMIZATION ?= s
#OPTIMIZATION ?= 3

CFLAGS   +=
CPPFLAGS +=

#
# End of configuration section
##############################################################################################

##############################################################################################
# Start of configuration section
#

# ---------------------------------------------------------------------------------------
# C Flags (NOT CPP Flags)

# Configure warnings:
CFLAGS += -Wall -Wchar-subscripts -Wcomment -Wformat=2 -Wimplicit-int
CFLAGS += -Werror-implicit-function-declaration -Wmain -Wparentheses
CFLAGS += -Wsequence-point -Wreturn-type -Wswitch -Wtrigraphs -Wunused
CFLAGS += -Wuninitialized -Wno-unknown-pragmas -Wfloat-equal -Wundef
CFLAGS += -Wshadow -Wpointer-arith -Wbad-function-cast -Wwrite-strings
CFLAGS += -Wsign-compare -Waggregate-return
#CFLAGS += -Wstrict-prototypes
#CFLAGS += -Wmissing-prototypes -Wmissing-declarations
CFLAGS += -Wformat -Wmissing-format-attribute -Wno-deprecated-declarations
CFLAGS += -Wredundant-decls -Wnested-externs -Winline -Wno-long-long
CFLAGS += -Wunreachable-code
#CFLAGS += -Wcast-align -Wpacked
#CFLAGS += -Wmissing-noreturn
#CFLAGS += -Wconversion

# Turn off printf() format strings. We use late bound FLASH and RAM format strings and this causes warnings
CFLAGS += -Wno-format-nonliteral -Wno-format-security

# ---------------------------------------------------------------------------------------
# CPP Flags

CPPFLAGS += -Wall -Wchar-subscripts -Wcomment -Wformat=2
CPPFLAGS += -Wmain -Wparentheses -Wunreachable-code
#CPPFLAGS += -Wcast-align
CPPFLAGS += -Wsequence-point -Wreturn-type -Wswitch -Wtrigraphs -Wunused
CPPFLAGS += -Wuninitialized -Wno-unknown-pragmas -Wno-float-equal -Wundef
CPPFLAGS += -Wshadow -Wpointer-arith -Wwrite-strings
#CPPFLAGS += -Wsign-compare -Waggregate-return -Wmissing-declarations
CPPFLAGS += -Wformat -Wmissing-format-attribute -Wno-deprecated-declarations
CPPFLAGS += -Wredundant-decls -Wlong-long
#CPPFLAGS += -Winline
#CPPFLAGS += -Wmissing-noreturn -Wpacked
#CPPFLAGS += -Wconversion
CPPFLAGS += -Wno-strict-aliasing

# Turn off printf() format strings. We use late bound FLASH and RAM format strings and this causes warnings
CPPFLAGS += -Wno-format-nonliteral -Wno-format-security

#
# End of configuration section
##############################################################################################

##############################################################################################
# Start of build support section
#

# Default is to compile more quielty.
# To show actual commands: make [options] VERBOSE=1
VERBOSE ?= 0
COLOR ?= 0

ifeq ($(VERBOSE),0)
QUIET := @
else
QUIET :=
endif

ifeq ($(VERBOSE),2)
define NEWLINE_ONLY


endef
define NEWLINE_TAB


endef
endif

#
# End of build support section
##############################################################################################

##############################################################################################
# Output directories (must be before platforms section)
#

BIN = bin/$(PLATFORM)
OBJ = build/$(PLATFORM)
DEPDIR = $(OBJ)/dep

#
# End of Output directories
##############################################################################################

##############################################################################################
# Start of platforms section
#

PLATFORM ?= V3

ifeq ("$(PLATFORM)","G2v9i")

BASE_PLATFORM=v9_3x8c
DEVICE_DEFINES += MOTATE_BOARD="G2v9i"

endif

ifeq ("$(PLATFORM)", "Othermill")

BASE_PLATFORM=v9_3x8c
SETTINGS_FILE="settings_othermill.h"
DEVICE_DEFINES += MOTATE_BOARD="G2v9i" SETTINGS_FILE=${SETTINGS_FILE}

endif

ifeq ("$(PLATFORM)", "OthermillPro")

BASE_PLATFORM=v9_3x8c
SETTINGS_FILE="settings_othermill_pro.h"
DEVICE_DEFINES += MOTATE_BOARD="G2v9i" SETTINGS_FILE=${SETTINGS_FILE}

endif

ifeq ("$(PLATFORM)",$(filter "$(PLATFORM)","V3" "v3"))

BASE_PLATFORM=v9_3x8c
SETTINGS_FILE="settings_v3.h"
DEVICE_DEFINES += MOTATE_BOARD="G2v9i" SETTINGS_FILE=${SETTINGS_FILE}

endif

ifeq ("$(BASE_PLATFORM)","")
	BASE_PLATFORM = $(PLATFORM)
endif

ifeq ("$(BASE_PLATFORM)","v9_3x8c")
	_PLATFORM_FOUND = 1
	FIRST_LINK_SOURCES += motate/SamTimers.cpp motate/SamUSB.cpp motate/SamPins.cpp

	CHIP = SAM3X8C
	PLATFORM_BASE = platform/atmel_sam
	CMSIS_ROOT = CMSIS

	DEVICE_INCLUDE_DIRS += $(PLATFORM_BASE)/board/v9_3x8c

	include $(PLATFORM_BASE).mk

endif

ifeq ("$(_PLATFORM_FOUND)", "0")
# errors cannot be indented
$(error Unknown platform "$(PLATFORM)")
endif

#
# End of platforms section
##############################################################################################

##############################################################################################
# Start of setup tools
#

# Output file basename
OUTPUT_BIN = $(BIN)/$(PLATFORM)

# Some basical (external) utility paths
RM      = rm
CP      = cp
# NOTE: Atmel Studio will default to the wrong mkdir that doesn't understand -p.
#       So, in windows we set MKDIR=gmkdir below.
MKDIR   = mkdir
GIT     = git

SHELL = bash

SPECIAL_ATMEL_STUDIO_DEFAULT_TARGETS =

# Here we use some heuristics to find the OS.
ifneq (,$(findstring Atmel,$(PATH)))
$(info "Found that we're in Atmel Studio")
OS := WIN32
TOOLS_SUBPATH := win32/gcc-$(CROSS_COMPILE)
PATH := $(TOOLS_PATH)/$(TOOLS_SUBPATH)/bin;c:\Program Files\Git\bin;c:\Program Files\Git\cmd;c:\Program Files\Git\mingw32\bin;c:\Program Files\Git\mingw64\bin;$(PATH)
MKDIR   = gmkdir
SPECIAL_ATMEL_STUDIO_DEFAULT_TARGETS = $(PROJECT).elf $(PROJECT).map
else
ifneq (,$(findstring Windows,$(PATH)))
$(info "Found that we're in Cygwin or Windows")
OS := WIN32
TOOLS_SUBPATH := win32/gcc-$(CROSS_COMPILE)
PATH := $(TOOLS_PATH)/$(TOOLS_SUBPATH)/bin;c:\Program Files\Git\bin;c:\Program Files\Git\cmd;c:\Program Files\Git\mingw32\bin;c:\Program Files\Git\mingw64\bin;$(PATH)
MKDIR   = gmkdir
COLOR := 1
else


# Unix/Linux section:
UNAME := $(shell uname -s)

ifeq (Darwin,${UNAME})
$(info "Found that we're in OS X")
OS = OSX
TOOLS_SUBPATH := osx/gcc-$(CROSS_COMPILE)
COLOR := 1
else
ifeq (Linux,${UNAME})
$(info "Found that we're in Linux")
OS = LINUX
TOOLS_SUBPATH := linux/gcc-$(CROSS_COMPILE)
COLOR := 1
endif #LINUX
endif #Darwin


PATH := $(TOOLS_PATH)/$(TOOLS_SUBPATH)/bin:$(PATH)

# end Unix/linux section
endif #Windows
endif #Atmel Studio

export PATH
export MOTATE_PATH

CWD = $(realpath .)

# Compilation tools - hardcode the full path to the ones we provide
TOOLS_FULLPATH = "$(TOOLS_PATH)/$(TOOLS_SUBPATH)/bin"
CC      = $(TOOLS_FULLPATH)/$(CROSS_COMPILE)-gcc
CXX     = $(TOOLS_FULLPATH)/$(CROSS_COMPILE)-g++
LD      = $(TOOLS_FULLPATH)/$(CROSS_COMPILE)-ld
AR      = $(TOOLS_FULLPATH)/$(CROSS_COMPILE)-ar
SIZE    = $(TOOLS_FULLPATH)/$(CROSS_COMPILE)-size
STRIP   = $(TOOLS_FULLPATH)/$(CROSS_COMPILE)-strip
OBJCOPY = $(TOOLS_FULLPATH)/$(CROSS_COMPILE)-objcopy
GDB     = $(TOOLS_FULLPATH)/$(CROSS_COMPILE)-gdb
GDB_PY  = $(TOOLS_FULLPATH)/$(CROSS_COMPILE)-gdb-py
NM      = $(TOOLS_FULLPATH)/$(CROSS_COMPILE)-nm


ifneq ($(NOT_IN_GIT),1)
	GIT_LOCATED := $(GIT)
	GIT_VERSION := $(shell $(GIT) describe --abbrev=4 --dirty --always --tags)
	GIT_EXACT_VERSION := $(shell $(GIT) describe --abbrev=0 --tags)

	# Push the git version into the code -- the ""s must be escaped and make it into the code intact
	DEVICE_DEFINES += GIT_VERSION=\"$(GIT_VERSION)\" GIT_EXACT_VERSION=$(GIT_EXACT_VERSION)
else
   # Push the git version into the code -- the ""s must be escaped and make it into the code intact
   DEVICE_DEFINES += GIT_VERSION=\"UNKNOWN-not-from-git\" GIT_EXACT_VERSION=0.0
endif


# ---------------------------------------------------------------------------------------
# LTO generic flags

ifeq ($(USE_LTO),1)
LTO = -flto
else
LTO =
endif


# ---------------------------------------------------------------------------------------
# C Flags (NOT CPP flags)

#CFLAGS +=  $(DEVICE_CFLAGS)
CFLAGS +=  $(DEVICE_CFLAGS) -DPLATFORM=$(PLATFORM) $(LTO) -DFIXMATH_NO_64BIT

# ---------------------------------------------------------------------------------------
# CPP Flags

#CPPFLAGS +=  $(DEVICE_CPPFLAGS)
CPPFLAGS +=  $(DEVICE_CPPFLAGS) -DPLATFORM=$(PLATFORM) $(LTO) -DFIXMATH_NO_64BIT

# ---------------------------------------------------------------------------------------
# ASM Flags

ASFLAGS = $(DEVICE_ASFLAGS)  $(LTO)

# ---------------------------------------------------------------------------------------
# Linker Flags

DEBUG_SYMBOLS = -g3

LDFLAGS += $(LIBS) $(USER_LIBS) $(DEBUG_SYMBOLS) -O$(OPTIMIZATION) -Wl,--cref -Wl,--check-sections -Wl,--gc-sections -Wl,--entry=Reset_Handler -Wl,--unresolved-symbols=report-all -Wl,--warn-common -Wl,--warn-section-align -Wl,--warn-unresolved-symbols $(DEVICE_LDFLAGS) $(LTO)

ifeq ($(COLOR),1)
START_BOLD=`tput bold`
END_BOLD=`tput sgr0`
NO_COLOR=`tput setaf 0`
OK_COLOR=`tput setaf 2`
ERROR_COLOR=`tput setaf 1`
WARN_COLOR=`tput setaf 3`
endif

#
# End of setup tools
##############################################################################################

#-------------------------------------------------------------------------------
#    Files
#-------------------------------------------------------------------------------

# Directories where source files can be found

C_SOURCES   = $(foreach dir, $(SOURCE_DIRS), $(wildcard $(dir)/*.c) )
CXX_SOURCES = $(foreach dir, $(SOURCE_DIRS), $(wildcard $(dir)/*.cpp) )

C_OBJECTS   := $(addsuffix .o,$(basename $(C_SOURCES)))
CXX_OBJECTS := $(addsuffix .o,$(basename $(CXX_SOURCES)))
FIRST_LINK_OBJECTS = $(addsuffix .o,$(basename $(FIRST_LINK_SOURCES)))

# Append OBJ and BIN directories to output filename
OUTPUT := $(BIN)/$(OUTPUT_BIN)

INCLUDES = $(patsubst %,-I%,$(DEVICE_INCLUDE_DIRS) $(USER_INCLUDE_DIRS))
LIBS     = $(patsubst %,-l%,$(DEVICE_LIBS) $(USER_LIBS))
LIBDIR   = $(patsubst %,-L%,$(DEVICE_LIB_DIRS) $(USER_LIB_DIRS))

#
# End of file paths setup
##############################################################################################

##############################################################################################
# Start of RULES
#

ifeq ($(VERBOSE),2)
$(info DEVICE_INCLUDE_DIRS+USER_INCLUDE_DIRS:$(patsubst %,$(NEWLINE_TAB)%,$(DEVICE_INCLUDE_DIRS) $(USER_INCLUDE_DIRS)))
$(info $(NEWLINE_ONLY)DEVICE_LIB_DIRS+USER_LIB_DIRS: $(patsubst %,$(NEWLINE_TAB)%,$(DEVICE_LIB_DIRS) $(USER_LIB_DIRS)))
$(info $(NEWLINE_ONLY)SOURCE_DIRS: $(patsubst %,$(NEWLINE_TAB)%,$(SOURCE_DIRS)))
$(info $(NEWLINE_ONLY)C_SOURCES: $(patsubst %,$(NEWLINE_TAB)%,$(C_SOURCES)))
$(info $(NEWLINE_ONLY)SAM_SOURCE_DIRS: $(patsubst %,$(NEWLINE_TAB)%,$(SAM_SOURCE_DIRS)))
$(info $(NEWLINE_ONLY)CXX_SOURCES: $(patsubst %,$(NEWLINE_TAB)%,$(CXX_SOURCES)))
$(info $(NEWLINE_ONLY)FIRST_LINK_SOURCES: $(patsubst %,$(NEWLINE_TAB)%,$(FIRST_LINK_SOURCES)))
endif


#-------------------------------------------------------------------------------
#    Rules
#-------------------------------------------------------------------------------

all: $(OUTPUT_BIN).elf $(SPECIAL_ATMEL_STUDIO_DEFAULT_TARGETS)

REQUIRED_DIRS := $(BIN) $(OBJ) $(DEPDIR)

MK_DIRS =   $(shell                              \
		for d in $(REQUIRED_DIRS);         \
		do                                 \
		[[ -d $$d ]] || mkdir -p $$d;    \
		done)

$(eval $(DEVICE_RULES))

# We use tools as a "macro" for dependencies later
tools: | $(TOOLS_PATH)/$(TOOLS_SUBPATH)/bin

$(TOOLS_PATH)/$(TOOLS_SUBPATH)/bin:
	@echo Installing the necessary tools...
	cd ${TOOLS_PATH} && make "ARCH=gcc-${CROSS_COMPILE}"

OUTDIR = $(OBJ)
REQUIRED_DIRS += $(OUTDIR)


FIRST_LINK_OBJECTS_PATHS := $(addprefix $(OUTDIR)/,$(subst $(MOTATE_PATH),motate,$(FIRST_LINK_OBJECTS)))

MOTATE_C_OBJECTS   := $(addprefix $(OUTDIR)/,$(subst $(MOTATE_PATH),motate,$(filter $(MOTATE_PATH)/%,$(C_OBJECTS))))
MOTATE_CXX_OBJECTS := $(addprefix $(OUTDIR)/,$(subst $(MOTATE_PATH),motate,$(filter $(MOTATE_PATH)/%,$(CXX_OBJECTS))))
MOTATE_ASM_OBJECTS := $(addprefix $(OUTDIR)/,$(subst $(MOTATE_PATH),motate,$(filter $(MOTATE_PATH)/%,$(ASM_OBJECTS))))

ALL_OTHER_C_OBJECTS   := $(addprefix $(OUTDIR)/,$(filter-out $(MOTATE_PATH)/%,$(C_OBJECTS)))
ALL_OTHER_CXX_OBJECTS := $(addprefix $(OUTDIR)/,$(filter-out $(MOTATE_PATH)/%,$(CXX_OBJECTS)))
ALL_OTHER_ASM_OBJECTS := $(addprefix $(OUTDIR)/,$(filter-out $(MOTATE_PATH)/%,$(ASM_OBJECTS)))

ALL_C_OBJECTS   := $(MOTATE_C_OBJECTS) $(ALL_OTHER_C_OBJECTS)
ALL_CXX_OBJECTS := $(MOTATE_CXX_OBJECTS) $(ALL_OTHER_CXX_OBJECTS)
ALL_ASM_OBJECTS := $(MOTATE_ASM_OBJECTS) $(ALL_OTHER_ASM_OBJECTS)


#
CFLAGS   += $(DEBUG_SYMBOLS) -O$(OPTIMIZATION) $(INCLUDES) $(patsubst %,-D%,$(DEVICE_DEFINES) $(USER_DEFINES))
CPPFLAGS += $(DEBUG_SYMBOLS) -O$(OPTIMIZATION) $(INCLUDES) $(patsubst %,-D%,$(DEVICE_DEFINES) $(USER_DEFINES))
ASFLAGS  += $(DEBUG_SYMBOLS) -O$(OPTIMIZATION) $(INCLUDES) -D__ASSEMBLY__ $(patsubst %,-D%,$(DEVICE_DEFINES) $(USER_DEFINES))

LINKER_SCRIPT ?= $(PLATFORM_BASE)/gcc_flash.ld
#LINKER_SCRIPT := $(DEVICE_LINKER_SCRIPT)
ABS_LINKER_SCRIPT = $(abspath $(LINKER_SCRIPT))

# Generate dependency information
DEPFLAGS = -MMD -MF $(OBJ)/dep/$(@F).d -MT $(subst $(OUTDIR),$(OBJ),$@)

$(OUTPUT_BIN).elf: $(ALL_C_OBJECTS) $(ALL_CXX_OBJECTS) $(ALL_ASM_OBJECTS) $(ABS_LINKER_SCRIPT)
	@echo $(START_BOLD)"Linking $(OUTPUT_BIN).elf" $(END_BOLD)
	@echo $(START_BOLD)"Using linker script: $(ABS_LINKER_SCRIPT)" $(END_BOLD)
	$(QUIET)$(CXX) $(LIB_PATH) -T"$(ABS_LINKER_SCRIPT)" -Wl,-Map,"$(OUTPUT_BIN).map" -o ${filter-out tools,$@} $(LDFLAGS) $(LD_OPTIONAL) $(LIBS) -Wl,--start-group $(FIRST_LINK_OBJECTS_PATHS) $(filter-out $(FIRST_LINK_OBJECTS_PATHS) $(ABS_LINKER_SCRIPT) tools,$+) -Wl,--end-group
	@echo $(START_BOLD)"Exporting symbols $(OUTPUT_BIN).elf.txt" $(END_BOLD)
	$(QUIET)$(NM) "$(OUTPUT_BIN).elf" >"$(OUTPUT_BIN).elf.txt"
	@echo $(START_BOLD)"Making binary $(OUTPUT_BIN).bin" $(END_BOLD)
	$(QUIET)$(OBJCOPY) -O binary "$(OUTPUT_BIN).elf" "$(OUTPUT_BIN).bin"
	$(QUIET)printf "%08x" `cksum $(OUTPUT_BIN).bin | cut -d' ' -f1` | xxd -r -p | dd of=$(OUTPUT_BIN).bin bs=1 count=4 seek=256 conv=notrunc
	$(QUIET)printf "%08x" `ls -nl "$(OUTPUT_BIN).bin" | cut -d' ' -f5` | xxd -r -p | dd of=$(OUTPUT_BIN).bin bs=1 count=4 seek=260 conv=notrunc
	@echo "--- SIZE INFO ---"
	$(QUIET)$(SIZE) "$(OUTPUT_BIN).elf"

## Note: The motate paths are seperated do to MOTATE_PATH having multple ../ in it.

$(MOTATE_CXX_OBJECTS): $(OUTDIR)/motate/%.o: $(MOTATE_PATH)/%.cpp tools
	$(QUIET)$(MKDIR) -p "$(@D)" "$(DEPDIR)" "$(BIN)"
	@echo $(START_BOLD)"Compiling cpp $<"; echo "    -> $@" $(END_BOLD)
	$(QUIET)$(CXX) $(CPPFLAGS) $(DEPFLAGS) -xc++ -c -o $@ $<

$(ALL_OTHER_CXX_OBJECTS): $(OUTDIR)/%.o: %.cpp tools
	$(QUIET)$(MKDIR) -p "$(@D)" "$(DEPDIR)" "$(BIN)"
	@echo $(START_BOLD)"Compiling cpp $<"; echo "    -> $@" $(END_BOLD)
	$(QUIET)$(CXX) $(CPPFLAGS) $(DEPFLAGS) -xc++ -c -o $@ $<

$(MOTATE_C_OBJECTS): $(OUTDIR)/motate/%.o: $(MOTATE_PATH)/%.c tools
	$(QUIET)$(MKDIR) -p "$(@D)" "$(DEPDIR)" "$(BIN)"
	@echo $(START_BOLD)"Compiling c $<"; echo "    -> $@" $(END_BOLD)
	$(QUIET)$(CC) $(CFLAGS) $(DEPFLAGS) -c -o $@ $<

$(ALL_OTHER_C_OBJECTS): $(OUTDIR)/%.o: %.c tools
	$(QUIET)$(MKDIR) -p "$(@D)" "$(DEPDIR)" "$(BIN)"
	@echo $(START_BOLD)"Compiling c $<"; echo "    -> $@" $(END_BOLD)
	$(QUIET)$(CC) $(CFLAGS) $(DEPFLAGS) -c -o $@ $<

$(MOTATE_ASM_OBJECTS): $(OUTDIR)/motate/%.o: $(MOTATE_PATH)/%.S tools
	$(QUIET)$(MKDIR) -p "$(@D)" "$(DEPDIR)" "$(BIN)"
	@echo $(START_BOLD)"Compiling $<"; echo "    -> $@"  $(END_BOLD)
	$(QUIET)$(CC) $(ASFLAGS) $(DEPFLAGS) -c -o $@ $<

$(ALL_OTHER_ASM_OBJECTS): $(OUTDIR)/%.o: %.S tools
	$(QUIET)$(MKDIR) -p "$(@D)" "$(DEPDIR)" "$(BIN)"
	@echo $(START_BOLD)"Compiling $<"; echo "    -> $@"  $(END_BOLD)
	$(QUIET)$(CC) $(ASFLAGS) $(DEPFLAGS) -c -o $@ $<

# Rule for debugging
debug: $(OUTPUT_BIN).elf
	$(GDB) -ex "dir '${MOTATE_PATH}/arch/'" -x "${PLATFORM_BASE}.gdb" -ex "monitor reset halt" -readnow -se "$(OUTPUT_BIN).elf"

# Rule for debugging (using python-enabled debugger)
debuggy: $(OUTPUT_BIN).elf
	$(GDB_PY) -ex "dir '${MOTATE_PATH}/arch/'" -x "${PLATFORM_BASE}.gdb" -ex "monitor reset halt" -readnow -se "$(OUTPUT_BIN).elf"

flash: $(FLASH_REQUIRES)
	$(DEVICE_FLASH_CMD)

#-------------------------------------------------------------------------------
#		Default $(PROJECT).elf, and clean
#-------------------------------------------------------------------------------

$(PROJECT).elf: $(OUTPUT_BIN).elf
	$(CP) $< $@

$(PROJECT).map: $(OUTPUT_BIN).map
	$(CP) $< $@

$(PROJECT).hex: $(OUTPUT_BIN).elf
	$(QUIET)$(OBJCOPY) -O ihex $(DEVICE_HEX_FLAGS) $< $@

$(PROJECT).bin: $(OUTPUT_BIN).elf
	$(QUIET)$(OBJCOPY) -O binary $< $@

clean:
	-$(RM) -fR $(OBJ)
	-$(RM) -fR $(BIN)
	-$(RM) -fR $(PLATFORM).elf $(PLATFORM).map $(PLATFORM).hex $(PLATFORM).bin


#
# Include the dependency files, should be the last of the makefile
#
-include $(shell mkdir $(OBJ)/dep 2>/dev/null) $(wildcard $(OBJ)/dep/*.d)

#
# End of RULES
##############################################################################################

# *** EOF ***
