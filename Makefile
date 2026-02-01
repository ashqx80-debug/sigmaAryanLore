################################################################################
######################### User configurable parameters #########################
# filename extensions
CEXTS:=c
ASMEXTS:=s S
CXXEXTS:=cpp c++ cc

# probably shouldn't modify these, but you may need them below
ROOT=.
FWDIR:=$(ROOT)/firmware
BINDIR=$(ROOT)/bin
SRCDIR=$(ROOT)/src
INCDIR=$(ROOT)/include

WARNFLAGS+=
# Optimization flags: only change when using 'make fast' for rapid development builds
ifdef FAST_BUILD
EXTRA_CFLAGS=-O0 -g0
EXTRA_CXXFLAGS=-O0 -g0
else
EXTRA_CFLAGS=
EXTRA_CXXFLAGS=
endif

# Set to 1 to enable hot/cold linking
USE_PACKAGE:=1

# Add libraries you do not wish to include in the cold image here
# EXCLUDE_COLD_LIBRARIES:= $(FWDIR)/your_library.a
EXCLUDE_COLD_LIBRARIES:= 

# Set this to 1 to add additional rules to compile your project as a PROS library template
IS_LIBRARY:=0
# TODO: CHANGE THIS! 
# Be sure that your header files are in the include directory inside of a folder with the
# same name as what you set LIBNAME to below.
LIBNAME:=libbest
VERSION:=1.0.0
# EXCLUDE_SRC_FROM_LIB= $(SRCDIR)/unpublishedfile.c
# this line excludes opcontrol.c and similar files
EXCLUDE_SRC_FROM_LIB+=$(foreach file, $(SRCDIR)/main,$(foreach cext,$(CEXTS),$(file).$(cext)) $(foreach cxxext,$(CXXEXTS),$(file).$(cxxext)))

# files that get distributed to every user (beyond your source archive) - add
# whatever files you want here. This line is configured to add all header files
# that are in the directory include/LIBNAME
TEMPLATE_FILES=$(INCDIR)/$(LIBNAME)/*.h $(INCDIR)/$(LIBNAME)/*.hpp

.DEFAULT_GOAL=quick

# Enable parallel builds by default
MAKEFLAGS += -j$(shell sysctl -n hw.ncpu)

# Optional: Enable ccache if available (install with: brew install ccache)
# ccache can speed up recompilation by caching previous compilations
ifneq ($(shell which ccache 2>/dev/null),)
    CC:=ccache $(CC)
    CXX:=ccache $(CXX)
endif

# Fast build target with minimal optimization (use 'make fast' for development)
.PHONY: fast
fast:
	$(MAKE) FAST_BUILD=1 $(DEFAULT_BIN)

# Help target to show available build commands
.PHONY: help
help:
	@echo "Available build targets:"
	@echo "  make          - Standard build (default: -Os -g, parallel compilation)"
	@echo "  make fast     - Fastest build with no optimization (-O0 -g0 for rapid iteration)"
	@echo "  make all      - Clean build from scratch"
	@echo "  make clean    - Remove all build artifacts"
	@echo ""
	@echo "Build optimizations enabled:"
	@echo "  - Parallel compilation using $(shell sysctl -n hw.ncpu) CPU cores (SAFE)"
	@echo "  - ccache support if installed (SAFE - run: brew install ccache)"
	@echo "  - Incremental builds (only recompiles changed files)"
	@echo ""
	@echo "Performance tips:"
	@echo "  - Default 'make' uses original safe flags (-Os -g)"
	@echo "  - Use 'make fast' for 3x faster development iteration"
	@echo "  - Parallel builds give ~4x speedup with no compromises"

################################################################################
################################################################################
########## Nothing below this line should be edited by typical users ###########
-include ./common.mk
