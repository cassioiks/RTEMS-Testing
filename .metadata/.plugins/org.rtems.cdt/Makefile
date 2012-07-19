include $(PROJECT_RELEASE)/Makefile.inc
include $(RTEMS_CUSTOM)
include $(PROJECT_ROOT)/make/leaf.cfg
all:
	@echo org.rtems.cdt.tool.archiver
	@for i in $(AR) ; do printf "\t%s\n" "$$i" ; done
	@echo org.rtems.cdt.tool.assembler
	@for i in $(AS) ; do printf "\t%s\n" "$$i" ; done
	@echo org.rtems.cdt.tool.compiler.c
	@for i in $(COMPILE.c) ; do printf "\t%s\n" "$$i" ; done
	@echo org.rtems.cdt.tool.compiler.cpp
	@for i in $(COMPILE.cc) ; do printf "\t%s\n" "$$i" ; done
	@echo org.rtems.cdt.tool.linker.c
	@for i in $(LINK.c) ; do printf "\t%s\n" "$$i" ; done
	@echo org.rtems.cdt.tool.linker.cpp
	@for i in $(LINK.cc) ; do printf "\t%s\n" "$$i" ; done
