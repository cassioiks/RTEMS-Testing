#
#  Makefile
#

#
#  RTEMS_MAKEFILE_PATH is typically set in an environment variable
#

EXEC=test.exe
PGM=${ARCH}/$(EXEC)

FLASHTOOL=$(HOME)/flashtool

# optional managers required
MANAGERS=io rate_monotonic

# C source names
CSRCS = init.c fqd.c tpu.c mcpwm.c lcd.c motor.c servo.c distance.c \
	spi.c robot.c flame.c mcp3208.c gyro.c pta.c accel.c \
	kalman.c f16_16.c fastint.c robot_trace.c tone.c
COBJS_ = $(CSRCS:.c=.o)
COBJS = $(COBJS_:%=${ARCH}/%)

# C++ source names
CXXSRCS =
CXXOBJS_ = $(CXXSRCS:.cc=.o)
CXXOBJS = $(CXXOBJS_:%=${ARCH}/%)

# AS source names
ASSRCS =
ASOBJS_ = $(ASSRCS:.s=.o)
ASOBJS = $(ASOBJS_:%=${ARCH}/%)

# Libraries
LIBS = -lrtemsall -lc

# Un-comment this line if you want to build a ROM image:
MRM_IN_ROM = yes

include $(RTEMS_MAKEFILE_PATH)/Makefile.inc

include $(RTEMS_CUSTOM)
include $(PROJECT_ROOT)/make/leaf.cfg

# temp hack - turn off optimization.  we should do a complete build of
# rtems with VARIANT=DEBUG, but the current version of gcc crashes.
CFLAGS_OPTIMIZE_V = -g

OBJS= $(COBJS) $(CXXOBJS) $(ASOBJS)

LINK_LIBS += -lm

all:    ${ARCH} $(PGM)

$(PGM): $(OBJS)
	$(make-exe)

genwallanglelookup: genwallanglelookup.c
	gcc $^ -lm -o $@

flash: all
	$(FLASHTOOL)/flashmrm ${ARCH}/${EXEC}
