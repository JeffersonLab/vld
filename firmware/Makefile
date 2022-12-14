#
# File:
#    Makefile
#
# Description:
#    Makefile for TI firmware updating program
#
#
DEBUG	?= 1
QUIET	?= 1
#
ifeq ($(QUIET),1)
        Q = @
else
        Q =
endif

ARCH	?= $(shell uname -m)
OS	?= LINUX

ifdef CODA_VME
CODA_VME_INC = -I${CODA_VME}/include
endif
ifdef CODA_VME_LIB
CODA_LIB = -L${CODA_VME_LIB}
endif

# linuxvme defaults, if they're not already defined
LINUXVME_INC	?= ../../include
LINUXVME_LIB	?= ../../lib
LINUXVME_BIN	?= ../../bin

CROSS_COMPILE		=
CC			= $(CROSS_COMPILE)gcc
AR                      = ar
RANLIB                  = ranlib
INCS			= -I. -I../ -I${LINUXVME_INC} ${CODA_VME_INC}
CFLAGS			= -L. -L../ -L${LINUXVME_LIB} ${CODA_LIB} -lrt -ljvme -lvld
ifeq ($(DEBUG),1)
	CFLAGS		+= -Wall -g
endif

SRC			= $(wildcard *.c)
DEPS			= $(SRC:.c=.d)
OBJ			= $(SRC:.c=.o)
PROGS			= $(SRC:.c=)

all: echoarch $(PROGS)

clean distclean:
	@rm -f $(PROGS) *~ $(OBJS) $(DEPS)

%: %.c
	@echo " CC     $@"
	${Q}$(CC) $(CFLAGS) $(INCS) -o $@ $<

%.d: %.c
	@echo " DEP    $@"
	@set -e; rm -f $@; \
	$(CC) -MM -shared $(INCS) $< > $@.$$$$; \
	sed 's,\($*\)\.o[ :]*,\1 $@ : ,g' < $@.$$$$ > $@; \
	rm -f $@.$$$$

install: $(PROGS)
	@echo " CP     ${PROGS}"
	${Q}cp ${PWD}/$(PROGS) $(LINUXVME_BIN)/

coda_install: $(PROGS)
	@echo " CODACP ${PROGS}"
	${Q}cp ${PWD}/$(PROGS) $(CODA_VME_BIN)/

-include $(DEPS)

.PHONY: all clean distclean

echoarch:
	@echo "Make for $(OS)-$(ARCH)"
