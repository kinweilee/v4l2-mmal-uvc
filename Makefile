CROSS_COMPILE	?=
ARCH		?= x86
KERNEL_DIR	?= /usr/src/linux

CC		:= $(CROSS_COMPILE)gcc
KERNEL_INCLUDE	:= -I$(KERNEL_DIR)/include -I$(KERNEL_DIR)/arch/$(ARCH)/include
CFLAGS		:= -W -Wall -Wno-unused-function -I/opt/vc/include -pipe
#CFLAGS		:= -W -Wall -Wno-unused-function -O2 -g -I/opt/vc/include -pipe
LDFLAGS		:= 

LIBS    := -L/opt/vc/lib -lrt -lbcm_host -lvcos -lvchiq_arm -pthread -lmmal_core -lmmal_util -lmmal_vc_client -lvcsm
src = v4l2-mmal-uvc.c #$(wildcard *.c)
obj = $(src:.c=.o)

%.o : %.c
	$(CC) $(CFLAGS) -o $@ -c $< 

all: v4l2-mmal-uvc

v4l2-mmal-uvc: $(obj)
	$(CC) $(LDFLAGS) -o $@ $^ $(LIBS)

clean:
	rm -f *.o
	rm -f v4l2-mmal-uvc

