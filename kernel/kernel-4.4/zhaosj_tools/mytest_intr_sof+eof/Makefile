# Makefile
obj-m := mytest_sof_eof_time.o

KERNEL_DIR :=/lib/modules/$(shell uname -r)/build

PWD :=$(shell pwd)

all:
	make -C $(KERNEL_DIR) SUBDIRS=$(PWD) modules

clean:

	rm *.o *.ko *.mod.c .*.cmd *.order *.symvers .tmp_versions -rf

.PHONY:clean