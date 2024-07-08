obj-m += i2c-ch341.o

KERNEL_DIR := /lib/modules/$(shell uname -r)/build

all:
		make -C $(KERNEL_DIR) M=$(PWD) modules

clean:
		make -C $(KERNEL_DIR) M=$(PWD) clean

reload:	all
		sudo rmmod i2c-ch341 ||true
		sudo insmod ./i2c-ch341.ko
