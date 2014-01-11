obj-m += i2c-ch341.o

all:
		make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules

clean:
		make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean

reload:	all
		sudo rmmod i2c-ch341 ||true
		sudo insmod ./i2c-ch341.ko 