# Build alok_driver.c as kernel module
obj-m := alok_driver.o

# User program (CLI)
alok-cli: cli.c alok.h
	$(CC) -Wall -Wextra -std=gnu11 -o $@ cli.c

all:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules
	$(MAKE) alok-cli

.PHONY: clean
clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean
	$(RM) alok-cli *.o *.ko modules.order .modules.order.cmd Module.symvers .Module.symvers.cmd .*.ko.cmd *.mod *.mod.c .*.mod.cmd *.mod.o .*.o.cmd
