#ubuntu的内核源码树，如果要编译在ubuntu中安装的模块就打开这2个
#KERN_VER = $(shell uname -r)
#KERN_DIR = /lib/modules/$(KERN_VER)/build	

		
# 开发板的linux内核的源码树目录
KERN_DIR = /root/driver/kernel


all:
	arm-linux-gcc app.c -o app

cp:
	cp app /root/rootfs/rootfs/driver
	

.PHONY: clean	
clean:
	rm -rf app

