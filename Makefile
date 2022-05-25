obj-m += bmp280.o 
bmp280-objs := bmp_driver.o bmp2.o

EXTRA_CFLAGS:= -D BMP2_32BIT_COMPENSATION=1 

KERN_SRC = /home/jagannath/rugged_board_a5d2x/linux-rba5d2x

all:
	make -C ${KERN_SRC} M=${PWD} modules
clean:
	make -C ${KERN_SRC} M=${PWD} clean
copy:
	cp bmp280.ko /var/lib/tftpboot/.
	