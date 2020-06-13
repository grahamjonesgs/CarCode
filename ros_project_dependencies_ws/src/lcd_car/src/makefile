all: lcd_car

lcd_car: lcd_car.o fontx.o wili9340.o
	 gcc -o lcd_car lcd_car.o  wili9340.o fontx.o -lm -lpthread -lbcm2835

lcd_car.o: lcd_car.c
	 gcc -c lcd_car.c

fontx.o: fontx.c
	gcc -c fontx.c

wili9340.o: wili9340.c
		gcc -c wili9340.c -DBCM

clean:
	 rm -f lcd_car.o lcd_car wili9340.o fontx.o
