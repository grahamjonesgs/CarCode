all: car_ctrl

car_ctrl: car_ctrl.o
	 gcc -o car_ctrl car_ctrl.o -lpthread -lpigpiod_if2 -lrt -lncurses

car_ctrl.o: car_ctrl.c
	 gcc -c car_ctrl.c



clean:
	 rm -f car_ctrl.o car_ctrl
