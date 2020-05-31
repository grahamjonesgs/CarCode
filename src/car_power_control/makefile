all: car_power_control

car_power_control: car_power_control.o
	 gcc -o car_power_control car_power_control.o -lpthread -lpigpiod_if2 -lrt

car_power_control.o: car_power_control.c
	 gcc -c car_power_control.c



clean:
	 rm -f car_power_control.o car_power_control
