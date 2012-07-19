
CFLAGS=-g

all: imu-1d imu-1d-fixed

imu-1d.o: imu-1d.c

imu-1d: imu-1d.o
	$(CC) $^ -lm -o $@

imu-1d-fixed.o: imu-1d-fixed.c

imu-1d-fixed: imu-1d-fixed.o
	$(CC) $^ -lm -o $@
