#makefile

CC = gcc
OBJECTS = SLAM_random.o SLAM_functions.o
CFLAGS = -I.

tofslam: tofslam.o $(OBJECTS)
	$(CC) -o tofslam tofslam.o $(OBJECTS) -lm -I.
	
tofslam.o: tofslam.h
SLAM_random.o: tofslam.h
SLAM_functions.o: tofslam.h

.PHONY : clean	
clean :
	-rm tofslam tofslam.o SLAM_random.o SLAM_functions.o	
