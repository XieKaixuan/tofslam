#makefile

CC = gcc
OBJECTS = SLAM_random.o SLAM_functions.o get_udp.o slam.o
CFLAGS = -pthread -lm -I. -lncurses

tofslam: tofslam.o $(OBJECTS)
	$(CC) -o tofslam tofslam.o $(OBJECTS) $(CFLAGS)
	
tofslam.o: tofslam.h
get_udp.o: tofslam.h
slam.o: tofslam.h
SLAM_random.o: tofslam.h
SLAM_functions.o: tofslam.h

.PHONY : clean	
clean :
	-rm tofslam tofslam.o SLAM_random.o SLAM_functions.o get_udp.o slam.o	
