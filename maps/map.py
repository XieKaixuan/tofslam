#Import library and initialise game engine
import pygame
pygame.init()

# Define some colors
BLACK = ( 0, 0, 0)
WHITE = ( 255, 255, 255)

#open a new window
size = (500,500)
screen = pygame.display.set_mode(size)
pygame.display.set_caption("ToF SLAM")

carryOn = True

#Set up the clock that controls the refresh rate
clock = pygame.time.Clock()

#Start main loop
while carryOn:

	# --- Main event loop
    for event in pygame.event.get(): # User did something
        if event.type == pygame.QUIT: # If user clicked close
              carryOn = False # Flag that we are done so we exit this loop

    x = 0
    y = 0

    #Read the file
    with open("/home/diego/esp_rtos/esp-open-rtos/tofslam/computer/map",'r') as f:
        for line in f:
        #line = f.readline()
            if line.strip():
                screen.set_at((x, y), (int(line),int(line),int(line)))
                x = x + 1
                if x > 499:
            	    x = 0
            	    y = y + 1

    #print(map[2])
    #for x in range(0,500):
    #    for y in range(0,500):
    #        screen.set_at((x, y), (int(map[y*500+x].strip()),int(map[y*500+x].strip()),int(map[y*500+x].strip())))

    #screen.fill(BLACK)

	#Display new frame
    pygame.display.flip()

	#Set refresh rate to 60Hz
    clock.tick(60)

print("Bye")

pygame.quit()