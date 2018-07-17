#Import library and initialise game engine
import pygame
import time
import re
pygame.init()

# Load images
robot = pygame.image.load('arrow.png')

def rotate_center(image, angle):
    """rotate a Surface, maintaining position."""

    loc = image.get_rect().center  #rot_image is not defined 
    rot_image = pygame.transform.rotate(image, angle)
    rot_image.get_rect().center = loc
    return rot_image

# Define some colors
BLACK = ( 0, 0, 0)
WHITE = ( 255, 255, 255)

#open a new window
MAP_SIZE = 300
size = (MAP_SIZE,MAP_SIZE)
screen = pygame.display.set_mode(size)
pygame.display.set_caption("ToF SLAM")

carryOn = True

#Set up the clock that controls the refresh rate
clock = pygame.time.Clock()

print_particles = 0
print_robot = 1
print_tra = 0
i = 0
pos = []

#Start main loop
while carryOn:

	# --- Main event loop
    for event in pygame.event.get(): # User did something
        if event.type == pygame.QUIT: # If user clicked close
            carryOn = False # Flag that we are done so we exit this loop
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_p:     
                print_particles = print_particles^1
            if event.key == pygame.K_r:   
                print_robot = print_robot^1
            if event.key == pygame.K_t:   
                print_tra = print_tra^1
    
    x = 0
    y = 0

    #Read the file
    f = open("/home/diego/esp_rtos/esp-open-rtos/tofslam/computer/map",'r')
    map = f.readlines()
    f.close()
    for line in map:
        if line.strip():
            if int(line) < 257:
                screen.set_at((x, y), (int(line),int(line),int(line)))
            x = x + 1
            if x > (MAP_SIZE-1):
                x = 0
                y = y + 1
    
    if print_particles == 1:
        # Print particles of the filter
        x = 0
        y = 0
        f = open("/home/diego/esp_rtos/esp-open-rtos/tofslam/computer/random",'r')
        map = f.readlines()
        f.close()
        for line in map:
            if line.strip():
                if int(line) == 0:
                    screen.set_at((x, y), (0,255,0))
                x = x + 1
                if x > (MAP_SIZE-1):
                    x = 0
                    y = y + 1
    
    # Lets print robot position	
    last_pos = (MAP_SIZE/2,MAP_SIZE/2)
    with open("/home/diego/esp_rtos/esp-open-rtos/tofslam/computer/pos",'r') as f:
        for line in f:
            if line.strip():
                pos = []
                line = re.split(r'\t+',line)
                pos.append(int(line[0]))
                pos.append(int(line[1]))
                pos.append(int(line[2]))
                if print_tra == 1:
                    pygame.draw.line(screen,(0,0,255),[pos[0],pos[1]],[last_pos[0],last_pos[1]],1)  
                last_pos = pos
    if print_robot == 1:    
        #Rotate and draw robot
        rot_robot = rotate_center(robot,pos[2])
        screen.blit(rot_robot,(pos[0] - rot_robot.get_rect().center[0],pos[1] - rot_robot.get_rect().center[1]))
    
    # Draw position   
    #pygame.draw.line(screen,(255,0,0),[pos[0],pos[1]],[pos[0]+10,pos[1]],1)  
   
    #screen.fill(BLACK)

	#Display new frame
    pygame.display.flip()

	#Set refresh rate to 60Hz
    clock.tick(60)

print("Bye")

pygame.quit()
