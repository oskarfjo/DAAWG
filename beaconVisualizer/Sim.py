#!/usr/bin/env python3

import pygame
import numpy as np
import time

    ### init ###

pygame.init()

screen_height = 990
screen_width = 900

screen = pygame.display.set_mode((screen_width, screen_height)) # makes the window. window dimentions = (width, height)
clock = pygame.time.Clock()
font = pygame.font.Font(None, 36)
font_small = pygame.font.Font(None, 24)
font_large = pygame.font.Font(None, 150)
text_color1 = (255, 255, 200)
text_color2 = (10, 10, 10)
bg_color = (240, 240, 200)
botom_color = (10, 10, 10)
overlay = pygame.Surface((screen_width, screen_height))
overlay.set_alpha(178)
overlay.fill((20, 20, 20))
imported_rocket_image = pygame.image.load('drone_sprite.png')
imported_circle_icon = pygame.image.load('degree_circle_icon.png')
circle_image = pygame.transform.scale(imported_circle_icon, (980/1.5, 900/1.5))
rocket_image = pygame.transform.scale(imported_rocket_image, (400, 200)) # Scales the picture of the rocket in the simulator. 
timer = 0.0
start_time = time.time()

angle = 0

running = True
last_time = time.time()

    ############
    ### LOOP ###
    ############


while running:

    for event in pygame.event.get(): # configures the input keys for interacting with the sim
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_x: # shuts down the program when pressing x
                running = False
            elif event.key == pygame.K_y:
                angle += 1
            elif event.key == pygame.K_t:
                angle -= 1

    current_time = time.time()
    dt = current_time - last_time
    last_time = current_time


    ### Aestetics ###
    screen.fill(bg_color) # makes the background
    pygame.draw.rect(screen, botom_color, [0, (screen_height - 40) , screen_width, 250])

    circle_center = (screen_width/2, screen_height/2)
    circle_rect = circle_image.get_rect(center=circle_center)
    screen.blit(circle_image, circle_rect)
        
    ### configures the sprite ###
    rocket_center = (screen_width/2, screen_height/2)
    rotated_image = pygame.transform.rotate(rocket_image, angle)
    rotated_rect = rotated_image.get_rect(center=rocket_center)
    screen.blit(rotated_image, rotated_rect)


    ## BOTTOM TEXT ##
    text_theta = font.render(f'Theta: {round(angle, 2)} deg', True, text_color1)
    screen.blit(text_theta, (screen_width/2 - 100, (screen_height - 30)))
    
    ## Normalize angle to [-180°, 180°]
    if angle > 180:
        angle -= 360
    elif angle < -180:
        angle += 360
        
    pygame.display.flip()
    clock.tick(60)

pygame.quit()