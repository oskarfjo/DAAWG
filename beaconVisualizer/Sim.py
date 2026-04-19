#!/usr/bin/env python3

import pygame
import numpy as np
import time
import socket
import csv

HEATMAP = True

### UDP Setup ###
UDP_IP = "0.0.0.0"
UDP_PORT = 8080

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
sock.setblocking(False) 


### CSV Logging ###
if (not HEATMAP):
    fieldnames = ['time(s)', 'angle(deg)', 'rssi', 'lat', 'lon']
else:
    fieldnames = ['rssi', 'lat', 'lon']
    
logg_title = 'logg'
with open(logg_title + '.csv', 'w', newline='') as f:    
    writer = csv.DictWriter(f, fieldnames=fieldnames)
    writer.writeheader()
    
### init ###
pygame.init()

screen_height = 990
screen_width = 900

screen = pygame.display.set_mode((screen_width, screen_height))
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
rocket_image = pygame.transform.scale(imported_rocket_image, (400, 200))

timer = 0.0
msg_time = 0.0
start_time = time.time()
last_time = time.time()

angle = 0.0
rssi = 0.0

running = True


############
### LOOP ###
############

while running:

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_x:
                running = False

    ### UDP ###
    try:
        data, addr = sock.recvfrom(1024)
        message = data.decode('utf-8').strip()
        print(message)
        
        if "," in message:
            parts = message.split(",")
            angle_id = parts[0]
            rssi = float(parts[1])
            lat = float(parts[2])
            lon = float(parts[3])
            
        angle_map = {"0": 45.0, "1": 22.5, "2": 0.0, "3": -22.5, "4": -45.0}
        
        if angle_id in angle_map:
            angle = angle_map[angle_id]
            msg_time = 0.0
        else:
            angle = 180
            msg_time = 0.0
        
        with open('logg.csv', 'a', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            if (not HEATMAP):
                data = [{'time(s)': round(timer, 2), 'angle(deg)': angle, 'rssi': rssi, 'lat': lat, 'lon': lon}]
            else:
                data = [{'rssi': rssi, 'lat': lat, 'lon': lon}]

            writer.writerows(data)
            
    except BlockingIOError:
        pass
    except ValueError:
        print(f"Received malformed data: {message}")

    current_time = time.time()
    dt = current_time - last_time
    last_time = current_time
    msg_time += dt
    timer += dt

    ## Normalize angle to [-180°, 180°]
    if angle > 180:
        angle -= 360
    elif angle < -180:
        angle += 360
        
    ### Aesthetics ###
    if msg_time >= 2:
        screen.fill((100, 100, 100))
    else:
        screen.fill(bg_color)
        
    pygame.draw.rect(screen, botom_color, [0, (screen_height - 40) , screen_width, 250])

    circle_center = (screen_width/2, screen_height/2)
    circle_rect = circle_image.get_rect(center=circle_center)
    screen.blit(circle_image, circle_rect)
        
    ### SPRITE ###
    rocket_center = (screen_width/2, screen_height/2)
    rotated_image = pygame.transform.rotate(rocket_image, angle)
    rotated_rect = rotated_image.get_rect(center=rocket_center)
    screen.blit(rotated_image, rotated_rect)

    ## BOTTOM TEXT ##
    text_theta = font.render(f'Theta: {round(angle, 2)} deg', True, text_color1)
    text_time = font.render(f'msg Time: {round(msg_time*100)}', True, text_color1)
    text_rssi = font.render(f'RSSI: {round(rssi, 1)}', True, text_color1)
    screen.blit(text_theta, (screen_width/2 - 100, (screen_height - 30)))
    screen.blit(text_time, (10, screen_height - 30))
    screen.blit(text_rssi, (screen_width-150, screen_height - 30))
    
    pygame.display.flip()
    clock.tick(60)

sock.close()
pygame.quit()