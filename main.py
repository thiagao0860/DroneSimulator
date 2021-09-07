import pygame
import sys
import math
import logging
import controler
import time
import numpy as np

class Drone:


    def __init__(self, image,frameconfig): 
        self.image = image
        self.__base_image=image
        self.angle = 0
        self.frameconfig=frameconfig
        self.space_config={
            'x_min': -60,
            'x_max': 60,
            'y_min': -10,
            'y_max': 40,
        }
        # [x,y] vector
        self.posByCenter(0,0)
     
    def translate(self,x_speed,y_speed):
        x_speed,y_speed = self.__transformEmulatedToRealVel(x_speed,y_speed)
        self.pos = self.pos.move(x_speed, y_speed)
        if self.pos.right > self.frameconfig['width']:
            self.pos.right = self.frameconfig['width']
        if self.pos.bottom > self.frameconfig['height']:
            self.pos.bottom = self.frameconfig['height']
        if self.pos.left < 0:
            self.pos.left = 0
        if self.pos.top < 0:
            self.pos.top = 0
    
    def posByCenter(self,x_pos,y_pos):
        x_pos,y_pos=self.__transformEmulatedToRealCoord(x_pos,y_pos)
        self.image.get_rect().center=(x_pos,y_pos)
        self.pos=self.image.get_rect()
        self.pos.center=(x_pos,y_pos)

    def set_rotation(self,angle):
        x,y=self.pos.center
        self.angle=angle
        self.image=pygame.transform.rotate(self.__base_image,self.angle)
        x_pos,y_pos= self.__transformRealToEmulatedCoord(x,y)
        self.posByCenter(x_pos,y_pos)

    def rotate(self,angle_speed):
        self.set_rotation(self.angle+angle_speed)

    def __transformEmulatedToRealCoord(self, x, y): 
        y_factor = -self.frameconfig['height']/(self.space_config['y_max']-self.space_config['y_min'])
        x_factor = self.frameconfig['width']/(self.space_config['x_max']-self.space_config['x_min'])
        y_transformed= y_factor*y+(self.space_config['y_max'])*self.frameconfig['height']/(self.space_config['y_max']-self.space_config['y_min'])
        x_transformed= x_factor*x-(self.space_config['x_min'])*self.frameconfig['width']/(self.space_config['x_max']-self.space_config['x_min'])
        return [x_transformed,y_transformed]

    def __transformRealToEmulatedCoord(self, x, y): 
        y_factor = -self.frameconfig['height']/(self.space_config['y_max']-self.space_config['y_min'])
        x_factor = self.frameconfig['width']/(self.space_config['x_max']-self.space_config['x_min'])
        y_transformed= (y-(self.space_config['y_max'])*self.frameconfig['height']/(self.space_config['y_max']-self.space_config['y_min']))/y_factor
        x_transformed= (x+(self.space_config['x_min'])*self.frameconfig['width']/(self.space_config['x_max']-self.space_config['x_min']))/x_factor
        return [x_transformed,y_transformed]

    def __transformEmulatedToRealVel(self, x, y): 
        y_factor = -self.frameconfig['height']/(self.space_config['y_max']-self.space_config['y_min'])
        x_factor = self.frameconfig['width']/(self.space_config['x_max']-self.space_config['x_min'])
        y_transformed= y_factor*y
        x_transformed= x_factor*x
        return [x_transformed,y_transformed]

    def __transformRealToEmulatedVel(self, x, y): 
        y_factor = -self.frameconfig['height']/(self.space_config['y_max']-self.space_config['y_min'])
        x_factor = self.frameconfig['width']/(self.space_config['x_max']-self.space_config['x_min'])
        y_transformed= y/y_factor
        x_transformed= x/x_factor
        return [x_transformed,y_transformed]

def eventKeyDownHandler(event,o):
    #Arrows Handle
    if event.key == pygame.K_UP:
        logging.debug('UP Key Pressed')
        #o.translate(0,-1)
    elif event.key == pygame.K_DOWN:
        logging.debug("DOWN Key Pressed")
        #o.translate(0,1)
    elif event.key == pygame.K_LEFT:
        logging.debug("LEFT Key Pressed")
        o.rotate(5)
    elif event.key == pygame.K_RIGHT:
        logging.debug("RIGHT Key Pressed")
        o.rotate(-5)
    
    #WASD Handle
    if event.key == pygame.K_w:
        logging.debug('W Key Pressed')
        o.translate(0,5)
    elif event.key == pygame.K_s:
        logging.debug("S Key Pressed")
        o.translate(0,-5)
    elif event.key == pygame.K_a:
        logging.debug("D Key Pressed")
        o.translate(-10,0)
    elif event.key == pygame.K_d:
        logging.debug("A Key Pressed")
        o.translate(10,0)
    
    else:
        logging.debug("Unknow Key Pressed: " + str(event.key))

def main():
    state_vector = controler.simulate(60)
    frameconfig={
        'width': 1200,
        'height': 600
    }
    screen = pygame.display.set_mode((frameconfig['width'],frameconfig['height']))
    player = pygame.image.load('drone.png.')
    player = pygame.transform.scale(player,(80,80))
    background = pygame.image.load('background.jpg') 
    background = pygame.transform.scale(background,(1200,600))
    screen.blit(background, (0, 0))
    o = Drone(player,frameconfig)
    prev_time=time.time()
    i=0
    while True:
        screen.blit(background, o.pos, o.pos)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                sys.exit()
            elif event.type == pygame.KEYDOWN:
                #eventKeyDownHandler(event,o)
                pass
        now=time.time()
        dt=now-prev_time
        prev_time=now
        di=math.floor(dt/5e-4)
        i=i+di
        if i  not in range(len(state_vector[1,:])):
            i=0
        o.posByCenter(state_vector[2,i],state_vector[3,i])
        o.set_rotation(state_vector[6,i]*180/np.pi)
        screen.blit(o.image, o.pos)
        pygame.display.update()

if __name__ == '__main__':
    logger=logging.basicConfig(encoding='utf-8', level=logging.DEBUG)
    main()