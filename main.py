import pygame
import copy
import sys
import math
import logging
import controler
import time
import numpy as np
import threading
from dataclasses import dataclass

class Drone:
    space_config = {
            'x_min': -60.0,
            'x_max': 60.0,
            'y_min': -10.0,
            'y_max': 60.0,
    }
    def __init__(self, image,frameconfig): 
        self.image = image
        self.__base_image=image
        self.angle = 0
        self.frameconfig=frameconfig
        self.space_config=Drone.space_config
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

class Target:
    @dataclass
    class Point:
        x: float
        y: float

    def __init__(self,x_co,y_co):
        self.t_point = Target.Point(x_co,y_co)
        self.velocity =2.0 #base velocity
        self.aceleration = 0.0 #only for muvControl
        self.space_config= Drone.space_config
        self.instant_vector=[0,0]
        self.refresh=self.mruControl

    def getVector(self):
        return np.array([self.t_point.x,self.t_point.y]).transpose()
    
    def setX(self,value):
        new_x= value 
        if new_x <= self.space_config['x_max'] and new_x >= self.space_config['x_min']:
            self.t_point.x = new_x

    def setY(self,value):
        new_y= value
        if new_y <= self.space_config['y_max'] and new_y >= self.space_config['y_min']:
            self.t_point.y =new_y

    def translateX(self,value):
        new_x= self.t_point.x + value 
        if new_x <= self.space_config['x_max'] and new_x >= self.space_config['x_min']:
            self.t_point.x = new_x

    def translateY(self,value):
        new_y= self.t_point.y + value
        if new_y <= self.space_config['y_max'] and new_y >= self.space_config['y_min']:
            self.t_point.y =new_y

    def setLocal(self,xy_tuple):
        self.setX(xy_tuple[0])
        self.setY(xy_tuple[1])

    def mruControl(self,instant):
        self.translateX(self.velocity*instant[0])
        self.translateY(self.velocity*instant[1])

    #TODO
    def muvControl(self):
        pass

def eventKeyHandler(event,o):
    ###implementing  mru control
    
    #region for muvControl ignore it for now
    sighn=None
    logging.debug('Key Handler')
    if event.type == pygame.KEYUP:
        logging.debug("KeyUp Event")
        sighn = -1.0
    elif event.type == pygame.KEYDOWN:
        logging.debug("KeyDown Event")
        sighn = 1.0
    #endregion
        
    if event.key == pygame.K_UP:
        logging.debug('UP Key Pressed')
        o.refresh((0,1))
        return
    elif event.key == pygame.K_DOWN:
        logging.debug("DOWN Key Pressed")
        o.refresh((0,-1))
        return
    elif event.key == pygame.K_LEFT:
        logging.debug("LEFT Key Pressed")
        o.refresh((-1,0))
        return
    elif event.key == pygame.K_RIGHT:
        logging.debug("RIGHT Key Pressed")
        o.refresh((1,0))
        return
    
    if event.key == pygame.K_w:
        logging.debug('W Key Pressed')
        o.refresh((0,1))
        return
    elif event.key == pygame.K_s:
        logging.debug("S Key Pressed")
        o.refresh((0,1))
        return
    elif event.key == pygame.K_a:
        logging.debug("D Key Pressed")
        o.refresh((0,1))
        return
    elif event.key == pygame.K_d:
        logging.debug("A Key Pressed")
        o.refresh((0,1))
        return

    
    logging.debug("Unknow Key Pressed: " + str(event.key))

def mainFrameHandle(simulator):
    frameconfig={
        'width': 1200,
        'height': 600
    }
    screen = pygame.display.set_mode((frameconfig['width'],frameconfig['height']))
    player = pygame.image.load('drone.png.')
    player = pygame.transform.scale(player,(160,80))
    background = pygame.image.load('background.jpg') 
    background = pygame.transform.scale(background,(1200,600))
    screen.blit(background, (0, 0))
    o = Drone(player,frameconfig)
    target = simulator.r_
    showing_state = [0,0,0]
    time_cut_acum =0
    while True:
        last_delta_tick = copy.deepcopy(np.minimum(simulator.last_delta_tick,80))
        state_vector = copy.deepcopy(simulator.x)
        screen.blit(background, o.pos, o.pos)
        
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                exit()
            elif event.type == pygame.KEYDOWN or event.type == pygame.KEYUP:
                eventKeyHandler(event,target)


        memory_ocup= len(state_vector[1,:])
        show_row= memory_ocup-2*last_delta_tick
        if (testDataQuality(showing_state,state_vector[2,show_row],state_vector[3,show_row],state_vector[6,show_row])):
            if time_cut_acum>5:
                o.posByCenter(showing_state[0],showing_state[1])
                o.set_rotation(showing_state[2]*180/np.pi)
        else :
            time_cut_acum =0
        time_cut_acum+=1
        showing_state = [state_vector[2,show_row],state_vector[3,show_row],state_vector[6,show_row]] 
        screen.blit(o.image, o.pos)
        pygame.display.update()

def testDataQuality(showing_state,x,y,rot):
    threshold = 1
    delta_x = showing_state[0]-x
    delta_y = showing_state[1]-y
    delta_rot = showing_state[2]-rot
    norma = math.sqrt(delta_x**2+delta_y**2+delta_rot**2)
    if (norma>1):
        logging.debug('norma:' + str(norma))
        return False
    else:
        return True

def simulationHandle(simulator):
    while True:
        simulator.nextStep()

def main():
    target = Target(0,0)
    simulator = controler.IteractiveSimulator(35,target)
    simulator.fillTimeWindow()
    sim_thread = threading.Thread(target=simulationHandle,args=(simulator,),daemon=True)
    sim_thread.start()
    frame_thread = threading.Thread(target=mainFrameHandle,args=(simulator,))
    frame_thread.start()
    try:
        while True:
            a=input()
            x, y=[float(i) for i in a.split(',')]
            print('target point ({},{})'.format(x, y))
            target.setLocal((x,y))
    finally:
        exit()


if __name__ == '__main__':
    logger=logging.basicConfig(encoding='utf-8', level=logging.ERROR)
    main()