#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
import pygame
from pygame.locals import *

#-------- Rotation --------#
# JoyCon R, Vertical   => 0
# JoyCon L, Vertical   => 1
# JoyCon R, Horizontal => 2
# JoyCon L, Horizontal => 3
ROTATION = 0

# Key Config
#              <▲ : UP Linear>
#  <◀ : UP Angular>       <▶ : DOWN Angular>
#              <▼ : DOWN Linear>
BTN_NAME = ['A', 'X', 'B', 'Y']
if ROTATION == 0:
    BTN_NUM = [1, 2, 3, 0]
elif ROTATION == 1:
    BTN_NUM = [2, 1, 0, 3]
elif ROTATION == 2:
    BTN_NUM = [3, 0, 2, 1] 
elif ROTATION == 3:
    BTN_NUM = [0, 3, 1, 2]

# Press button to change speed.
SPEED_DIF = 0.1
# When you add 0.1, machine adds 0.1000000001.
TOLERANCE = 0.01

class joycon_controller(Node):

    def __init__(self):
        super().__init__('joycon_controller')
        self.twist_pub = self.create_publisher(Twist, "/turtle1/cmd_vel", 100)

        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # JoyStick Initialization
        pygame.joystick.init()
        self.joystick0 = pygame.joystick.Joystick(0)
        self.joystick0.init()
        self.joystickx = 0
        self.joysticky = 0
        pygame.init()

        # Initializing Default Count and Speed
        self.button_cnt = 0
        self.coff_linearx = 1
        self.coff_angularz = 1

        # Twist Initialization
        self.vel = Twist()
        self.vel.linear.x = float(0)
        self.vel.linear.y = float(0)
        self.vel.linear.z = float(0)
        self.vel.angular.x = float(0)
        self.vel.angular.y = float(0)
        self.vel.angular.z = float(0)
        self.twist_pub.publish(self.vel)

        # Print Operation Description
        print() 
        print( f"▲ : UP Linear  {BTN_NAME[BTN_NUM[0]]}, ▼ : DOWN Linear  {BTN_NAME[BTN_NUM[1]]}" )
        print( f"◀ : UP Angular {BTN_NAME[BTN_NUM[2]]}, ▶ : DOWN Angular {BTN_NAME[BTN_NUM[3]]}" )
        print()

    def timer_callback(self):
        eventlist = pygame.event.get()

        # Event Processing
        for e in eventlist:
            if e.type == QUIT:
                return
            # Stick Processing
            if e.type == pygame.locals.JOYHATMOTION:
                self.button_cnt += 1
                self.joystickx, self.joysticky = self.joystick0.get_hat(0)
                print(f"linear x: {self.joystickx * self.coff_linearx:5.2f}, angular z: {self.joysticky * self.coff_angularz:5.2f}")
            # Button Processing
            # Set the Limit 0.1 to 9.9
            elif e.type == pygame.locals.JOYBUTTONDOWN:
                self.button_cnt += 1
                if   e.button == BTN_NUM[0] and self.coff_linearx  < 9.9 - TOLERANCE:
                    self.coff_linearx += SPEED_DIF
                elif e.button == BTN_NUM[1] and self.coff_linearx  > 0.1 + TOLERANCE:
                    self.coff_linearx -= SPEED_DIF
                elif e.button == BTN_NUM[2] and self.coff_angularz < 9.9 - TOLERANCE:
                    self.coff_angularz += SPEED_DIF
                elif e.button == BTN_NUM[3] and self.coff_angularz > 0.1 + TOLERANCE:
                    self.coff_angularz -= SPEED_DIF
                print(f"linear x: {self.coff_linearx:5.2f}, angular z: {self.coff_angularz:5.2f}")

        # Calculate Twist from stick input.
        if   ROTATION == 0:
            self.vel.linear.x  = float( self.joystickx * self.coff_linearx)
            self.vel.angular.z = float( self.joysticky * self.coff_angularz)
        elif ROTATION == 1:
            self.vel.linear.x  = float(-self.joystickx * self.coff_linearx)
            self.vel.angular.z = float(-self.joysticky * self.coff_angularz)
        elif ROTATION == 2:
            self.vel.linear.x  = float( self.joysticky * self.coff_angularz)
            self.vel.angular.z = float(-self.joystickx * self.coff_linearx)
        elif ROTATION == 3:
            self.vel.linear.x  = float(-self.joysticky * self.coff_angularz)
            self.vel.angular.z = float( self.joystickx * self.coff_linearx)
        # Publish Twist
        self.twist_pub.publish(self.vel)

        # Print the operation description every 10 lines.
        # if self.button_cnt>=10:
        #     print() 
        #     print( f"▲ : UP Linear  {BTN_NAME[BTN_NUM[0]]}, ▼ : DOWN Linear  {BTN_NAME[BTN_NUM[1]]}" )
        #     print( f"◀ : UP Angular {BTN_NAME[BTN_NUM[2]]}, ▶ : DOWN Angular {BTN_NAME[BTN_NUM[3]]}" )
        #     print()
        #     self.button_cnt = 0

def ros_main(args=None):
    rclpy.init(args=args)
    
    joycon_controller = joyconController()
    rclpy.spin(joycon_controller)

    joycon_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    ros_main()