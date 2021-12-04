#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

from Puppy import Puppy

import time

# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.


# Create your objects here.
ev3 = EV3Brick()


# Write your program here.
ev3.speaker.beep()

class StateMachine:
    def __init__(self):
        self.current_state = 0
        self.final_state = 5
        self.robot = Puppy()
        # self.main_loop()
        pass

    def finalize(self):
        self.robot.move_head(0)

    def get_input(self):
        buttons_pressed = self.robot.ev3.buttons.pressed()
        n_buttons_pressed = len(buttons_pressed)
        touch_pressed = self.robot.touch_sensor.pressed()
        
        if n_buttons_pressed == 0 and touch_pressed:
            print('sensor: touch sensor')
            return 0
        if n_buttons_pressed == 1 and Button.CENTER in buttons_pressed:
            print('sensor: center button')
            return 1
        if n_buttons_pressed == 1 and Button.LEFT   in buttons_pressed:
            print('sensor: left button')
            return 2
        if n_buttons_pressed == 1 and Button.RIGHT  in buttons_pressed:
            print('sensor: right button')
            return 3
        if n_buttons_pressed == 1 and Button.UP     in buttons_pressed:
            print('sensor: up button')
            return 4
        if n_buttons_pressed == 1 and Button.DOWN   in buttons_pressed:
            print('sensor: down button')
            return 5
        
        # no input
        print('sensor: no input')
        return -1 

    def get_motor_state(self):
        if self.robot.left_leg_motor.angle() <= 0 \
            and self.robot.right_leg_motor.angle() <= 0:
            print('motor: sitting')
            return 1 # sitting state
        elif self.robot.left_leg_motor.angle() > 0 \
            and self.robot.right_leg_motor.angle() > 0:
            print('motor: standing 2')
            return 0 # standing state
        return -1

    def do_state_0_action(self):
        self.robot.ev3.speaker.say("Simon says, Nod your head.")
        self.robot.move_head(-20)
        self.robot.move_head(0)

    def do_state_1_action(self):
        self.robot.ev3.speaker.say("Simon says, Do leg lifts")
        self.robot.left_leg_motor.run_target(100, self.robot.STRETCH_ANGLE)
        self.robot.left_leg_motor.run_target(100, self.robot.STAND_UP_ANGLE)
        self.robot.right_leg_motor.run_target(100, self.robot.STRETCH_ANGLE)
        self.robot.right_leg_motor.run_target(100, self.robot.STAND_UP_ANGLE)
        
    def do_state_2_action(self):
        self.robot.ev3.speaker.say("Simon says, Sit down")
        self.robot.sit_down()

    def do_state_3_action(self):
        self.robot.ev3.speaker.say("Simon says, Stand up")
        self.robot.stand_up()

    def do_final_state_action(self):
        self.robot.ev3.speaker.say("Simon says, Good job!")
        self.robot.eyes = self.robot.HEART_EYES
        self.robot.ev3.speaker.say("Give yourself a pat on the back!")
        self.robot.eyes = self.robot.TIRED_EYES
        self.robot.ev3.speaker.say("Now, take some very deserved rest.")
        self.robot.eyes = self.robot.SLEEPING_EYES
        self.robot.move_head(self.robot.HEAD_DOWN_ANGLE)
        self.robot.ev3.speaker.play_file(SoundFile.SNORING)

    def do_state_action(self):
        current_state = self.current_state
        if current_state == 0:
            self.do_state_0_action()
        elif current_state == 1:
            self.do_state_1_action()
        elif current_state == 2:
            self.do_state_2_action()
        elif current_state == 3:
            self.do_state_3_action()
        elif current_state == 4:
            self.do_final_state_action()

    def do_state_transition(self, sensor_input_enum, motor_state_enum):
        # at state 0
        if self.current_state == 0:                                 
            if sensor_input_enum == -1 or sensor_input_enum == 1:   #   did nothing or pressed center button
                self.current_state = 0                              #     goto state 0
            elif sensor_input_enum == 4 or sensor_input_enum == 5:  #   pressed up or down button
                self.current_state = 3                              #     goto state 3
            elif sensor_input_enum == 0 and motor_state_enum == 1:  #   pressed touch sensor and is sitting
                self.current_state = 4                              #     goto state 4
        # at state 1
        elif self.current_state == 1:       
            if sensor_input_enum == 1:      #   pressed center button
                self.current_state = 0      #     goto state 0
            elif sensor_input_enum == 5:    #   pressed down button 
                self.current_state = 2      #     goto state 2
        # at state 2
        elif self.current_state == 2:       
            if sensor_input_enum == 4:      # pressed up button
                self.current_state = 3      #   goto state 3
            elif sensor_input_enum == 1:    # pressed center button
                self.current_state = 0      #   goto state 0
            elif sensor_input_enum == 0:    # pressed touch sensor
                self.current_state = 4      #   goto terminal state
        # at state 3
        elif self.current_state == 3:
            if sensor_input_enum == 2 or sensor_input_enum == 3:    # pressed left or right button
                self.current_state = 1                              #   goto state 0
            elif sensor_input_enum == 5:                            # pressed down
                self.current_state = 2                              #   goto state 2
            elif sensor_input_enum == 1:                            # pressed center button
                self.current_state = 0                              #   goto state 0
        else:
            # at final state! no where to go :(  (also, this should never be called...)
            pass
    
    def at_terminal_state(self):
        return self.current_state == 4

    def main_loop(self):
        while not self.at_terminal_state():
            self.do_state_action()
            sensor_input = self.get_input()
            motor_state = self.get_motor_state()
            self.do_state_transition(sensor_input, motor_state)
        
        # one last call for when at terminal state, so you do the terminal action
        self.do_state_action() 


if __name__ == "__main__":
    fsm = StateMachine()
    fsm.main_loop()
    wait(5000)
    fsm.finalize()

