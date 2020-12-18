import time

import xbox
import RPi.GPIO as GPIO

import utils
from grid import Grid, Instruction
from wheel import Wheel
from vision import Vision


class Robot:
    def __init__(self):
        GPIO.setmode(GPIO.BOARD)
        self.left_wheel = Wheel(37, 40, 33, speed=35)
        self.right_wheel = Wheel(38, 36, 32, speed=35)
        self.eye = Vision()
        self.joy = xbox.Joystick()
        self.grid = Grid(4, 6)

    def move_forward(self):
        self.right_wheel.forward()
        self.left_wheel.forward()

    def move_back(self):
        self.right_wheel.back()
        self.left_wheel.back()

    def turn_left(self):
        self.right_wheel.forward()
        self.left_wheel.stop()

    def turn_right(self):
        self.right_wheel.stop()
        self.left_wheel.forward()

    def stop_movement(self):
        self.right_wheel.stop()
        self.left_wheel.stop()

    def test(self):
        while not self.joy.Back():
            # Show connection status
            utils.show("Connected:")
            utils.show_if(self.joy.connected(), "Y", "N")

            if self.joy.dpadUp():
                utils.show("UP")
                self.move_forward()

            elif self.joy.dpadDown():
                utils.show("BACK")
                self.move_back()

            elif self.joy.dpadLeft():
                utils.show("LEFT")
                self.turn_left()

            elif self.joy.dpadRight():
                utils.show("RIGHT")
                self.turn_right()

            else:
                utils.show("NO INP")
                self.stop_movement()

            utils.show(chr(13))
        self.joy.close()

    def explore_grid(self):
        instruction = Instruction.STRAIGHT
        while instruction != Instruction.STOP:
            cent_x, cent_y, sides, area = self.eye.get_main_contour_params()
            if sides > 4:
                time.sleep(1)
                instruction = self.grid.get_next_instruction(False)
                self.__handle_intersection(instruction)
            else:
                self.__follow_line(cent_x)

    def __follow_line(self, centroid):
        if centroid >= 320:
            self.turn_right()
        if 320 > centroid > 180:
            self.move_forward()
        if centroid <= 180:
            self.turn_left()

    def __handle_intersection(self, instruction):
        cent_x, cent_y, sides, area = self.eye.get_main_contour_params()
        while sides > 4:
            self.move_forward()
        time.sleep(.5)
        self.stop_movement()
    
    def intersection_test(self):
        self.stop_movement()
        time.sleep(3)
        while True:
            cent_x, cent_y, sides, area = self.eye.get_main_contour_params()
            utils.show("Number of sides currently visible: ", sides)
            if sides > 8:
                self.stop_movement() 
                time.sleep(3)
                while True:
                    cx,_,s,_ = self.eye.get_main_contour_params()
                    if s > 4:
                        self.move_forward()
                    else:
                        self.__handle_left_turn
                        break
                return
            else:
                self.__follow_line(cent_x)

            utils.show(chr(13))
                
                #self.__handle_intersection(Instruction.TURN_RIGHT)

if __name__ == "__main__":
    robot = Robot()
    #robot.test()
    robot.intersection_test()
