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
        self.top_speed = 95
        self.left_wheel = Wheel(37, 40, 33, speed=self.top_speed)
        self.right_wheel = Wheel(38, 36, 32, speed=self.top_speed)
        self.eye = Vision()
        self.joy = xbox.Joystick()
        self.grid = Grid(4, 6)
        self.kp = 0.75
        self.ki = 1
        self.kd = 1

    def move_forward(self):
        self.right_wheel.forward()
        self.left_wheel.forward()

    def move_back(self):
        self.right_wheel.back()
        self.left_wheel.back()

    def turn_left(self, hard=False):
        self.right_wheel.forward()
        if hard:
            self.left_wheel.back()
        else:
            self.left_wheel.stop()

    def turn_right(self, hard=False):
        if hard:
            self.right_wheel.back()
        else:
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
            centroid, area, sides, angle, is_obstacle = self.eye.what_do_i_see()
            if sides > 4:
                time.sleep(1)
                instruction = self.grid.get_next_instruction(False)
                self.handle_intersection(instruction)
            else:
                self.__follow_line(centroid, angle)

    def __follow_line(self, centroid, angle):
        # error = int(320 - centroid[0])*self.kp
        if centroid[0] >= 250:
            self.turn_left()
        if 250 > centroid[0] > 240:
            self.move_forward()
        if centroid[0] <= 240:
            self.turn_right()

    def __turn_left_at_intersection(self):
        centroid, _, _, _, _ = self.eye.what_do_i_see()
        while len(centroid) == 0 or centroid[0] > 150:
            self.turn_left(hard=True)
            centroid, _, _, _, _ = self.eye.what_do_i_see()
        while centroid[0] > 325 or centroid[0] < 315:
            self.turn_left(hard=True)
            centroid, _, _, _, _ = self.eye.what_do_i_see()

    def __turn_right_at_instruction(self):
        centroid, _, _, _, _ = self.eye.what_do_i_see()
        while len(centroid) == 0 or centroid[0] < 350:
            self.turn_right(hard=True)
            centroid, _, _, _, _ = self.eye.what_do_i_see()
        while centroid[0] > 260 or centroid[0] < 240:
            self.turn_right(hard=True)
            centroid, _, _, _, _ = self.eye.what_do_i_see()

    def handle_intersection(self, instruction):
        centroid, area, sides, angle, is_obstacle = self.eye.what_do_i_see()
        while sides > 4:
            self.move_forward()
        time.sleep(.2)
        self.stop_movement()

        if instruction == Instruction.STRAIGHT:
            return
        elif instruction == Instruction.RIGHT_TURN:
            self.__turn_right_at_instruction()
        elif instruction == Instruction.LEFT_TURN:
            self.__turn_left_at_intersection()
        elif instruction == Instruction.U_TURN:
            self.__turn_left_at_intersection()
            self.__turn_left_at_intersection()


if __name__ == "__main__":
    robot = Robot()
    # robot.test()
    robot.handle_intersection()
