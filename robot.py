import time

import xbox
import RPi.GPIO as GPIO

import utils
from grid import Grid, Instruction
from wheel import Wheel
from vision import Vision
from simple_pid import PID

class Robot:
    def __init__(self):
        GPIO.setmode(GPIO.BOARD)
        self.top_speed = 35
        self.base_speed = 25
        self.left_wheel = Wheel(40, 37, 33, frequency=100)
        self.right_wheel = Wheel(36, 38, 32, frequency=100)
        self.eye = Vision()
        self.joy = xbox.Joystick()
        self.grid = Grid(4, 6)
        # PID PARAM (MIGHT MOVE)
        self.kp = 1
        self.kd = 0
        self.ki = 0
        self.error = 0
        self.integral = 0

    def move_robot(self, left_motor_speed, right_motor_speed):
        self.left_wheel.forward(left_motor_speed)
        self.right_wheel.forward(right_motor_speed)

    def move_forward(self, speed):
        self.right_wheel.forward(speed)
        self.left_wheel.forward(speed)

    def move_back(self, speed):
        self.right_wheel.back(speed)
        self.left_wheel.back(speed)

    def turn_left(self, speed, hard=False):
        self.left_wheel.forward(speed)
        if hard:
            self.right_wheel.back(speed)
        else:
            self.right_wheel.stop()

    def turn_right(self, speed, hard=False):
        if hard:
            self.left_wheel.back(speed)
        else:
            self.left_wheel.stop()
        self.right_wheel.forward(speed)

    def stop_movement(self):
        self.right_wheel.stop()
        self.left_wheel.stop()

    def test(self):
        while not self.joy.Back():
            self.eye.what_do_i_see()
            # Show connection status
            utils.show("Connected:")
            utils.show_if(self.joy.connected(), "Y", "N")

            if self.joy.dpadUp():
                utils.show("UP")
                self.move_forward(self.top_speed)

            elif self.joy.dpadDown():
                utils.show("BACK")
                self.move_back(self.top_speed)

            elif self.joy.dpadLeft():
                utils.show("LEFT")
                self.turn_left(self.top_speed, hard=True)

            elif self.joy.dpadRight():
                utils.show("RIGHT")
                self.turn_right(self.top_speed, hard=True)
            else:
                utils.show("NO INP")
                self.stop_movement()

            utils.show(chr(13))
        self.joy.close()

    def explore_grid(self):
        time.sleep(2)
        instruction = Instruction.STRAIGHT
        while instruction != Instruction.STOP:
            print("explore_grid"+str(instruction))
            i_saw = self.eye.what_do_i_see()
            if len(i_saw) == 0:
                continue
            if i_saw[2] > 10000 and i_saw[1][0]* i_saw[1][1] > 30000:
                self.stop_movement()
                time.sleep(3)
                instructions = self.grid.get_next_instruction(False)
                instruction = instructions[0]
                print("intersection_seen"+str(instruction))
                self.handle_intersection(instruction)
            else:
                self.__follow_line(i_saw[0])

    def simple_line_follower(self):
        time.sleep(2)
        while True:
            i_saw = self.eye.what_do_i_see()
            self.__follow_line(i_saw[0])

    def __follow_line(self, centroid):
        error = self.eye.roi_width//2 - centroid[0]
        prop = error
        self.integral += error
        derivative = error - self.last_error
        self.last_error = error

        correction = prop*self.kp + self.integral*self.ki + derivative*self.kd

        speed_left = self.base_speed + correction
        speed_right = self.base_speed - correction

        if speed_left < 0:
            speed_left = 0
        if speed_right < 0:
            speed_right = 0
        if speed_left > self.top_speed:
            speed_left = self.top_speed
        if speed_right > self.top_speed:
            speed_right = self.top_speed

        self.move_robot(speed_left, speed_right)

        return

        error_range = 20
        if centroid[0] < self.eye.roi_width//2 - error_range:
            self.turn_left(self.top_speed, hard=True)
        if self.eye.roi_width//2 - error_range <= centroid[0] <= self.eye.roi_width//2 + error_range:
            self.move_forward(self.top_speed)
        if centroid[0] > self.eye.roi_width//2 + error_range:
            self.turn_right(self.top_speed, hard=True)

    def handle_intersection(self, instruction):
        i_saw = self.eye.what_do_i_see()
        while i_saw[2] > 10000 and i_saw[1][0] * i_saw[1][1] > 30000:
            if instruction.name == Instruction.STRAIGHT.name:
                self.__follow_line(i_saw[0])
            elif instruction.name == Instruction.RIGHT_TURN.name:
                self.turn_right(self.top_speed+10)
            elif instruction.name == Instruction.LEFT_TURN.name:
                self.turn_left(self.top_speed+10)
            elif instruction.name == Instruction.U_TURN.name:
                print("cant do that right now")
            i_saw = self.eye.what_do_i_see()
        print("milestone")

if __name__ == "__main__":
    robot = Robot()
    robot.test()
    robot.simple_line_follower()
    #robot.explore_grid()
