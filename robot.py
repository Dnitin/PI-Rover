import time

import xbox
import RPi.GPIO as GPIO

import utils
from grid import Grid, Instruction
from wheel import Wheel
from ping import Ping
from vision import Vision
import sys


class Robot:
    def __init__(self):
        GPIO.setmode(GPIO.BOARD)
        self.top_speed = 30
        self.base_speed = 15
        self.left_wheel = Wheel(37, 40, 33, frequency=100)
        self.right_wheel = Wheel(36, 38, 32, frequency=100)
        self.ping = Ping(5, 3)
        self.eye = Vision()
        self.joy = None
        self.grid = Grid(4, 6)
        # PID PARAM (MIGHT MOVE)
        self.kp = 0.15
        self.kd = 1
        self.ki = 0
        self.error = 0
        self.integral = 0

    def init_xbox(self):
        return xbox.Joystick()

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
        self.joy = self.init_xbox()
        while not self.joy.Back():
            self.eye.what_do_i_see()
            dis = self.ping.distance()
            utils.show(dis)
            # Show connection status
            utils.show("Connected:")
            utils.show_if(self.joy.connected(), "Y", "N")

            if self.joy.dpadUp() and dis > 15:
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

    def is_intersection_visible(self, i_saw):
        return i_saw[1][0] * i_saw[1][1] > 15000

    def explore_grid(self):
        time.sleep(2)
        instruction = Instruction.STRAIGHT
        while instruction != Instruction.STOP:
            print("explore_grid"+str(instruction))
            i_saw = self.eye.what_do_i_see()
            if len(i_saw) == 0:
                continue
            distance = self.ping.distance()
            is_intersection = False
            is_obstacle = False
            if self.is_intersection_visible(i_saw):
                self.stop_movement()
                time.sleep(2)
                is_intersection = True
                count = 1
                while count > 0:
                    self.move_forward(self.base_speed)
                    time.sleep(0.1)
                    i_saw = self.eye.what_do_i_see()
                    is_intersection = is_intersection and self.is_intersection_visible(i_saw)
                    count -= 1

            if distance <= 10:
                self.stop_movement()
                time.sleep(2)
                is_obstacle = True
                count = 2
                while count > 0:
                    distance = self.ping.distance()
                    is_obstacle = is_obstacle and (distance <= 10)
                    count -= 1

            if is_intersection or is_obstacle:
                self.stop_movement()
                time.sleep(1)
                print("exiting: "+str(distance))
                # exit()
                instructions = self.grid.get_next_instruction(is_obstacle)
                instruction = instructions[0]
                print("intersection_seen"+str(instruction)+str(distance))
                self.handle_intersection(instruction)
            else:
                self.__follow_line(i_saw[0])


    def simple_line_follower(self):
        time.sleep(2)
        while True:
            i_saw = self.eye.what_do_i_see()
            if len(i_saw) > 0:
                self.__follow_line(i_saw[0])

    def __follow_line(self, centroid):
        error = self.eye.roi_width//2 - centroid[0]
        prop = error
        self.integral += error
        derivative = error - self.error
        self.error = error

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
        
        #print("error:"+str(error)+" correction:"+str(correction)+" s_l:"+str(speed_left)+" s_r:"+str(speed_right))
        self.move_robot(speed_left, speed_right)

    def handle_back_gear(self):
        i_saw = self.eye.what_do_i_see()
        while not self.is_intersection_visible(i_saw):
            self.move_back(self.base_speed)
            i_saw = self.eye.what_do_i_see()
        self.stop_movement()
        print("exitttttttttttt")

    def handle_intersection(self, instruction):
        if instruction.name == Instruction.BACK.name:
            self.handle_back_gear()
            return

        i_saw = self.eye.what_do_i_see()
        while self.is_intersection_visible(i_saw):
            if instruction.name == Instruction.STRAIGHT.name:
                self.move_forward(self.base_speed)
            elif instruction.name == Instruction.RIGHT_TURN.name:
                self.turn_right(self.base_speed+7)
            elif instruction.name == Instruction.LEFT_TURN.name:
                self.turn_left(self.base_speed+7)
            elif instruction.name == Instruction.U_TURN.name:
                print("adjustment needed")
                self.move_forward(self.base_speed)
            i_saw = self.eye.what_do_i_see()

        if instruction.name == Instruction.U_TURN.name:
            time.sleep(0.2)
            self.turn_left(self.base_speed, hard=True)
            time.sleep(3)
        print("milestone")


if __name__ == "__main__":
    robot = Robot()
    for arg in sys.argv:
        if arg == 'explore_grid':
            robot.explore_grid()
        elif arg == 'line_follower':
            robot.simple_line_follower()
        else:
            robot.test()
