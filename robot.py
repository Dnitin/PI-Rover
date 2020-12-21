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
        self.top_speed = 38
        self.left_wheel = Wheel(37, 40, 33, speed=self.top_speed)
        self.right_wheel = Wheel(38, 36, 32, speed=self.top_speed)
        self.eye = Vision()
        self.joy = xbox.Joystick()
        self.grid = Grid(4, 6)
        self.pid = PID(0.5, 0.01, 0, setpoint=self.eye.roi_width//2)
        self.pid.sample_time = 0.01
        self.pid.output_limits = (35, 45)

    def move_forward(self, speed):
        self.right_wheel.forward(speed)
        self.left_wheel.forward(speed)

    def move_back(self, speed):
        self.right_wheel.back(speed)
        self.left_wheel.back(speed)

    def turn_right(self, speed, hard=False):
        self.right_wheel.forward(self.top_speed)
        if hard:
            self.left_wheel.back(speed)
        else:
            self.left_wheel.stop()

    def turn_left(self, speed, hard=False):
        if hard:
            self.right_wheel.back(speed)
        else:
            self.right_wheel.stop()
        self.left_wheel.forward(self.top_speed)

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
            i_saw = self.eye.what_do_i_see()
            if len(i_saw) == 0:
                continue
            if i_saw[2] > 10000:
                self.stop_movement()
                time.sleep(3)
                print("intersection_seen")
                instruction = Instruction.LEFT_TURN #self.grid.get_next_instruction(False)
                self.handle_intersection(instruction)
            else:
                self.__follow_line(i_saw[0])

    def __follow_line(self, centroid):
        # error = int(320 - centroid[0])*self.kp
        speed = self.top_speed #self.pid(centroid[0])
        error_range = 15
        if centroid[0] < self.eye.roi_width//2 - error_range:
            self.turn_left(speed, hard=True)
        if self.eye.roi_width//2 - error_range <= centroid[0] <= self.eye.roi_width//2 + error_range:
            self.move_forward(speed)
        if centroid[0] > self.eye.roi_width//2 + error_range:
            self.turn_right(speed, hard=True)

    def __turn_left_at_intersection(self):
        i_saw = self.eye.what_do_i_see()
        while len(i_saw) == 0 or not self.__is_centroid_on_left_corner(i_saw[0]):
            self.turn_left(self.top_speed, hard=True)
            i_saw = self.eye.what_do_i_see()
        while i_saw[0][0] < self.eye.roi_width//2:
            self.turn_left(self.top_speed, hard=True)
            i_saw = self.eye.what_do_i_see()

    def __is_centroid_on_left_corner(self, centroid):
        return 0 < centroid[0] < 75

    def __turn_right_at_instruction(self):
        i_saw = self.eye.what_do_i_see()
        while len(i_saw) == 0 or i_saw[0][0] < 350:
            self.turn_right(hard=True)
            i_saw = self.eye.what_do_i_see()
        while i_saw[0][0] > 260 or i_saw[0][0] < 240:
            self.turn_right(hard=True)
            i_saw = self.eye.what_do_i_see()

    def handle_intersection(self, instruction):
        i_saw = self.eye.what_do_i_see()
        while i_saw[2] > 10000:
            self.move_forward(self.top_speed)
            i_saw = self.eye.what_do_i_see()
        self.stop_movement()
        print("milestone")
        time.sleep(5)
        return

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
    robot.test()
    robot.explore_grid()
