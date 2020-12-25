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
        self.top_speed = 18
        self.left_wheel = Wheel(40, 37, 33, frequency=100)
        self.right_wheel = Wheel(36, 38, 32, frequency=100)
        self.eye = Vision()
        self.joy = xbox.Joystick()
        self.grid = Grid(4, 6)
        self.pid = PID(0.9, 0, 0, setpoint=self.eye.roi_width//2)
        self.pid.sample_time = 0.01
        self.pid.output_limits = (15, 30)

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
        op = self.pid(error)
        print("error : " + str(error) + " speed : " + str(op))
        if error < 0:
            self.turn_left(op, hard=True)
        elif error > 0:
            self.turn_right(op, hard=True)
        else:
            self.move_forward(op)

        return


        speed = self.top_speed #self.pid(centroid[0])
        error_range = 20
        if centroid[0] < self.eye.roi_width//2 - error_range:
            self.turn_left(speed, hard=True)
        if self.eye.roi_width//2 - error_range <= centroid[0] <= self.eye.roi_width//2 + error_range:
            self.move_forward(speed)
        if centroid[0] > self.eye.roi_width//2 + error_range:
            self.turn_right(speed, hard=True)

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
