import xbox
import RPi.GPIO as GPIO

import utils
from wheel import Wheel
from vision import Vision


class Robot:
    def __init__(self):
        GPIO.setmode(GPIO.BOARD)
        self.left_wheel = Wheel(40, 37, 33)
        self.right_wheels = Wheel(36, 38, 32)
        self.eye = Vision()
        self.joy = xbox.Joystick()

    def move_forward(self):
        self.right_wheels.forward()
        self.left_wheel.forward()

    def turn_left(self):
        self.right_wheels.forward()
        self.left_wheel.stop()

    def turn_right(self):
        self.right_wheels.stop()
        self.left_wheel.forward()

    def stop_movement(self):
        self.right_wheels.stop()
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
                self.stop_movement()

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


if __name__ == "__main__":
    robot = Robot()
    robot.test()
