import RPi.GPIO as GPIO


class Wheel:
    def __init__(self, pin_a, pin_b, pin_en, speed=50, frequency=1000):
        self.pin_a = pin_a
        self.pin_b = pin_b
        self.enable = pin_en
        self.init_pins()
        self.pwm = GPIO.PWM(self.enable, frequency)
        self.stop()
        self.pwm.start(speed)

    def init_pins(self):
        GPIO.setup(self.pin_a, GPIO.OUT)
        GPIO.setup(self.pin_b, GPIO.OUT)
        GPIO.setup(self.enable, GPIO.OUT)

    def forward(self, speed):
        self.pwm.ChangeDutyCycle(speed)
        GPIO.output(self.pin_a, GPIO.LOW)
        GPIO.output(self.pin_b, GPIO.HIGH)

    def back(self, speed):
        self.pwm.ChangeDutyCycle(speed)
        GPIO.output(self.pin_a, GPIO.HIGH)
        GPIO.output(self.pin_b, GPIO.LOW)

    def stop(self):
        GPIO.output(self.pin_a, GPIO.LOW)
        GPIO.output(self.pin_b, GPIO.LOW)
