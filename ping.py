import RPi.GPIO as GPIO
import time

class Ping:
    def __init__(self, echo_pin, trig_pin):
        self.echo = echo_pin
        self.trig = trig_pin
        self.init_pins()

    def init_pins(self):
        GPIO.setup(self.echo, GPIO.IN)
        GPIO.setup(self.trig, GPIO.OUT)

    def distance(self):
        GPIO.output(self.trig, True)
        time.sleep(0.00001)
        GPIO.output(self.trig, False)

        st_t = time.time()
        en_t = time.time()

        while GPIO.input(self.echo) == 0:
            st_t = time.time()

        while GPIO.input(self.echo) == 1:
            en_t = time.time()

        elapsed = en_t - st_t

        return (elapsed * 34300)/2
