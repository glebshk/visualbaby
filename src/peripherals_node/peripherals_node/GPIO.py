from gpiozero import Device, Servo
from gpiozero.pins.pigpio import PiGPIOFactory

Device.pin_factory = PiGPIOFactory()

CONE_SERVO_PIN = 17


class GPIOController:
    def __init__(self):
        self.cone_servo = Servo(CONE_SERVO_PIN)
        self.attach_cone_servo()

    def attach_cone_servo(self):
        self.cone_servo.min()

    def detach_cone_servo(self):
        self.cone_servo.max()

    def cleanup(self):
        self.cone_servo.close()
