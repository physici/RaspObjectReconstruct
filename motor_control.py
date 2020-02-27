"""
Module for providing access to the tmcm1070 motor controller
via Raspberry Pi GPIO

author: Rainer Jacob
date:   Feb. 23, 2020
"""

import time
import RPi.GPIO as GPIO

# change the following constants according to hardware
# see https://www.element14.com/community/docs/DOC-92640/l/raspberry-pi-4-model-b-default-gpio-pinout-with-poe-header
PIN_STEP = 32
PIN_DIRECTION = 36
PIN_ENABLE_MOTOR = 40

class Motor():
    """
    Class that allows control of the motor.
    """

    def __init__(self):
        """
        Initialize Class instance
        """
        # prevent multiple instances accessing the hardware
        if getattr(self.__class__, '_has_instance', False):
            raise RuntimeError('Cannot create another instance')
        self.__class__._has_instance = True

        # allocate hardware ressources
        self.open()

        # motor is off
        self._enabled = False

        print('Setup done.')

    def __del__(self):
        """
        Finalize class. Close GPIO if required

        Returns
        -------
        None-type
        """
        self.close()
        del self

    def open(self):
        """
        Open motor connection
        """
        # set Pin numbering mode
        GPIO.setmode(GPIO.BOARD)

        # setup the hardware, initial low
        GPIO.setup(PIN_DIRECTION, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(PIN_STEP, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(PIN_ENABLE_MOTOR, GPIO.OUT, initial=GPIO.LOW)

    def close(self):
        """
        Close motor connection.
        """
        # Hardware ressources need to be released properly.
        print('Closing hardware connections')
        GPIO.cleanup(PIN_STEP)
        GPIO.cleanup(PIN_DIRECTION)
        GPIO.cleanup(PIN_ENABLE_MOTOR)

    def motor_enable(self):
        """
        Enable the motor
        """
        self._enabled = True
        GPIO.output(PIN_ENABLE_MOTOR, GPIO.HIGH)

    def motor_disable(self):
        """
        Disable the motor
        """
        self._enabled = False
        GPIO.output(PIN_ENABLE_MOTOR, GPIO.LOW)

    @property
    def motor_is_enabled(self):
        """
        Return the motor state.

        Returns
        -------
        Bool
            Motor enabled True/False
        """
        return self._enabled

    def step(self, direction=None):
        """
        Make a step.
        """
        # check motor enabled
        if self.motor_is_enabled:
            # get current direction
            tmp = int(GPIO.input(PIN_DIRECTION))
            # print('Current direction: {}'.format(tmp))
            # move in same direction as before when not specified
            if not direction:
                GPIO.output(PIN_STEP, GPIO.HIGH)
                time.sleep(0.1)
                GPIO.output(PIN_STEP, GPIO.LOW)
                time.sleep(0.5)
            # use direction given
            else:
                # same direction as before
                if direction == tmp:
                    GPIO.output(PIN_STEP, GPIO.HIGH)
                    time.sleep(0.1)
                    GPIO.output(PIN_STEP, GPIO.LOW)
                    time.sleep(0.5)
                # the other direction, but direction is resetted afterwards
                else:
                    # print('Turning in direction: {}'.format(direction))
                    GPIO.output(PIN_DIRECTION, direction)
                    GPIO.output(PIN_STEP, GPIO.HIGH)
                    time.sleep(0.1)
                    GPIO.output(PIN_STEP, GPIO.LOW)
                    time.sleep(0.5)
                    GPIO.output(PIN_DIRECTION, tmp)
        else:
            print('Motor is not enabled')

    @property
    def motor_direction(self):
        """
        Return the motor direction.

        Returns
        -------
        int
            Motor direction (level of corresponding pin)
        """
        return int(GPIO.input(PIN_DIRECTION))

    @motor_direction.setter
    def motor_direction(self, direction):
        """
        Set the motor direction.

        Parameters
        ----------
        direction : int
            Direction the motor should turn
        """
        GPIO.output(PIN_DIRECTION, direction)
