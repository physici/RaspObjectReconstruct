"""
Module for control of a laser connected via transistor switch
via Raspberry Pi GPIO

author: Rainer Jacob
date:   Mar. 5, 2020
"""

import RPi.GPIO as GPIO

# change the following constants according to hardware
# see https://www.element14.com/community/docs/DOC-92640/l/raspberry-pi-4-model-b-default-gpio-pinout-with-poe-header
LASER_ENABLE = 7

class Laser():
    """
    Class that allows control of the Laser.
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
        Open Laser connection
        """
        # set Pin numbering mode
        GPIO.setmode(GPIO.BOARD)

        # setup the hardware, initial low
        GPIO.setup(LASER_ENABLE, GPIO.OUT, initial=GPIO.LOW)

    def close(self):
        """
        Close Laser connection.
        """
        # Hardware ressources need to be released properly.
        print('Closing hardware connections')
        self.laser_disable()
        GPIO.cleanup(LASER_ENABLE)

    def laser_enable(self):
        """
        Enable the laser
        """
        GPIO.output(LASER_ENABLE, GPIO.HIGH)

    def laser_disable(self):
        """
        Disable the laser
        """
        GPIO.output(LASER_ENABLE, GPIO.LOW)

    @property
    def laser_is_enabled(self):
        """
        Return the laser state.

        Returns
        -------
        Bool
            laser enabled True/False
        """
        return GPIO.input(LASER_ENABLE)
