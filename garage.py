#!/usr/bin/python

""" 
  Author: Surendra Kane
  Script to control individual Raspberry Pi GPIO's.

  Modified by Craig Little Feb 2019

  Simplified the script to handle one relay
  Added statuses for door position
  Added callbacks  for reed switches to sense door position
  Modified the code to handle the momentary switches for a garage door
"""

import fauxmo
import logging
import time
import RPi.GPIO as GPIO
from enum import Enum
from debounce_handler import debounce_handler


gpio_ports = {'Garage Door':21}                          # change this to change the GPIO you use


class device_handler(debounce_handler):
    """ 
      Triggers on/off based on GPIO 'device' selected.
      Processes callbacks from height sensors
      Publishes the IP address of the Echo making the request.
    """

    class Status(Enum):
       OPEN = 1
       OPENING = 2
       CLOSED = 3
       CLOSING = 4

    TRIGGERS = {"Garage Door":50001}
    sensor_ports = {'top':13, 'middle':19, 'floor':26}   # the height sensors use these GPIOs
    status = Status.CLOSED

    def activate_door(self,port):                        # actions to mimick pressing the door opener
        GPIO.setup(port, GPIO.OUT)
        GPIO.output(port,GPIO.LOW)
        time.sleep(0.5)
        GPIO.output(port,GPIO.HIGH)


    def callback_top(self,channel):                      # the door has reached the top
        logging.info("Top callback")
        if self.status == self.Status.OPENING:
            self.status = self.Status.OPEN
        else:
            self.activate_door(gpio_ports[str('Garage Door')])
            self.status = self.Status.CLOSING
        logging.info("status {}".format(self.status))


    def callback_floor(self,channel):                    # the door has reached the floor
        logging.info("Floor callback")
        if self.status == self.Status.CLOSING:
            self.status = self.Status.CLOSED
        else:
            self.activate_door(gpio_ports[str('Garage Door')])
            self.status = self.Status.OPENING
        logging.info("status {}".format(self.status))


    def __init__(self):
        # set up the callbacks
        GPIO.setup(self.sensor_ports[str('floor')], GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.add_event_detect(self.sensor_ports[str('floor')], GPIO.RISING, callback=self.callback_floor, bouncetime=200)  # add rising edge detection on floor sensor
        GPIO.setup(self.sensor_ports[str('top')], GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.add_event_detect(self.sensor_ports[str('top')], GPIO.RISING, callback=self.callback_top, bouncetime=200)  # add rising edge detection on top sensor

        # Make sure the door isn't already open - close it for security
        if GPIO.input(4) == True:
          self.status = self.Status.CLOSING
          self.activate_door(gpio_ports[str('Garage Door')])


    def trigger(self,port,state):                        # take action from Alexa service
        Status = self.Status                             # get a copy of the Status's

        logging.info('port: {}, state: {}, status {}'.format(port, state, self.status))

        if state == True:                                # open the door
            if self.status == Status.OPEN or self.status == Status.OPENING:
                logging.info("Doing nothing")
                pass
            elif self.status == Status.CLOSED:           # if the door is closed, open it
                logging.info("Opening")
                self.activate_door(port)
                self.status = Status.OPENING
            elif self.status == Status.CLOSING:          # if the door is closing, stop it and open it
                logging.info("Stopping door, then opening")
                self.activate_door(port)
                time.sleep(1)
                self.activate_door(port)
                self.status = Status.OPENING
        else:                                            # close the door
            if self.status == Status.CLOSED or self.status == Status.CLOSING:
                logging.info("Doing nothing")
                pass
            elif self.status == Status.OPEN:             # if the door is opened, close it
                logging.info("Closing")
                self.activate_door(port)
                self.status = Status.CLOSING
            elif self.status == Status.OPENING:          # if the door is opening, stop it and close it
                logging.info("Stopping door, then closing")
                self.activate_door(port)
                time.sleep(1)
                self.activate_door(port)
                self.status = Status.CLOSING


    def act(self, client_address, state, name):          # Action received from Alexa service
        logging.info("State {} on {} from client {} gpio port: {}".format(state, name, client_address, gpio_ports[str(name)]))
        self.trigger(gpio_ports[str(name)],state)
        return True

if __name__ == "__main__":
    # set the log messages to display
    logging.basicConfig(level=logging.INFO)

    # set up the GPIO block
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

    # Startup the fauxmo server
    fauxmo.DEBUG = True
    p = fauxmo.poller()
    u = fauxmo.upnp_broadcast_responder()
    u.init_socket()
    p.add(u)

    # Register the device callback as a fauxmo handler
    d = device_handler()
    for trig, port in d.TRIGGERS.items():
        fauxmo.fauxmo(trig, u, p, None, port, d)

    # Loop and poll for incoming Echo requests
    logging.debug("Entering fauxmo polling loop")
    while True:
        try:
            # Allow time for a ctrl-c to stop the process
            p.poll(100)
            time.sleep(0.1)
        except Exception as e:
            logging.critical("Critical exception: " + str(e))
            GPIO.cleanup()
            break
