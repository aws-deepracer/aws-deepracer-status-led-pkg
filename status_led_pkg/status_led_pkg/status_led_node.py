#!/usr/bin/env python

#################################################################################
#   Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.          #
#                                                                               #
#   Licensed under the Apache License, Version 2.0 (the "License").             #
#   You may not use this file except in compliance with the License.            #
#   You may obtain a copy of the License at                                     #
#                                                                               #
#       http://www.apache.org/licenses/LICENSE-2.0                              #
#                                                                               #
#   Unless required by applicable law or agreed to in writing, software         #
#   distributed under the License is distributed on an "AS IS" BASIS,           #
#   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.    #
#   See the License for the specific language governing permissions and         #
#   limitations under the License.                                              #
#################################################################################

"""
status_led_node.py

This module creates the status_led_node which contains the logic of the blink and
solid light effects for status led lights by the side of the DeepRacer device.
It provides services and functions to enable/disable the RGB GPIO ports for led lights,
start and stop the particular effect on the led lights.

The node defines:
    led_solid_service: A service to stop the current effect and set a solid light effect with
                       the color passed as parameter.
    led_blink_service: A service to stop the current effect and set a blink light effect with
                       the colors passed as parameter.
"""

import time
import threading
import queue
import rclpy
from rclpy.node import Node

from deepracer_interfaces_pkg.srv import (SetStatusLedSolidSrv,
                                          SetStatusLedBlinkSrv)
from status_led_pkg import (constants,
                            led_control)


class StatusLedNode(Node):
    """Node responsible managing the status led lights and the logic of the blink
       and solid light effects.
    """

    def __init__(self):
        """Create a StatusLedNode.
        """
        super().__init__("status_led_node")
        self.get_logger().info("status_led_node started")

        # Service to set a solid light effect with the color passed as parameter.
        self.led_solid_service = self.create_service(SetStatusLedSolidSrv,
                                                     constants.LED_SOLID_SERVICE_NAME,
                                                     self.set_led_solid_callback)

        # Service to set a solid blink effect with the color passed as parameter.
        self.led_blink_service = self.create_service(SetStatusLedBlinkSrv,
                                                     constants.LED_BLINK_SERVICE_NAME,
                                                     self.set_led_blink_callback)

        # Queue to maintain the effects to be executed.
        self.effect_queue = queue.Queue()

        # Thread where the effect loop is run looking for next effect to execute
        # from the effect queue.
        self.loop_thread = None

        # List of LEDControl objects for each led_index.
        self.leds = list()
        # List of thread objects running the effect for each led_index.
        self.threads = list()
        # List of thread event objects for each led_index.
        self.stop = list()

        for _ in range(0, 3):
            self.leds.append(None)
            self.threads.append(None)
            self.stop.append(threading.Event())

        # Heartbeat timer.
        self.timer_count = 0
        self.timer = self.create_timer(5.0, self.timer_callback)

    def timer_callback(self):
        """Heartbeat function to keep the node alive.
        """
        self.get_logger().debug(f"Timer heartbeat {self.timer_count}")
        self.timer_count += 1

    def __enter__(self):
        """Called when the node object is created using the 'with' statement.

        Returns:
           StatusLedNode : self object returned.
        """
        self.loop_thread = threading.Thread(target=self.effect_loop)
        self.loop_thread.start()
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        """Called when the object is destroyed.
        """
        self.effect_queue.put((None, None))
        self.loop_thread.join()
        for led in self.leds:
            if led is not None:
                led.disable()

    def set_led_solid_callback(self, req, res):
        """Callback method for the led_solid service. Calls the solid LED effect on the
           led to set the color passed as parameters.

        Args:
            req (SetStatusLedSolidSrv.Request): Request object with led_index (int),
                                                color(str) and hold(float) time sent
                                                as part of the service call.
            res (SetStatusLedSolidSrv.Response): Response object with error(int) flag
                                                 indicating successful service
                                                 call(0: success or 1: failure).

        Returns:
            SetStatusLedSolidSrv.Response: Response object with error(int) flag indicating
                                           successful service call(0: success or 1: failure).
        """
        try:
            self.start(constants.SupportedLEDEffects.SOLID_COLOR,
                       led_index=req.led_index,
                       color=req.color,
                       hold=req.hold)
            res.error = 0
            return res
        except Exception as ex:
            self.get_logger().error(f"Failed to set led solid light: {ex}")
            res.error = 1
            return res

    def set_led_blink_callback(self, req, res):
        """Callback method for the led_blink service. Calls the two color siren LED effect
           on the led to simulate blink with the colors passed as parameters.

        Args:
            req (SetStatusLedBlinkSrv.Request): Request object with led_index (int),
                                                color1(str), color2(str) and delay(float)
                                                time sent as part of the service call.
            res (SetStatusLedBlinkSrv.Response): Response object with error(int) flag
                                                 indicating successful service call
                                                 (0: success or 1: failure).

        Returns:
            SetStatusLedBlinkSrv.Response: Response object with error(int) flag indicating
                                           successful service call(0: success or 1: failure).
        """

        try:
            self.start(constants.SupportedLEDEffects.TWO_COLOR_SIREN,
                       led_index=req.led_index,
                       color1=req.color1,
                       color2=req.color2,
                       delay=req.delay)
            res.error = 0
            return res
        except Exception as ex:
            self.get_logger().error(f"Failed to set led blink light: {ex}")
            res.error = 1
            return res

    def get_led(self, led_index):
        """Helper function to return the LEDControl object for the led_index passed as parameter.

        Args:
            led_index (int): LED index to identify the status LED.

        Returns:
            LEDControl: LEDControl object that provides access to the RGB channel ports for the
                        specific LED light.
        """
        if self.leds[led_index] is None:
            led = led_control.LEDControl(led_index, self.get_logger())
            led.enable()
            self.leds[led_index] = led
        else:
            led = self.leds[led_index]
        return led

    def stop_effect(self, led_index):
        """Helper function to stop the current effect on the LED light for led_index
           by setting the stop event.

        Args:
            led_index (int): LED index to identify the status LED.
        """
        if self.threads[led_index] is not None:
            self.stop[led_index].set()
            self.threads[led_index].join()
            self.threads[led_index] = None

    def effect_solid_color(self,
                           led_index=constants.DEFAULT_LED_INDEX,
                           color=constants.LEDColors.BLUE,
                           hold=0.0):
        """Main function implementing the solid color effect with the color and the led index
           passed as parameter.

        Args:
            led_index (int, optional): LED index to identify the status LED.
                                       Defaults to constants.DEFAULT_LED_INDEX.
            color (str, optional): Color to set. Defaults to constants.LEDColors.BLUE.
            hold (float, optional): Least amount of time in seconds to hold the effect before
                                    waiting for next effect. Defaults to 0.0.
        """
        led = self.get_led(led_index)
        stop = self.stop[led_index]
        if color in constants.LED_COLOR_VALUES:
            led.rgb(constants.LED_COLOR_VALUES[color])
            time.sleep(hold)
            stop.wait()
            led.off()
        else:
            self.get_logger().error(f"Color not supported: {color}")

    def effect_two_color_siren(self,
                               led_index=constants.DEFAULT_LED_INDEX,
                               color1=constants.LEDColors.RED,
                               color2=constants.LEDColors.BLUE,
                               delay=0.1):
        """Main function implementing the blink color effect with the colors and the led index
           passed as parameter.

        Args:
            led_index (int, optional): LED index to identify the status LED.
                                       Defaults to constants.DEFAULT_LED_INDEX.
            color1 (str, optional): First color to set. Defaults to constants.LEDColors.RED.
            color2 (str, optional): Second color to set. Defaults to constants.LEDColors.BLUE.
            delay (float, optional): Time in seconds to hold each color for the blink effect.
                                     Defaults to 0.1.
        """
        led = self.get_led(led_index)
        stop = self.stop[led_index]
        if color1 in constants.LED_COLOR_VALUES and color2 in constants.LED_COLOR_VALUES:
            while not stop.isSet():
                led.rgb(constants.LED_COLOR_VALUES[color1])
                time.sleep(delay)
                led.rgb(constants.LED_COLOR_VALUES[color2])
                time.sleep(delay)
            led.off()
        else:
            self.get_logger().error(f"Colors not supported: {color1} {color2}")

    def effect_loop(self):
        """Main daemon function to pick next effect to execute from the effect queue.
        """
        while True:
            # Get new effect from the queue.
            new_effect, keyword_args = self.effect_queue.get()
            # Terminate?
            if new_effect is None:
                break

            # Get LED index.
            led_index = keyword_args.get("led_index", constants.DEFAULT_LED_INDEX)

            # Stop the current effect.
            self.stop_effect(led_index)

            # Start new effect.
            self.stop[led_index].clear()
            self.threads[led_index] = threading.Thread(target=new_effect, kwargs=keyword_args)
            self.threads[led_index].start()

        for led_index in range(0, 3):
            self.stop_effect(led_index)

    def start(self, effect, **keyword_args):
        """Helper function to add specific effect to the effect queue with the required arguments.

        Args:
            effect (int): Flag identifying the type of the LED effect.
        """
        if effect == constants.SupportedLEDEffects.SOLID_COLOR:
            self.effect_queue.put((self.effect_solid_color, keyword_args))
        elif effect == constants.SupportedLEDEffects.TWO_COLOR_SIREN:
            self.effect_queue.put((self.effect_two_color_siren, keyword_args))


def main(args=None):
    rclpy.init(args=args)
    with StatusLedNode() as status_led_node:
        rclpy.spin(status_led_node)
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        status_led_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
