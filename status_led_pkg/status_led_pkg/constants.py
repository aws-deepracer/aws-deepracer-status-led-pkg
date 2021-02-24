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


LED_SOLID_SERVICE_NAME = "led_solid"
LED_BLINK_SERVICE_NAME = "led_blink"

# Base path of the GPIO ports.
GPIO_ROOT_PATH = "/sys/class/gpio"

# Default LED index.
DEFAULT_LED_INDEX = 0

# Status light LED GPIO port matrix.
# Cols: led indices; rows: r, g, b channel ports.
LED_PORTS = (
    (448, 447, 437),
    (446, 445, 443),
    (450, 457, 458)
)


class SupportedLEDEffects():
    """Supported led effects.
    """
    SOLID_COLOR = 0
    TWO_COLOR_SIREN = 1


class LEDColors():
    """LED color values that are passed as part of led_blink and led_solid service calls.
    """
    RED = "red"
    GREEN = "green"
    BLUE = "blue"
    BLACK = "black"
    WHITE = "white"
    NO_COLOR = "no_color"
    ON = "on"


# R,G,B Channel mapping for each of the LED color values.
LED_COLOR_VALUES = {
    LEDColors.RED: (1, 0, 0),
    LEDColors.GREEN: (0, 1, 0),
    LEDColors.BLUE: (0, 0, 1),
    LEDColors.BLACK: (0, 0, 0),
    LEDColors.WHITE: (1, 1, 1),
    LEDColors.NO_COLOR: (0, 0, 0),
    LEDColors.ON: (1, 1, 1)
}
