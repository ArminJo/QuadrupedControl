<div align = center>

# QuadrupedControl a quadruped / mePed V2 spider robot control library
Arduino library for generating all the quadruped movements for controlling a mePed robot V2 with 8 servos.<br/>
It also controls the optional IR receiver and NeoPixels.

[![Badge License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)
 &nbsp; &nbsp; 
[![Badge Version](https://img.shields.io/github/v/release/ArminJo/QuadrupedControl?include_prereleases&color=yellow&logo=DocuSign&logoColor=white)](https://github.com/ServoEasing/QuadrupedControl/releases/latest)
 &nbsp; &nbsp; 
[![Badge Commits since latest](https://img.shields.io/github/commits-since/ArminJo/QuadrupedControl/latest?color=yellow)](https://github.com/ArminJo/QuadrupedControl/commits/master)
 &nbsp; &nbsp; 
[![Badge Build Status](https://github.com/ArminJo/QuadrupedControl/workflows/LibraryBuild/badge.svg)](https://github.com/ArminJo/QuadrupedControl/actions)
 &nbsp; &nbsp; 
![Badge Hit Counter](https://visitor-badge.laobi.icu/badge?page_id=ArminJo_QuadrupedControl)
<br/>
<br/>
[![Stand With Ukraine](https://raw.githubusercontent.com/vshymanskyy/StandWithUkraine/main/badges/StandWithUkraine.svg)](https://stand-with-ukraine.pp.ua)

Available as [QuadrupedControl](https://github.com/ArminJo/ServoEasing/tree/master/examples/QuadrupedControl) example of the Arduino library [ServoEasing](https://github.com/ArminJo/ServoEasing).

</div>

#### If you find this library useful, please give it a star.

<br/>

## YouTube video of mePed V2 in action controlled by the [QuadrupedControl example](https://github.com/ArminJo/QuadrupedControl/tree/master/examples/QuadrupedControl) and IR remote.
[![mePed V2 in actions](https://i.ytimg.com/vi/MsIjTRRUyGU/hqdefault.jpg)](https://youtu.be/MsIjTRRUyGU)

Smooth servo movements are controlled by the [Servo easing library for Arduino](https://github.com/ArminJo/ServoEasing).
For lifting the legs, the lift servos just use the easing type EASE_QUADRATIC_BOUNCING.

### A very simple and easy to understand version of controlling a mePed can be found [here](https://github.com/oracid/Easy-Quadruped-kinematic)

<br/>

# Installation
Install **[ServoEasing library](https://github.com/ArminJo/ServoEasing)** with *Tools > Manage Libraries...* or *Ctrl+Shift+I*. Use "ServoEasing" as filter string.<br/>
Then open the example **[QuadrupedControl](https://github.com/ArminJo/ServoEasing/tree/master/examples/QuadrupedControl)** available at File > Examples > Examples from Custom Libraries / ServoEasing.

<br/>

# Compile options / macros for this software
To customize the software to different requirements, there are some compile options / macros available.<br/>
Modify them by enabling / disabling them in the file *QuadrupedConfiguration.h*, or change the values if applicable.

| Name | Default value | Description |
|-|-|-|
| `QUADRUPED_HAS_IR_CONTROL` | disabled | IR remote control is enabled. |
| `IR_INPUT_PIN` | A0 | Pin for IR remote control sensor. |
| `PIN_BUZZER` | 3 | Pin for buzzer / piezo. |
| `QUADRUPED_HAS_NEOPIXEL` | disabled | NeoPattern animations on a 24 pieces Neopixel strip handled logically as 3 8 pieces strips is enabled. |
| `QUADRUPED_HAS_US_DISTANCE` | disabled | US distance sensor at pin A3 + A4 is enabled. The distance is displayed on the middle/front 8 pieces of the Neopixel strips. |
| `QUADRUPED_HAS_US_DISTANCE_SERVO` | disabled | A pan servo for the US distance sensor is enabled at pin 13. |
| `QUADRUPED_ENABLE_RTTTL` | disabled | The quadruped plays a melody at startup. |
| `QUADRUPED_1_WITH_DVD_REMOTE`, `QUADRUPED_2_WITH_LAFVIN_REMOTE`, `QUADRUPED_3_WITH_KEYES_CLONE_REMOTE` | disabled | 3 predefined configurations. |

<br/>

# Pictures
Bottom view of my mePed. You can see the two lipos connected parallel resulting in a 4.2 to 3.6 volt supply.
![Bottom view](pictures/mePed_bottom.jpg)

Using a PCA9685 expander for the servos, gaining pins for other purposes.
![PCA9685 expander](pictures/mePedWithPCA9685.jpg)