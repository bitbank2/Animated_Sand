Animated Sand demo<br>
Project started 2/2/2019<br>
Written by Larry Bank<br>
bitbank@pobox.com<br>
Based on Phil Burgess's Animated Sand code<br>
<br>
[![Demo video](https://img.youtube.com/vi/ktr4Itf9JtU/0.jpg)](https://www.youtube.com/watch?v=ktr4Itf9JtU)

<br>
The purpose of this code is simulate grains of sand on a 2 dimensional
surface. An accelerometer provides the tilt sensing to give motion to the
grains. My purpose in writing this was to understand the logic of such
a physics demo and to optimize it. Starting from Phil's code, I made the
following changes:<br>
- Switched to a different accelerometer (MPU-6050 - what I had on hand)<br>
- Upgraded display from a 15x7 LED matrix to a 128x64 monochrome OLED (SSD1306)<br>
- Increased the number of grains from 20 to 250<br>
- Image matrix from 1 byte per pixel to 1 bit per pixel to match the display<br>
- Image memory mapping to match the memory layout of the display for easy updating<br>
- Simplified the pixel 'bounce' loop by removing some calls to abs() and simplifying other code<br>
- Removed the floating point math in the 2D vector limiting code<br>
- Added oledDumpBuffer() function to my oled_96 library which only transmits blocks of 16x8 pixels which have changed since the last call (aka dirty rectangle - for speed)<br>
- Fixed bug in collision logic which allowed grains to overlap<br>
<br>
This code depends on my oled_96 library. You can download it here:<br>
<br>
https://github.com/bitbank2/oled_96
<br>

If you find this code useful, please consider buying me a cup of coffee

[![paypal](https://www.paypalobjects.com/en_US/i/btn/btn_donateCC_LG.gif)](https://www.paypal.com/cgi-bin/webscr?cmd=_s-xclick&hosted_button_id=SR4F44J2UR8S4)

