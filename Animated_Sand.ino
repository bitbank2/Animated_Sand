//
// Animated sand demo
// Written by Larry Bank (bitbank@pobox.com), https://github.com/bitbank2
// Project started 2/2/2019
// original code by Phil Burgess of Adafruit:
// https://github.com/adafruit/Adafruit_Learning_System_Guides/tree/master/LED_Sand
//
// I built an Adafruit Feather "wing" for another project which consists of a SSD1306 I2C OLED
// two push buttons and an MPU-6050 accelerometer/gyroscope. I changed the project design and
// was no longer going to use the hat. I then saw the animated sand demo and thought it
// would be a fun exercise to port the code to the SSD1306 and optimize it.
// The original code animated 20 grains on a small 15x7 LED matrix.
//
// Tested on the Adafruit nRF52832 Feather and the Arduino MKR Zero
//
// What did I change?
// 1) Switched to a different accelerometer (MPU-6050 - what I had on hand)
// 2) Upgraded display from a 15x7 LED matrix to a 128x64 monochrome OLED (SSD1306)
// 3) Increased the number of grains from 20 to 250
// 4) Image matrix from 1 byte per pixel to 1 bit per pixel to match the display
// 5) Image memory mapping to match the memory layout of the display for easy updating
// 6) Simplified the pixel 'bounce' loop by removing some calls to abs() and simplifying other code
// 7) Removed the floating point math in the 2D vector limiting code
// 8) Added oledDumpBuffer() function to my oled_96 library which only transmits blocks of 16x8
//    pixels which have changed since the last call (aka dirty rectangle - for speed)
// 9) Fixed bug in collision logic which allowed grains to overlap
//
// Dependencies - oled_96 library: https://github.com/bitbank2/oled_96
//
#include <oled_96.h>
#include <Wire.h>

static byte imu_addr;
// frame counter for updating display every other pixel update
static int iFrame;

// Number of grains of sand
#define N_GRAINS     250
// Display width in pixels
#define WIDTH        128
// Display height in pixels
#define HEIGHT       64
// Maximum redraw rate, frames/second
#define MAX_FPS      120
 
// The 'sand' grains exist in an integer coordinate space that's 256X
// the scale of the pixel grid, allowing them to move and interact at
// less than whole-pixel increments.
#define MAX_X (WIDTH  * 256 - 1) // Maximum X coordinate in grain space
#define MAX_Y (HEIGHT * 256 - 1) // Maximum Y coordinate
struct Grain {
  int16_t  x,  y; // Position
  int16_t vx, vy; // Velocity
} grain[N_GRAINS];
uint32_t prevTime = 0;           // Used for frames-per-second throttle
uint8_t img[WIDTH * (HEIGHT/8)]; // Copy of pixel map laid out the same way as the SSD1306

// Wrapper function to write I2C data on Arduino
static void I2CWrite(int iAddr, unsigned char *pData, int iLen)
{
  Wire.beginTransmission(iAddr);
  Wire.write(pData, iLen);
  Wire.endTransmission();
} /* I2CWrite() */

static byte I2CReadRegister(byte bAddr, byte bRegister, byte *pData, byte iLen)
{
byte ucTemp[2];
byte x;

  ucTemp[0] = bRegister;
  I2CWrite(bAddr, ucTemp, 1); // write address of register to read
  Wire.requestFrom(bAddr, iLen); // request N bytes
  x = 0;
  while (x < iLen && Wire.available())
  {
     pData[x] = Wire.read();
     x++;
  }
  return x;
} /* I2CReadRegister() */

void mpu6050Init(byte bAddr)
{
uint8_t ucTemp[4];
byte i;

  imu_addr = bAddr;
  i = I2CReadRegister(imu_addr, 0x75, ucTemp, 1); // Get ID
  if (i != 1 || ucTemp[0] != 0x68)
  {
//     printf("Error, ID doesn't match 0x68; wrong device?\n");
//     printf("Value read = %02x\n", ucTemp[0]);
     return;
  }
 // pwr mgmt 1 register
 // bits: 7=reset, 6=sleep, 5=cycle, 4=n/a, 3=temp_disable, 2-0=clock select
  ucTemp[0] = 0x6b; // power management 1 register
  ucTemp[1] = 0x00; // disable sleep mode
  I2CWrite(imu_addr, ucTemp, 2);
} /* mpu6050Init() */

void mpu6050ReadAccel(int16_t *X, int16_t *Y, int16_t *Z)
{
uint8_t ucTemp[8];
byte i;

  i = I2CReadRegister(imu_addr, 0x3b, ucTemp, 6);
  if (i == 6)
  {
    *X = (ucTemp[0] << 8) + ucTemp[1]; // reverse endian order
    *Y = (ucTemp[2] << 8) + ucTemp[3];
    *Z = (ucTemp[4] << 8) + ucTemp[5];
  }
} /* mpu6050ReadAccel() */

void setup() {
int i, j, x, y;
// initialize SSD1306 display
// 1Mhz is wishful thinking, but worst case, the I2C driver will settle for 400Khz
  oledInit(0x3c, OLED_128x64, 0, 0, -1, -1, 1000000);
  oledFill(0); // fill display with black
  mpu6050Init(0x68); // Initialize the accelerometer
  
  memset(img, 0, sizeof(img)); // Clear our copy of the image array
  for(i=0; i<N_GRAINS; i++) {  // For each sand grain...
    do {
      grain[i].x = random(WIDTH  * 256); // Assign random position within
      grain[i].y = random(HEIGHT * 256); // the 'grain' coordinate space
      // Check if corresponding pixel position is already occupied...
      for(j=0; (j<i) && (((grain[i].x / 256) != (grain[j].x / 256)) ||
                         ((grain[i].y / 256) != (grain[j].y / 256))); j++);
    } while(j < i); // Keep retrying until a clear spot is found
    x = grain[i].x / 256; y = grain[i].y / 256;
    img[((y / 8) * 128) + x] |= (1 << (y & 7)); // Mark it
    grain[i].vx = grain[i].vy = 0; // Initial velocity is zero
  }
} // setup
void loop() {
  int16_t i, x, y, z;
  uint32_t t;
  int32_t v2; // Velocity squared
  int16_t ax, ay, az;
  int        oldidx, newidx;
  uint8_t    oldbit, newbit;
  int        newx, newy;
  int        x1, y1, x2, y2;
  
  // Limit the animation frame rate to MAX_FPS.  Because the subsequent sand
  // calculations are non-deterministic (don't always take the same amount
  // of time, depending on their current states), this helps ensure that
  // things like gravity appear constant in the simulation.
  while(((t = micros()) - prevTime) < (1000000L / MAX_FPS));
  prevTime = t;
 
  // Display the current pixels (every other time through the loop)
  // Since we have velocities of less than 1 pixel, this allows for movement up to 2 pixels
  // per display update and still looks smooth since there are fractional velocities
  if ((iFrame & 1) == 0)
     oledDumpBuffer(img);
  iFrame++;

  // Read accelerometer...
  mpu6050ReadAccel(&x, &y, &z);
  ax = -x / 512;      // Transform accelerometer axes
  ay = -y / 512;      // to grain coordinate space
  az = abs(z) / 2048; // Random motion factor
  az = (az >= 3) ? 1 : 4 - az;      // Clip & invert
  ax -= az;                         // Subtract motion factor from X, Y
  ay -= az;
 
  // Apply 2D accelerometer vector to grain velocities...
  //
  // Theory of operation:
  // if the 2D vector of the new velocity is too big (sqrt is > 256), this means it might jump
  // over pixels. We want to limit the velocity to 1 pixel as a maximum.
  // To avoid using floating point math (sqrt + 2 multiplies + 2 divides)
  // Instead of normalizing the velocity to keep the same direction, we can trim the new
  // velocity to 5/8 of it's value. This is a reasonable approximation since the maximum
  // velocity impulse from the accelerometer is +/-64 (16384 / 256) and it gets added every frame
  //
  for(i=0; i<N_GRAINS; i++) {
    grain[i].vx += ax + random(5); // Add a little random impulse to each grain
    grain[i].vy += ay + random(5);
    v2 = (int32_t)(grain[i].vx*grain[i].vx) + (int32_t)(grain[i].vy*grain[i].vy);
    if (v2 > 65536) // too big, trim it
    {
      grain[i].vx = (grain[i].vx * 5)/8; // quick and dirty way to avoid doing a 'real' divide
      grain[i].vy = (grain[i].vy * 5)/8;
    }
  } // for i
 
  // Update the position of each grain, one at a time, checking for
  // collisions and having them react.  This really seems like it shouldn't
  // work, as only one grain is considered at a time while the rest are
  // regarded as stationary.  Yet this naive algorithm, taking many not-
  // technically-quite-correct steps, and repeated quickly enough,
  // visually integrates into something that somewhat resembles physics.
  // (I'd initially tried implementing this as a bunch of concurrent and
  // "realistic" elastic collisions among circular grains, but the
  // calculations and volument of code quickly got out of hand for both
  // the tiny 8-bit AVR microcontroller and my tiny dinosaur brain.)
  //
  // (x,y) to bytes mapping:
  // The SSD1306 has 8 rows of 128 bytes with the LSB of each byte at the top
  // In other words, bytes are oriented vertically with bit 0 as the top pixel
  // Part of my optimizations were writing the pixels into memory the same way they'll be
  // written to the display. This means calculating an offset and bit to test/set each pixel
  //
  for(i=0; i<N_GRAINS; i++) {
    newx = grain[i].x + grain[i].vx; // New position in grain space
    newy = grain[i].y + grain[i].vy;
    if(newx > MAX_X) {               // If grain would go out of bounds
      newx         = MAX_X;          // keep it inside, and
      grain[i].vx /= -2;             // give a slight bounce off the wall
    } else if(newx < 0) {
      newx         = 0;
      grain[i].vx /= -2;
    }
    if(newy > MAX_Y) {
      newy         = MAX_Y;
      grain[i].vy /= -2;
    } else if(newy < 0) {
      newy         = 0;
      grain[i].vy /= -2;
    }

    x1 = grain[i].x / 256; y1 = grain[i].y / 256;
    oldidx = ((y1/8) * WIDTH + x1); // Prior pixel #
    oldbit = 1 << (y1 & 7);
    x2 = newx / 256; y2 = newy / 256;
    newidx = ((y2/8) * WIDTH + x2); // New pixel #
    newbit = 1 << (y2 & 7);
    if((oldidx != newidx || oldbit != newbit) && // If grain is moving to a new pixel...
        (img[newidx] & newbit) != 0) {       // but if that pixel is already occupied...
        // Try skidding along just one axis of motion if possible (start w/faster axis)
        if(abs(grain[i].vx) > abs(grain[i].vy)) { // X axis is faster
          x1 = newx / 256; y1 = grain[i].y / 256;
          newidx = ((y1 / 8) * WIDTH + x1);
          newbit = 1 << (y1 & 7);
          if((img[newidx] & newbit) == 0) { // That pixel's free!  Take it!  But...
            newy         = grain[i].y; // Cancel Y motion
            grain[i].vy /= -2;         // and bounce Y velocity
          } else { // X pixel is taken, so try Y...
            y1 = newy / 256; x1 = grain[i].x / 256;
            newidx = ((y1 / 8) * WIDTH + x1);
            newbit = 1 << (y1 & 7);
            if((img[newidx] & newbit) == 0) { // Pixel is free, take it, but first...
              newx         = grain[i].x; // Cancel X motion
              grain[i].vx /= -2;         // and bounce X velocity
            } else { // Both spots are occupied
              newx         = grain[i].x; // Cancel X & Y motion
              newy         = grain[i].y;
              grain[i].vx /= -2;         // Bounce X & Y velocity
              grain[i].vy /= -2;
            }
          }
        } else { // Y axis is faster
          y1 = newy / 256; x1 = grain[i].x / 256;
          newidx = ((y1 / 8) * WIDTH + x1);
          newbit = 1 << (y1 & 7);
          if((img[newidx] & newbit) == 0) { // Pixel's free!  Take it!  But...
            newx         = grain[i].x; // Cancel X motion
            grain[i].vx /= -2;         // and bounce X velocity
          } else { // Y pixel is taken, so try X...
            y1 = grain[i].y / 256; x1 = newx / 256;
            newidx = ((y1 / 8) * WIDTH + x1);
            newbit = 1 << (y1 & 7);
            if((img[newidx] & newbit) == 0) { // Pixel is free, take it, but first...
              newy         = grain[i].y; // Cancel Y motion
              grain[i].vy /= -2;         // and bounce Y velocity
            } else { // Both spots are occupied
              newx         = grain[i].x; // Cancel X & Y motion
              newy         = grain[i].y;
              grain[i].vx /= -2;         // Bounce X & Y velocity
              grain[i].vy /= -2;
            }
          }
        }
    }
    if (grain[i].x != newx || grain[i].y != newy)
    {
      y1 = newy / 256; x1 = newx / 256;
      newidx = ((y1 / 8) * WIDTH + x1);
      newbit = 1 << (y1 & 7);
      grain[i].x  = newx; // Update grain position
      grain[i].y  = newy;
      img[oldidx] &= ~oldbit; // erase old pixel
      img[newidx] |= newbit;  // Set new pixel
    }
  }
 } // loop
