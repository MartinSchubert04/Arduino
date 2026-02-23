// Example sketch for the MPU-6050.
//
// Version 1, 9 August 2021, by Koepel with help from Wokwi community.
//
// Start the simulation and click on the MPU-6050 module to change the values.
//
// Note:
//   This example does not use a library with quaternations for the 3D
//   orientation. The values are directly read from the MPU-6050 and are used to
//   rotate a cube. It is just a fun example. The rotation of the cube is not
//   according to the physical values of the accelerometer and the gyro.
//
// Based on:
//   (1)
//     3D_Cube for Arduino OLED module by Colin Ord, 9/1/2015
//     A port of my (Colin Ord) original JustBasic Cube_3D demo to the Arduino
//     Uno using U8G library.
//     http://colinord.blogspot.com/2015/01/arduino-oled-module-with-3d-demo.html
//     (no known copyrights)
//
//   (2)
//     MPU-6050 Short Example Sketch
//     By Arduino User JohnChi
//     August 17, 2014
//     Public Domain
//     https://playground.arduino.cc/Main/MPU-6050/#short
//
//   (3)
//     The Adafruit GFX library with the SSD1306 driver.
//

#include <Adafruit_GFX.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>

Adafruit_SSD1306 display(128, 64); // 128 pixels width, 64 pixels height

#define LED_PIN 6
#define LED_COUNT 1
Adafruit_NeoPixel neoPixel(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

const int mpuAddress = 0x68; // I2C address of the MPU-6050

float xByGyro, yByGyro, zByGyro; // Global variables for the rotation by gyro

// Set the origin in the middle of the display
const int xOrigin = 64;
const int yOrigin = 32;

const float viewDistance =
    150.0; // higher for less perspective, lower for more.

// Vertices for a cube
// A cube has 8 corners and each coordinate has x,y,z values.
#define NUM_VERTICES 8
const int cube_vertex[NUM_VERTICES][3] = {
    {-20, -20, 20}, // x, y, z
    {20, -20, 20},  {20, 20, 20},  {-20, 20, 20}, {-20, -20, -20},
    {20, -20, -20}, {20, 20, -20}, {-20, 20, -20}};

// The wirefram is to display the lines on the OLED display
// It contains the corners of the shape in 2D coordinates
int wireframe[NUM_VERTICES][2];

void setup() {
  Serial.begin(115200);

  Wire.begin();

  // Initialize the OLED display and test if it is connected.
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;)
      ; // halt the sketch if error encountered
  }

  // Initialize the MPU-6050 and test if it is connected.
  Wire.beginTransmission(mpuAddress);
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0);    // set to zero (wakes up the MPU-6050)
  auto error = Wire.endTransmission();
  if (error != 0) {
    Serial.println(F("Error, MPU-6050 not found"));
    for (;;)
      ; // halt the sketch if error encountered
  }

  // Initialize the NeoPixel
  neoPixel.begin();
}

void loop() {
  Wire.beginTransmission(mpuAddress);
  Wire.write(0x3B);            // Starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false); // No stop condition for a repeated start

  // The MPU-6050 has the values as signed 16-bit integers.
  // There are 7 values in 14 registers.
  int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;

  Wire.requestFrom(mpuAddress, 14); // request a total of 14 bytes
  AcX = Wire.read() << 8 |
        Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  AcY = Wire.read() << 8 |
        Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ = Wire.read() << 8 |
        Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp =
      Wire.read() << 8 | Wire.read(); // 0x41 (TEMP_OUT_H)   & 0x42 (TEMP_OUT_L)
  GyX = Wire.read() << 8 |
        Wire.read(); // 0x43 (GYRO_XOUT_H)  & 0x44 (GYRO_XOUT_L)
  GyY = Wire.read() << 8 |
        Wire.read(); // 0x45 (GYRO_YOUT_H)  & 0x46 (GYRO_YOUT_L)
  GyZ = Wire.read() << 8 |
        Wire.read(); // 0x47 (GYRO_ZOUT_H)  & 0x48 (GYRO_ZOUT_L)

  // The acceleration is directly mapped into the angles.
  // That is rather artificial.
  // The combined gravity could be used for an angle, while ignoring the
  // strength.
  //
  // The gyro sets the rotation speed.
  // The angle created by the rotation speed is added to angle by the
  // accelerometer.
  //
  // The conversion from the sensor values to the rotation is just a value
  // that makes it look good on the display.

  float xByAccel = (float)AcX * 0.0001; // static angle by accelerometer
  float yByAccel = (float)AcY * 0.0001;
  float zByAccel = (float)AcZ * 0.0001;

  xByGyro += (float)GyX * 0.00001; // moving angle by gyro
  yByGyro += (float)GyY * 0.00001;
  zByGyro += (float)GyZ * 0.00001;

  float x = xByAccel + xByGyro; // combine both angles
  float y = yByAccel + yByGyro;
  float z = zByAccel + zByGyro;

  // Keep the radians in range (although the cos/sin functions accept every
  // value)
  if (x < 0.0)
    x += 2.0 * M_PI;
  else if (x > 2.0 * M_PI)
    x -= 2.0 * M_PI;
  if (y < 0.0)
    y += 2.0 * M_PI;
  else if (y > 2.0 * M_PI)
    y -= 2.0 * M_PI;
  if (z < 0.0)
    z += 2.0 * M_PI;
  else if (z > 2.0 * M_PI)
    z -= 2.0 * M_PI;

  // Draw 3D picture
  for (int i = 0; i < NUM_VERTICES; i++) {
    // Rotate Y
    float rotx = cube_vertex[i][2] * sin(y) + cube_vertex[i][0] * cos(y);
    float roty = cube_vertex[i][1];
    float rotz = cube_vertex[i][2] * cos(y) - cube_vertex[i][0] * sin(y);

    // Rotate X
    float rotxx = rotx;
    float rotyy = roty * cos(x) - rotz * sin(x);
    float rotzz = roty * sin(x) + rotz * cos(x);

    // Rotate Z
    float rotxxx = rotxx * cos(z) - rotyy * sin(z);
    float rotyyy = rotxx * sin(z) + rotyy * cos(z);
    float rotzzz = rotzz;

    // Add depth perspective
    rotxxx *= viewDistance / (viewDistance + rotzzz);
    rotyyy *= viewDistance / (viewDistance + rotzzz);

    // Bring to middle of screen
    rotxxx += (float)xOrigin;
    rotyyy += (float)yOrigin;

    // Store new vertices values for wireframe drawing
    wireframe[i][0] = (int)rotxxx;
    wireframe[i][1] = (int)rotyyy;
    wireframe[i][2] = (int)rotzzz;
  }
  draw_wireframe();

  // Set the color of the NeoPixel according to the temperature
  // Temperature by the MPU-6050 is -40 to 85.
  // According to the datasheet:
  //   Temperature in Celsius = (raw_value / 340) + 36.53
  // The Hue range for the NeoPixel is the full uint16_t range.
  float Celsius = ((float)Tmp / 340.00) + 36.53;
  float hue = (Celsius + 40.0) / 125.0 * 65535.0;
  uint32_t rgbcolor = neoPixel.ColorHSV((uint16_t)hue);
  neoPixel.setPixelColor(0, rgbcolor);
  neoPixel.show(); // update new values to NeoPixel
}

void draw_wireframe(void) {
  // Start with a empty buffer
  display.clearDisplay();

  // A cube has 8 points and 12 sides.
  // The wireframe contains the 8 points, and the 12 lines are drawn here.
  display.drawLine(wireframe[0][0], wireframe[0][1], wireframe[1][0],
                   wireframe[1][1], SSD1306_WHITE);
  display.drawLine(wireframe[1][0], wireframe[1][1], wireframe[2][0],
                   wireframe[2][1], SSD1306_WHITE);
  display.drawLine(wireframe[2][0], wireframe[2][1], wireframe[3][0],
                   wireframe[3][1], SSD1306_WHITE);
  display.drawLine(wireframe[3][0], wireframe[3][1], wireframe[0][0],
                   wireframe[0][1], SSD1306_WHITE);

  display.drawLine(wireframe[4][0], wireframe[4][1], wireframe[5][0],
                   wireframe[5][1], SSD1306_WHITE);
  display.drawLine(wireframe[5][0], wireframe[5][1], wireframe[6][0],
                   wireframe[6][1], SSD1306_WHITE);
  display.drawLine(wireframe[6][0], wireframe[6][1], wireframe[7][0],
                   wireframe[7][1], SSD1306_WHITE);
  display.drawLine(wireframe[7][0], wireframe[7][1], wireframe[4][0],
                   wireframe[4][1], SSD1306_WHITE);

  display.drawLine(wireframe[0][0], wireframe[0][1], wireframe[4][0],
                   wireframe[4][1], SSD1306_WHITE);
  display.drawLine(wireframe[1][0], wireframe[1][1], wireframe[5][0],
                   wireframe[5][1], SSD1306_WHITE);
  display.drawLine(wireframe[2][0], wireframe[2][1], wireframe[6][0],
                   wireframe[6][1], SSD1306_WHITE);
  display.drawLine(wireframe[3][0], wireframe[3][1], wireframe[7][0],
                   wireframe[7][1], SSD1306_WHITE);

  // Extra cross face on one side
  // display.drawLine( wireframe[1][0], wireframe[1][1], wireframe[3][0],
  // wireframe[3][1], SSD1306_WHITE); display.drawLine( wireframe[0][0],
  // wireframe[0][1], wireframe[2][0], wireframe[2][1], SSD1306_WHITE);

  // Write the new picture to the display
  display.display();
}