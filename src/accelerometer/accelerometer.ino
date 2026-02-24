#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <MD_MAX72xx.h>
#include <Wire.h>

int findPoint(int x, int y);
bool isInside(int x, int y);

// hardware setup & const
Adafruit_MPU6050 mpu;

#define CLK_PIN 13
#define DATA_PIN 11
#define CS_PIN 10

#define MAX_DEVICES 1

MD_MAX72XX mx = MD_MAX72XX(MD_MAX72XX::PAROLA_HW, CS_PIN, MAX_DEVICES);

#define MAX_POINTS 64
#define MIN_X 0
#define MAX_X 7
#define MIN_Y 0
#define MAX_Y 7

struct Vec2 {
  int x;
  int y;
};

struct Point {
  Vec2 pos;
  Vec2 speed;
  bool active;
};

Point points[64];

char daysOfTheWeek[7][12] = {"Sunday",   "Monday", "Tuesday", "Wednesday",
                             "Thursday", "Friday", "Saturday"};

void setup(void) {
  Serial.begin(115200);

  while (!mpu.begin(0x69)) {
    Serial.println("MPU6050 not connected!");
    delay(1000);
  }
  Serial.println("MPU6050 ready!");

  while (!mx.begin()) {
    Serial.println("MPU6050 not connected!");
    delay(1000);
  }

  mx.control(MD_MAX72XX::INTENSITY, MAX_INTENSITY / 2);
  mx.clear();

  for (int i = 0; i < 25; i++) {
    int x, y;
    do {
      x = random(8);
      y = random(8);
    } while (findPoint(x, y) != -1);

    mx.setPoint(x, y, true);
    points[i].pos.x = x;
    points[i].pos.y = y;
    points[i].active = true;
  }
}

sensors_event_t event;

unsigned long prevTime = 0;
const unsigned long interval = 100;

Vec2 gravity2D = {0, 0};

void loop() {
  unsigned long currentTime = millis();

  mpu.getAccelerometerSensor()->getEvent(&event);

  if (event.acceleration.x > 3) {
    gravity2D.x = 1;
    gravity2D.y = 0;
  }

  if (event.acceleration.x < -3) {
    gravity2D.x = -1;
    gravity2D.y = 0;
  }

  if (event.acceleration.y > 3) {
    gravity2D.x = 0;
    gravity2D.y = -1;
  }
  if (event.acceleration.y < -3) {
    gravity2D.x = 0;
    gravity2D.y = 1;
  }

  if (currentTime - prevTime >= interval) {
    prevTime = currentTime;

    // Serial.print("[");
    // Serial.print(millis());
    // Serial.println("] X: ");
    // Serial.println(event.acceleration.x);

    Serial.println();

    for (int i = MAX_POINTS - 1; i >= 0; i--) {

      if (!points[i].active)
        continue;

      Point &p = points[i];
      int nx = p.pos.x + gravity2D.x;
      int ny = p.pos.y + gravity2D.y;

      if (findPoint(nx, ny) == -1 && isInside(nx, ny)) {
        mx.setPoint(p.pos.y, p.pos.x, false);
        p.pos.x = nx;
        p.pos.y = ny;
        continue;
      }

      Vec2 perp;
      perp.x = -gravity2D.y;
      perp.y = gravity2D.x;

      int side = random(2);

      Vec2 slide;

      if (side == 0) {
        slide.x = gravity2D.x + perp.x;
        slide.y = gravity2D.y + perp.y;
      } else {
        slide.x = gravity2D.x - perp.x;
        slide.y = gravity2D.y - perp.y;
      }

      int sx = p.pos.x + slide.x;
      int sy = p.pos.y + slide.y;

      if (findPoint(sx, sy) == -1 && isInside(sx, sy)) {
        mx.setPoint(p.pos.y, p.pos.x, false);

        p.pos.x = sx;
        p.pos.y = sy;
        continue;
      }
    }

    for (int i = 0; i < MAX_POINTS; i++) {
      if (points[i].active) {
        mx.setPoint(points[i].pos.y, points[i].pos.x, true);
      }
    }

    mx.update();
  }
}

bool isInside(int x, int y) {
  if (x < MIN_X || x > MAX_X || y < MIN_Y || y > MAX_Y)
    return false;

  return true;
}

// find given pos of point or return -1
int findPoint(int x, int y) {

  for (int i = 0; i < MAX_POINTS; i++) {
    if (points[i].active && points[i].pos.x == x && points[i].pos.y == y) {
      return i; // return pos
    }
  }
  return -1;
}