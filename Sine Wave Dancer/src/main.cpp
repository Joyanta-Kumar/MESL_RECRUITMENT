#include <Arduino.h>
#include <Servo.h>

#define SERVO_ANGLE_MIN 0
#define SERVO_ANGLE_MAX 180
#define DELAY 20
#define STEP 1.0
#define SERVO_PIN 9

double getY(double x);
double mapRange(double value, double fromMin, double fromMax, double toMin, double toMax);

double x = 0.0;
Servo dancer;


void setup()
{
  dancer.attach(SERVO_PIN);
}


void loop()
{
  double y = getY(x);
  int servoAngle = (int) ceil(mapRange(y, 0, 1, SERVO_ANGLE_MIN, SERVO_ANGLE_MAX));
  dancer.write(servoAngle);
  _delay_ms(DELAY);
  x += STEP;
}


double getY(double x)
{
  return pow(sin(x * PI / 180.0), 2);
}

double mapRange(double value, double fromMin, double fromMax, double toMin, double toMax)
{
  return (value - fromMin) / (fromMax - fromMin) * (toMax - toMin) + toMin;
}
