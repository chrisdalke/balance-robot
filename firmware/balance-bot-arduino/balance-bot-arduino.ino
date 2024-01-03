#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

/*
Balance Bot Arduino firmware

- Reads pitch angle from BNO055 IMU board
- Runs a PID control loop to optimize pitch error
- Outputs control values to L298N motor driver
*/

#define TICK_RATE 20
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

#define PIN_MOTOR_1_EN 6
#define PIN_MOTOR_1_DIR1 5
#define PIN_MOTOR_1_DIR2 4

#define PIN_MOTOR_2_EN 9
#define PIN_MOTOR_2_DIR1 8
#define PIN_MOTOR_2_DIR2 7

float pitch = 0.0f;
float targetPitch = 0.0f;
float rawOut = 0.0f;

float err = 0.0f;
float errLast = 0.0f;
float errD = 0.0f;
float errI = 0.0f;
float errISaturation = 255.0f;
float kp = 15.0f;
float ki = 30.0f;
float kd = 0.5f;

void setup() {
  // Init motor driver
  pinMode(PIN_MOTOR_1_EN, OUTPUT);
  pinMode(PIN_MOTOR_1_DIR1, OUTPUT);
  pinMode(PIN_MOTOR_1_DIR2, OUTPUT);
  pinMode(PIN_MOTOR_2_EN, OUTPUT);
  pinMode(PIN_MOTOR_2_DIR1, OUTPUT);
  pinMode(PIN_MOTOR_2_DIR2, OUTPUT);


  // Init serial debugging
  Serial.begin(115200);
//  while (!Serial) {
//    delay(10);
//  }

  // Init IMU
  bno.begin();
  delay(100);
  bno.setExtCrystalUse(true);
}

void loop() {
  // Get latest pose from BNO055
  sensors_event_t event;
  bno.getEvent(&event);

  // Debug: Output pitch (Z)
  pitch = event.orientation.z;
  Serial.print("pitch:");
  Serial.print(pitch, 4);

  // Iterate PID loop
  float dt = 0.02f;
  errLast = err;
  err = targetPitch - pitch;
  errD = (err - errLast) / dt;
  errI = constrain(errI + (err * dt), -errISaturation, errISaturation);
  rawOut = (err * kp) + (errD * kd) + (errI * ki);

  if (abs(err) < 1.0f) {
     errI = 0.0f;
  }

  // Write motor outputs
  bool dir = true;
  if (rawOut > 0.0f) {
    dir = false;
  }
  int pwm = constrain(0 + (int)(abs(rawOut)), 0, 255);
  digitalWrite(PIN_MOTOR_1_DIR1, dir);
  digitalWrite(PIN_MOTOR_1_DIR2, !dir);
  digitalWrite(PIN_MOTOR_2_DIR1, dir);
  digitalWrite(PIN_MOTOR_2_DIR2, !dir);
  analogWrite(PIN_MOTOR_1_EN, pwm);
  analogWrite(PIN_MOTOR_2_EN, pwm);
  Serial.print(",out:");
  Serial.print(rawOut);
  Serial.print(",pwm:");
  Serial.print(pwm);
  Serial.print(",dir:");
  Serial.print(dir);
  Serial.print(",err:");
  Serial.print(err);
  Serial.print(",errD:");
  Serial.print(errD);
  Serial.print(",errI:");
  Serial.print(errI);

  Serial.println("");
  delay(TICK_RATE);
}
