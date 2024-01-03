#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <AccelStepper.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

AccelStepper stepper1 = AccelStepper(1, 9, 8);
AccelStepper stepper2 = AccelStepper(1, 10, 16);

unsigned long last_time2 = 0;
unsigned long last_time = 0;
unsigned long last_pid_time = 0;
unsigned long last_pos_pid_time = 0;

float pitch = 0.0f;
int microstep_mode = 2;
float microstep_scalar = 1.0;
float stepping_denominator = 1.0;

float speedout = 0.0;
float speedout1 = 0.0;
float speedout2 = 0.0;

float target_pos = 0.0f;
float current_pos = 0.0f;
float control_out = 0.0f;
float err_pos = 0.0f;
float errLast_pos = 0.0f;
float errD_pos = 0.0f;
float errI_pos = 0.0f;
float errISaturation_pos = 255.0f;
//float kp_pos = 0.0f;
//float ki_pos = 0.1f;
//float kd_pos = 0.0f; 
float kp_pos = 0.0f;
float ki_pos = 0.0f;
float kd_pos = 0.0f; 

const int bufferSize = 64;
char inputBuffer[bufferSize];
byte bufferIndex = 0;

int steeringRateRaw = 0;
float serialTargetPitch = 0.0f;
float targetPitch = 0.0f;
float rawOut = 0.0f;
float pitchRad = 0.0f;
float pitchRate = 0.0f;
float err = 0.0f;
float errLast = 0.0f;
float errD = 0.0f;
float errI = 0.0f;
float errISaturation = 255.0f;
float kp = 18.0f;
float ki = 0.0f;
float kd = 0.4f;
//float kp = 0.0f;
//float ki = 0.0f;
//float kd = 100.0f; 

float dt_sec = 0.0;
float rpm = 0.0;
float wheelPos = 0.0;
float centerOfMassPos = 0.0;
bool isDisabled = false;

void setup() {
  // Init serial debugging
  Serial.begin(115200);

  // init esp comms
  Serial1.begin(9600);
  
  // Init IMU
  bno.begin();
  delay(100);
  bno.setExtCrystalUse(true);

  // init steppers
  stepper1.setMaxSpeed(4000);
  stepper2.setMaxSpeed(4000);

  // microstepping
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(14, OUTPUT);
  pinMode(15, OUTPUT);
}

void loop() {
  // Initialize clocks when loop starts
  if (last_time2 == 0) {
    last_time2 = millis();
    last_pos_pid_time = millis();
    last_pid_time = millis();
  }
  
  // Get latest pose from BNO055
  sensors_event_t orentationData, angVelocityData;
  bno.getEvent(&orentationData, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  pitch = orentationData.orientation.z;
  pitchRate = angVelocityData.gyro.z;
  if (abs(pitch) > 20.0 && !isDisabled) {
    isDisabled = true;
  }
  if (abs(pitch) < 2.0 && isDisabled) {
    isDisabled = false;
  }

  // Iterate position PID loop at 10Hz
  if (millis() - last_pos_pid_time > 100.0) {
    float dt = (millis() - last_pos_pid_time) / 1000.0f;
    errLast_pos = err_pos;
    err_pos = (target_pos - current_pos);
    err_pos *= err_pos;
    errD_pos = pitchRate;
    errI_pos = constrain(errI_pos + (err_pos * dt), -errISaturation_pos, errISaturation_pos);
    control_out = (err_pos * kp_pos) + (errD_pos * kd_pos) + (errI_pos * ki_pos);
    if (abs(err_pos) < 5.0f) {
       errI_pos = 0.0f;
    }
    last_pos_pid_time = millis();
    
    targetPitch = 0.0 - control_out;
    if (targetPitch < -10.0) {
      targetPitch = -10.0;
    }
    if (targetPitch > 10.0) {
      targetPitch = 10.0;
    }
  }
  targetPitch = serialTargetPitch;

  if (isDisabled) {
     errI_pos = 0.0f;
  }
  
  // Iterate pitch PID loop at 100Hz
  if (millis() - last_pid_time > 10.0) {
    float dt = (millis() - last_pid_time) / 1000.0f;
    errLast = err;
    err = (targetPitch - pitch);
    errD = (err - errLast);
    errI = constrain(errI + (err * dt), -errISaturation, errISaturation);
    rawOut = (err * kp) + (errD * kd) + (errI * ki);
  
    if (abs(err) < 5.0f) {
       errI = 0.0f;
    }
    last_pid_time = millis();
  }
  bool dir = true;
  if (rawOut > 0.0f) {
    dir = false;
  }

  // TEST
  //rawOut = targetPitch;

  // Compute which microstep mode to use
  float absSpeed = abs(rawOut);
  if (absSpeed <= 32) {
    microstep_mode = 16;
    microstep_scalar = 53.33333333;
  } else if (absSpeed <= 64) {
    microstep_mode = 8;
    microstep_scalar = 26.66666667 * 0.9;
  } else if (absSpeed <= 128) {
    microstep_mode = 4;
    microstep_scalar = 13.33333333 * 0.8;
  } else if (absSpeed <= 256) {
    microstep_mode = 2;
    microstep_scalar = 6.666666667 * 0.6;
  } else {
    microstep_mode = 1;
    microstep_scalar = 3.333333333 * 0.5;
  }

  // Scale + clamp stepper speeds
  speedout = (rawOut) * microstep_scalar;
  if (speedout > 4000.0) {
    speedout = 4000.0;
  }
  if (speedout < -4000.0) {
    speedout = -4000.0;
  }

  // Periodically compute wheel position
  if (millis() - last_time2 >= 10.0) {
    wheelPos += (speedout / microstep_scalar) * (0.0163362818);
    pitchRad = ((pitch * -0.0174533) + (1.57079632679));
    centerOfMassPos = wheelPos + (180.0 * cos(pitchRad));
    last_time2 = millis();
  }
  
  if (isDisabled) {
    speedout = 0.0;
  }

  // compute additional turning speed
  if (abs(steeringRateRaw - 128) > 40) {
    speedout1 = speedout + ((steeringRateRaw - 128) * 10);
    speedout2 = speedout + ((steeringRateRaw - 128) * -10);
  } else {
    speedout1 = speedout;
    speedout2 = speedout;
  }
  
  // Set stepper speed
  stepper1.setSpeed(speedout1);
  stepper2.setSpeed(-speedout2);


  // set dynamic stepping
  if (microstep_mode == 32) {
    // 1/32 step
    digitalWrite(5, HIGH);
    digitalWrite(6, LOW);
    digitalWrite(7, HIGH);
  } else if (microstep_mode == 16) {
    // 1/16 step
    digitalWrite(5, LOW);
    digitalWrite(6, LOW);
    digitalWrite(7, HIGH);
  } else if (microstep_mode == 8) {
    // 1/8 step
    digitalWrite(5, HIGH);
    digitalWrite(6, HIGH);
    digitalWrite(7, LOW);
  } else if (microstep_mode == 4) {
    // quarter step
    digitalWrite(5, LOW);
    digitalWrite(6, HIGH);
    digitalWrite(7, LOW);
  } else if (microstep_mode == 2) {
    // half step
    digitalWrite(5, HIGH);
    digitalWrite(6, LOW); 
    digitalWrite(7, LOW);
  } else {
    // full step
    digitalWrite(5, LOW);
    digitalWrite(6, LOW);
    digitalWrite(7, LOW);
  }

  // Run steppers and check delta wheel position
  long lastPosition = stepper1.currentPosition();
  stepper1.runSpeed();
  stepper2.runSpeed();
  long deltaPosition = stepper1.currentPosition() - lastPosition;
  current_pos += deltaPosition / microstep_scalar;
  
  // Read bytes from ESP

  // Read bytes from ESP
  while (Serial1.available() > 0) {
    char inByte = Serial1.read();
    if (inByte != '\n') {
      if (bufferIndex < bufferSize - 1) {
        inputBuffer[bufferIndex++] = inByte;
      }
    } else {
      inputBuffer[bufferIndex] = '\0';
      
      // parse controls as ints
      char *token = strtok(inputBuffer, ",");
      if (token != NULL) {
        serialTargetPitch = (atoi(inputBuffer) - 128) / 10.0;
        token = strtok(NULL, ",");
        if (token != NULL) {
          steeringRateRaw = atoi(token);
        }
      }
      
      bufferIndex = 0;
    }
  }
  
  if (millis() - last_time > 10) {
      Serial.println(steeringRateRaw);
    last_time = millis();
  }
}
