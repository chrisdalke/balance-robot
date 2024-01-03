#include <PS4Controller.h>

void setup() {
  PS4.begin("40:c7:11:95:42:0b");
  Serial1.begin(9600);
  Serial.begin(9600);
}

char outX = 128;
char outY = 128;
void loop() {
  if (PS4.isConnected()) {
      //Serial.printf("%d,%d,%d,%d\n", PS4.LStickX(), PS4.LStickY(), PS4.RStickX(), PS4.RStickY());
      outY = constrain(PS4.LStickY() + 128, 0, 255);
      outX = constrain(PS4.RStickX() + 128, 0, 255);
  } else {
      outX = 128;//constrain((sin(millis() / 1000.0) * 128) + 128, 0, 255);
      outY = 128;
  }

  Serial1.print((int)outY);
  Serial.print((int)outY);
  Serial1.print(",");
  Serial.print(",");
  Serial1.println((int)outX);
  Serial.println((int)outX);
  delay(50);
}
