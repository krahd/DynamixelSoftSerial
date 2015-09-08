#include <DynamixelSoftSerial.h>
#include <SoftwareSerialWithHalfDuplex.h>

/* Although we will not use SoftwareSerialWithHalfDuplex directly, it has to be included here for DynamixelSoftSerial
   to be able to include it. Another option is not to include it here and copy it to the same folder of DynamixelSoftSerial
*/

AX12 motor;
int mode;
int spinSpeed;
int id = 1;

void setup() {
  motor = AX12(57600, 8, 2);

  Serial.begin(9600); // Serial port to communicate with the computer

  mode = 0;
}

void loop() {

  switch (mode)  {
    case 0:
      motor.setTorqueOn();
      motor.setEndlessTurnMode (false);

      motor.setSpeed (random (400, 800));
      motor.setPos (random (200, 800));

      Serial.print ("Position: ");
      Serial.println (motor.getPos(), DEC);
      Serial.print ("Speed: ");
      Serial.println (motor.getSpeed(), DEC);
      Serial.print ("Load: ");
      Serial.println(motor.getLoad());
      delay (500);
      break;

    case 1:

      motor.setSpeed(spinSpeed);
      break;

    case 2:
      motor.setSpeed(spinSpeed);
      break;
  }


  if (Serial.available() > 0) {
    byte in = Serial.read();
    switch (in) {
      case '0':
        mode = 0;
        break;

      case '1':
        mode = 1;
        motor.setTorqueOn();
        motor.setEndlessTurnMode (true);
        break;

      case '2':
        motor.setTorqueOff();
        spinSpeed = 0;
        mode = 2;
        break;

      case 'w':
        spinSpeed++;
        break;

      case 'W':
        spinSpeed += 10;
        break;

      case 's':
        spinSpeed--;
        break;

      case 'S':
        spinSpeed -= 10;
        break;

      case 'p':
        Serial.print ("mode: ");
        Serial.println(mode);
        Serial.print ("spinSpeed: ");
        Serial.println(spinSpeed);
        Serial.print ("Position: ");
        Serial.println (motor.getPos(), DEC);
        Serial.print ("Speed: ");
        Serial.println (motor.getSpeed(), DEC);
        Serial.print ("Load: ");
        Serial.println(motor.getLoad());
    }
  }

  delay (10);
}
