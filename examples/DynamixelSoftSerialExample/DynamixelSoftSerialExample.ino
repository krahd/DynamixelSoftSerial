#include <DynamixelSoftSerial.h>
#include <SoftwareSerialWithHalfDuplex.h> 
/* Although we will not use SoftwareSerialWithHalfDuplex directly, it has to be included here for DynamixelSoftSerial 
   to be able to include it. Another option is not to include it here and copy it to the same folder of DynamixelSoftSerial
*/

AX12 motorController;

void setup() {
  motorController = AX12();
  
}

void loop() {

}
