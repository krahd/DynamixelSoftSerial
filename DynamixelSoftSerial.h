/*
  ax12.h - arbotiX Library for AX-12 Servos
  Copyright (c) 2008,2009 Michael E. Ferguson.  All right reserved.
 
  Modified on 15/11/09 by Pablo Gindel
  version 2.0 - 10/02/10
  version 2.1 - 27/06/10
  version 2.2 - 19/10/10
  version 2.31 - 2/3/11
 
  Modified on September 4, 2015 by Diego García del Río (garci66@gmail.com) and Tomás Laurenzo (tomas@laurenzo.net)
  Let's say is version 2.4
 
  We added the possibility of using SoftwareSerialHalfDuplex at a lower bitrate and therefore controlling the motors with an arbitrary pin. This frees the UART and allows to have serial connectivity over USB with and Arduino UNO and no extra hardware.
 
   However, given the speed of the Arduino UNO, one first need to use the pins 0 and 1 (shorted) as before to set the lower speed and the none is free to use any GPIO.
 
 
  We use the following library for SoftwareSerialWithHalfDuplex: https://github.com/krahd/SoftwareSerialWithHalfDuplex forked from https://github.com/nickstedman/SoftwareSerialWithHalfDuplex
 
 
  Please see the readme file for more information.

*/

// todo: defines para los errores
// todo: create keywords.txt

#ifndef _AX12_H
#define _AX12_H

#include <SoftwareSerialWithHalfDuplex.h>

#define AX12_MAX_SERVOS          18
#define AX12_BUFFER_SIZE         32

#define AX12_DEFAULT_ID         1       // ax12's come with id 1 by default
#define AX12_DEFAULT_BAUD       57600   // arbitrary. I don't pick the default 1Mbps as it woulnd't work over SoftwareSerial

/** EEPROM AREA **/
#define AX12_MODEL_NUMBER             0
#define AX12_VERSION                  2
#define AX12_ID                       3
#define AX12_BAUD_RATE                4  //TODO add AX12_ prefix to all the defines
#define RETURN_DELAY_TIME        5
#define CW_ANGLE_LIMIT           6
#define CCW_ANGLE_LIMIT          8
#define LIMIT_TEMPERATURE        11
#define DOWN_LIMIT_VOLTAGE       12
#define UP_LIMIT_VOLTAGE         13
#define MAX_TORQUE               14
#define STATUS_RETURN_LEVEL      16
#define ALARM_LED                17
#define ALARM_SHUTDOWN           18
#define DOWN_CALIBRATION         20
#define UP_CALIBRATION           22

/** RAM AREA **/
#define TORQUE_ENABLE            24
#define LED                      25
#define CW_COMPLIANCE_MARGIN     26
#define CCW_COMPLIANCE_MARGIN    27
#define CW_COMPLIANCE_SLOPE      28
#define CCW_COMPLIANCE_SLOPE     29
#define GOAL_POSITION            30
#define MOVING_SPEED             32
#define TORQUE_LIMIT             34
#define PRESENT_POSITION         36
#define PRESENT_SPEED            38
#define PRESENT_LOAD             40
#define PRESENT_VOLTAGE          42
#define PRESENT_TEMPERATURE      43
#define REGISTERED_INSTRUCTION   44
#define MOVING                   46
#define LOCK                     47
#define PUNCH                    48

/** Status Return Levels **/
#define RETURN_NONE              0
#define RETURN_READ              1
#define RETURN_ALL               2

/** Instruction Set **/
#define AX_PING                  1
#define READ_DATA                2
#define WRITE_DATA               3
#define REG_WRITE                4
#define ACTION                   5
#define RESET                    6
#define SYNC_WRITE               131

/** Special IDs **/
#define AX12_BROADCAST_ID             254

// #define USE_UART // uncomment for using the UART with pins 0 and 1 joined. 

typedef unsigned char byte;

typedef struct {
    int error;
    byte *data;
} AX12data;

typedef struct {
    int error;
    int value;
} AX12info;

class AX12 {
    
  public:

    AX12 (long baud, uint8_t commPin, byte motor_id, bool inv);
    AX12 (byte motor_id, bool inv);
    AX12 (long baud, uint8_t commPin, byte motor_id);
    AX12 (byte motor_id);
    AX12 ();

    
    bool inverse;
    byte SRL;  // status return level

    // The buffer can't be private, as it needs to be visible from the ISR
    static volatile byte ax_rx_buffer[AX12_BUFFER_SIZE]; // receiving buffer
    static volatile byte ax_rx_Pointer; // making these volatile keeps the compiler from optimizing loops of available()

    void init (long baud, uint8_t commPin);
    
    void setId (byte newId);
    byte getId ();
    
    byte autoDetect (byte* list_motors, byte num_motors);
    
    void syncWrite (byte start, byte length, byte targetlength, byte* targets, byte** valuess);
    void syncInfo (byte registr, byte targetlength, byte* targets, int* values);
    void setMultiPosVel (byte targetlength, byte* targets, int* posvalues, int* velvalues);
    
    int ping ();
    int reset ();
    
    AX12data readData (byte start, byte length);
    int writeData (byte start, byte length, byte* values, bool isReg = false);
    
    int action ();
    
    AX12info readInfo (byte registr);
    int writeInfo (byte registr, int value, bool isReg = false);
    
    void setEndlessTurnMode (bool endless);
    void endlessTurn (int vel);
    
    int presentPSL (int* PSL);
    
    void setSRL (byte _srl);
    
    byte changeID (byte newID);
    int setPosVel (int pos, int vel);
    
    void setBroadcast(bool broadcast);
    
  private:

    byte id;
    byte oldId; // used when turning broadcast off
    byte writeByte (byte data);
    
    void sendPacket (byte _id, byte datalength, byte instruction, byte* data);
    byte readPacket ();
    
    AX12data returnData (byte _srl);
    
    void processValue (byte registr, int* value);
    
    //SoftwareSerialWithHalfDuplex *serialHandle;

    Stream *serialHandle;
    
    bool useSoftwareSerial;

};

// utils

bool sign2bin (int number);
char bin2sign (bool var);


// macros

#define setPos(pos) writeInfo (GOAL_POSITION, pos)
#define regPos(pos) writeInfo (GOAL_POSITION, pos, true)
#define getPos() readInfo (PRESENT_POSITION).value
#define setMultiPos(a, b, c) syncInfo (GOAL_POSITION, a, b, c)

#define setSpeed(vel) writeInfo (MOVING_SPEED, vel)
#define getSpeed() readInfo (PRESENT_SPEED).value
#define setMultiSpeed(a, b, c) syncInfo (MOVING_SPEED, a, b, c)

#define setTorqueOn() writeInfo (TORQUE_ENABLE, 1)
#define setTorqueOff() writeInfo (TORQUE_ENABLE, 0)
#define setTorque(torque) writeInfo (TORQUE_LIMIT, torque)
#define setMultiTorque(a, b, c) syncInfo (TORQUE_LIMIT, a, b, c)
#define getLoad() readInfo (PRESENT_LOAD).value


#endif
