/*
 ax12.cpp - arbotiX Library for AX-12 Servos
 Copyright (c) 2008,2009 Michael E. Ferguson.  All right reserved.
 Modificada el 15/11/09 por Pablo Gindel.
 versión 2.0 - 10/02/10
 versión 2.1 - 27/06/10
 versión 2.2 - 19/10/10
 versión 2.3 - 30/12/10
 versión 2.31 - 2/3/11
 
 Modified on September 4, 2015 by Diego García del Río (garci66@gmail.com) and Tomás Laurenzo (tomas@laurenzo.net)
 Let's say is version 2.4
 
 */

// #includes order seems to be important. Also, check the examples for more #include notes

#include <DynamixelSoftSerial.h>
#include <string.h>
#include <arduino.h>


// some utils definition
int makeInt (byte *dir, byte reglength);
byte lengthRead (byte registr);
byte lengthWrite (byte registr);



/** Sends a character out the serial port */
byte AX12::writeByte (byte data) {
    //serialHandle->write(data);
    serialHandle->write(data);
    return data;
}



/** We have a one-way receive buffer, which is reset after each packet is receieved.
 A wrap-around buffer does not appear to be fast enough to catch all bytes at 1Mbps. */
volatile byte AX12::ax_rx_buffer[AX12_BUFFER_SIZE];
volatile byte AX12::ax_rx_Pointer;


/** initializes serial transmit at baud, 8-N-1 */
// the Ax12 internally uses Double Speed Operation (U2Xn=1). Some baudrates don't match if it's not used.

void AX12::init (long baud, uint8_t commPin) {
    
    serialHandle = new SoftwareSerialWithHalfDuplex(commPin, commPin, false, false);
    
        ((SoftwareSerialWithHalfDuplex *) (serialHandle))->begin(baud);
    //serialHandle->begin(baud);
    
}


void AX12::setId(byte newId) {
    id = newId;
}

byte AX12::getId() {
    return id;
}

/******************************************************************************
 * Initialization
 ******************************************************************************/

// Scanns all IDS 0..253 sending pings.
// returns a list with the answering IDs
byte AX12::autoDetect (byte* list_motors, byte num_motors) {
    
    byte counter = 0;
    byte *data;
    
    for (byte i = 0; i < 254; i++) {
        sendPacket (i, 0, AX_PING, data);
        byte index = readPacket ();
        byte id = ax_rx_buffer [index];
        byte int_error = ax_rx_buffer [index+1];
        
        if (int_error==0 && id==i) {
            list_motors[counter++] = i;
            if (counter == num_motors) {
                break;
            }
        }
    }
    return counter;
}

/** constructors */
AX12::AX12 (long baud, uint8_t commPin,  byte motor_id, bool inv) {
    id = motor_id;
    inverse = inv;
    SRL = RETURN_ALL;
    init (baud, commPin);
}

AX12::AX12 (byte motor_id, bool inv) {
    id = motor_id;
    inverse = inv;
    SRL = RETURN_ALL;
}

AX12::AX12 (long baud, uint8_t commPin, byte motor_id) {
    id = motor_id;
    inverse = false;
    SRL = RETURN_ALL;
    init (baud, commPin);
}

AX12::AX12 (byte motor_id) {
    id = motor_id;
    inverse = false;
    SRL = RETURN_ALL;
}

AX12::AX12 () {
    id = AX12_DEFAULT_ID; // by default AX12's are with ID = 1;
    inverse = false;
    SRL = RETURN_ALL;
}




/******************************************************************************
 * Packet Level
 ******************************************************************************/

/** send instruction packet */
void AX12::sendPacket (byte _id, byte datalength, byte instruction, byte* data) {
    
    byte checksum = 0;
    writeByte (0xFF);
    writeByte (0xFF);
    checksum += writeByte (_id);
    checksum += writeByte (datalength + 2);
    checksum += writeByte (instruction);
    for (byte f=0; f<datalength; f++) {     // data = parámetros
        checksum += writeByte (data[f]);
    }
    // checksum =
    writeByte (~checksum);
    ((SoftwareSerialWithHalfDuplex *) (serialHandle))->listen();
    //serialHandle->listen();
}

/* read status packet
 returns the position in the buffer from where you can read the following:
 position [0] = status_id
 position [1] = internal error (0 = OK)
 position [2] = status_error
 position [3,4,...] = status_data        */

byte AX12::readPacket () {
    unsigned long ulCounter;
    byte timeout, error, status_length, checksum, offset, bcount;
    
    offset = 0; timeout = 0; bcount = 0;
    
    //Wipe the input buffer
    while (bcount < AX12_BUFFER_SIZE)
        ax_rx_buffer[bcount++]=0;
    
    bcount=0;
    // we wait for all the data
    
    while (bcount < 13) {       // 13 is max packet length
        ulCounter = 0;
        while (!serialHandle->available())
            if (ulCounter++ > 2000L) { // TODO: this is horrible, we should NOT busy wait
                timeout = 1;
                break;
            }
        if (timeout) break;
        ax_rx_buffer[bcount++] = serialHandle->read();
    }
    
    // The buffer should have bcount bytes (including 2 0xff headers)
    error = 0;
    
    while (ax_rx_buffer[offset] == 0xFF and bcount >0) {
        error++;
        offset++;
        bcount--;
    }
    
    if (bcount >0) {
        //Serial.println("Enter decode loop");
        if (error == 2) error = 0;                              // prueba de cabecera
        // First two bytes MUST have been 0xFF. If we read more or less, its an issue!
        // offset = first byte (without header)
        // bcount = length of the read message (without header)
        
        status_length = 2 + ax_rx_buffer[offset+1];            // largo del mensaje decodificado
        //DEBUG
        //Serial.println(status_length);
        if (bcount != status_length) error+=2;                 // prueba de coherencia de data
        checksum = 0;                                          // cálculo de checksum
        for (byte f=0; f<status_length; f++)
            checksum += ax_rx_buffer[offset+f];
        if (checksum != 255) error+=4;                          // prueba de checksum
        ax_rx_buffer[offset+1] = error;
        return offset;
    }
    else return -1;
}

/******************************************************************************
 * Instruction Level
 ******************************************************************************/

void AX12::setBroadcast(bool broadcast){
    if (broadcast) {
        oldId = id;
        id = AX12_BROADCAST_ID;
        inverse = false;
        SRL = RETURN_NONE;
    } else {
        id = oldId;
        inverse = false;
        SRL = RETURN_ALL;
    }
}





/** ping */
int AX12::ping () {
    byte* data;
    sendPacket (id, 0, AX_PING, data);
    return returnData (RETURN_NONE).error;
}

/** reset */
int AX12::reset () {
    byte* data;
    sendPacket (id, 0, RESET, data);
    return returnData (RETURN_ALL).error;
}

/** action */
int AX12::action () {
    byte *data;
    sendPacket (id, 0, ACTION, data);
    return returnData (RETURN_ALL).error;
}

/** read data */
AX12data AX12::readData (byte start, byte length) {
    byte data [2];
    data [0] = start; data [1] = length;
    sendPacket (id, 2, READ_DATA, data);
    return returnData (RETURN_READ);
}

/** write data + reg write */
// seteando a "true" el parámetro adicional (isReg) se transforma en un reg write
int AX12::writeData (byte start, byte length, byte* values, bool isReg) {
    byte data [length+1];
    data [0] = start;
    memcpy (&data[1], values, length);
    if (isReg) {
        sendPacket (id, length+1, REG_WRITE, data);
    } else {
        sendPacket (id, length+1, WRITE_DATA, data);
    }
    int error = returnData (RETURN_ALL).error;
    if (start < 23) delay (5);       // Wait 5 seconds if writing to eprom
    // (las operaciones en la EEPROM no suelen ser real-time)
    return error;
}

/** sync write */
void AX12::syncWrite (byte start, byte length, byte targetlength, byte* targets, byte** valuess) {
    byte rowlength = length + 1;
    byte superlength = rowlength*targetlength + 2;
    byte data [superlength];
    data [0] = start;
    data [1] = length;
    byte index = 2;
    for (byte f=0; f<targetlength; f++) {
        data [index++] = targets[f];                 // pongo el ID
        memcpy (&data[index], valuess[f], length);   // copio los valores
        index += length;
    }
    sendPacket (AX12_BROADCAST_ID, superlength, SYNC_WRITE, data);
}


/******************************************************************************
 * Register Level
 ******************************************************************************/

/** "intelligent" read data */
AX12info AX12::readInfo (byte registr) {
    byte reglength = lengthRead (registr);
    AX12info returninfo;
    returninfo.error = -2;
    
    if (reglength == 0) {
        return returninfo;
    }
    
    AX12data returndata = readData (registr, reglength);
    returninfo.error = returndata.error;
    returninfo.value = makeInt (returndata.data, reglength);
    processValue (registr, &returninfo.value);
    return returninfo;
}

/** "intelligent" write data + reg write */
// seteando a "true" el parámetro adicional se transforma en un reg write
int AX12::writeInfo (byte registr, int value, bool isReg) {
    byte reglength = lengthWrite (registr);
    if (reglength==0) {return -2;}
    processValue (registr, &value);
    byte values [reglength];
    values [0] = lowByte(value);
    if (reglength > 1) {values[1] = highByte(value);}
    return writeData (registr, reglength, values, isReg);
}

/** "intelligent" sync write */
void AX12::syncInfo (byte registr, byte targetlength, byte* targets, int* values) {
    byte reglength = lengthWrite (registr);
    if (reglength==0) {return;}
    byte valuess [targetlength][reglength];
    byte * pointers [targetlength];
    for (byte f=0; f<targetlength; f++) {
        valuess [f][0] = lowByte(values[f]);
        if (reglength > 1) {valuess[f][1] = highByte(values[f]);}
        pointers[f] = &valuess[f][0];
    }
    //nota: la sync write no respeta la propiedad "inverse"
    syncWrite (registr, reglength, targetlength, targets, pointers);
}


/******************************************************************************
 * Macro Level
 ******************************************************************************/

void AX12::setEndlessTurnMode (bool endless) {           // prende o apaga el modo "endless turn"
    writeInfo (CW_ANGLE_LIMIT, 0);
    if (endless) {
        writeInfo (CCW_ANGLE_LIMIT, 0);
    } else {
        writeInfo (CCW_ANGLE_LIMIT, 1023);
    }
}

void AX12::endlessTurn (int vel) {                    // setea la vel, en el modo "endless turn"
    bool direccion = sign2bin (vel);
    writeInfo (MOVING_SPEED, abs(vel)|((direccion^inverse)<<10));
}

int AX12::presentPSL (int* PSL) {                                // lee position, speed & load de una sola vez
    AX12data data = readData (PRESENT_POSITION, 6);
    for (byte f=0; f<3; f++) {
        PSL[f] = makeInt (&data.data[2*f], 2);
        processValue (PRESENT_POSITION + 2*f, &PSL[f]);
    }
    return data.error;
}

// nota: si no coincide el SRL declarado con el del motor, los mensajes de respuesta son malinterpretados
void AX12::setSRL (byte _srl) {
    SRL = _srl;
    writeInfo (STATUS_RETURN_LEVEL, SRL);
}

byte AX12::changeID (byte newID) {
    if (newID > 253) {return id;}
    writeInfo (AX12_ID, newID);
    id = newID;
    return id;
}

int AX12::setPosVel (int pos, int vel) {
    processValue (GOAL_POSITION, &pos);
    byte values [4];
    values [0] = lowByte(pos);
    values[1] = highByte(pos);
    values [2] = lowByte(vel);
    values[3] = highByte(vel);
    return writeData (GOAL_POSITION, 4, values);
}

void AX12::setMultiPosVel (byte targetlength, byte* targets, int* posvalues, int* velvalues) {
    byte valuess [targetlength][4];
    byte * pointers [targetlength];
    for (byte f=0; f<targetlength; f++) {
        valuess [f][0] = lowByte(posvalues[f]);
        valuess[f][1] = highByte(posvalues[f]);
        valuess [f][2] = lowByte(velvalues[f]);
        valuess[f][3] = highByte(velvalues[f]);
        pointers[f] = &valuess[f][0];
    }
    //nota: la sync write no respeta la propiedad "inverse"
    syncWrite (GOAL_POSITION, 4, targetlength, targets, pointers);
}


/******************************************************************************
 * Misc.
 ******************************************************************************/

// solución para que bin2sign y sign2bin queden fuera de acá: distribuir junto con "util.cpp"

bool sign2bin (int number) {         // number > 0 --> true; number <= 0 --> false
    return (number > 0);
}

char bin2sign (bool var) {           // var = 0 --> sign = -1; var = 1 --> sign = 1
    return 2*var - 1;
}

int makeInt (byte *dir, byte reglength) {          // transforma 2 bytes en un int (según la lógica AX12)
    if (reglength > 1) {
        return (dir[1] << 8) | dir[0];
    } else {
        return dir[0];
    }
}

byte lengthRead (byte registr) {
    byte reglength = 0;
    switch (registr) {
        case AX12_VERSION: case AX12_ID: case AX12_BAUD_RATE: case RETURN_DELAY_TIME:
        case LIMIT_TEMPERATURE: case DOWN_LIMIT_VOLTAGE: case UP_LIMIT_VOLTAGE:
        case STATUS_RETURN_LEVEL: case ALARM_LED: case ALARM_SHUTDOWN: case 19: case TORQUE_ENABLE: case LED:
        case CW_COMPLIANCE_MARGIN: case CCW_COMPLIANCE_MARGIN: case CW_COMPLIANCE_SLOPE: case CCW_COMPLIANCE_SLOPE:
        case PRESENT_VOLTAGE: case PRESENT_TEMPERATURE: case REGISTERED_INSTRUCTION: case MOVING: case LOCK: reglength = 1; break;
            
        case AX12_MODEL_NUMBER: case CW_ANGLE_LIMIT: case CCW_ANGLE_LIMIT:
        case MAX_TORQUE: case DOWN_CALIBRATION: case UP_CALIBRATION:
        case GOAL_POSITION: case MOVING_SPEED: case TORQUE_LIMIT:
        case PRESENT_POSITION: case PRESENT_SPEED: case PRESENT_LOAD: case PUNCH: reglength = 2; break;
    }
    return reglength;
}

byte lengthWrite (byte registr) {
    byte reglength = 0;
    switch (registr) {
        case AX12_ID: case AX12_BAUD_RATE: case RETURN_DELAY_TIME:
        case LIMIT_TEMPERATURE: case DOWN_LIMIT_VOLTAGE: case UP_LIMIT_VOLTAGE:
        case STATUS_RETURN_LEVEL: case ALARM_LED: case ALARM_SHUTDOWN: case 19:
        case TORQUE_ENABLE: case LED: case CW_COMPLIANCE_MARGIN: case CCW_COMPLIANCE_MARGIN:
        case CW_COMPLIANCE_SLOPE: case CCW_COMPLIANCE_SLOPE: case REGISTERED_INSTRUCTION: case LOCK: reglength = 1; break;
            
        case CW_ANGLE_LIMIT: case CCW_ANGLE_LIMIT:
        case MAX_TORQUE: case GOAL_POSITION:
        case MOVING_SPEED: case TORQUE_LIMIT: case PUNCH: reglength = 2; break;
    }
    return reglength;
}

AX12data AX12::returnData (byte _srl) {
    AX12data returndata;
    if (SRL >= _srl) {
        byte index = readPacket ();  //We're not checking if index is not -1 (problems in the readPacket library)
        byte status_id = ax_rx_buffer [index];
        byte int_error = ax_rx_buffer [index+1];
        byte status_error = ax_rx_buffer [index+2];
        returndata.error = (int_error<<7) | status_error | ((status_id != id)<<10);       // genera el mensaje de error, combinación de error interno con error ax12
        returndata.data = (byte*) &(ax_rx_buffer [index+3]);
    } else {
        returndata.error = -1;
    }
    return returndata;
}

// Atención: modificada a lo bestia para los MX28
void AX12::processValue (byte registr, int* value) {                           // procesa el valor para la salida segun la propiedad "inverse"
    switch (registr) {
        case PRESENT_POSITION: case GOAL_POSITION:
            if (inverse) {*value = 4095 - *value;}       // era 1023
            break;
        case PRESENT_SPEED: case PRESENT_LOAD:
            *value = ((*value)&0x03FF) * bin2sign(((*value)>0x03FF)^inverse); 
            break;
    }
}
