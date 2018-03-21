// AU<->FG Falcon climate control software
// Version 0.5.2- 4/2/18

#include <mcp_can.h>
#include <SPI.h>
#include <EEPROM.h>

// CAN0 INT and CS definitions.
#define CAN0_INT 2                              // Set INT to pin 2 - interrupt from can chip
MCP_CAN CAN0(10);                               // Set CS to pin 10
// pins 11-13 also used by the MCP

// Define pin states
// Analogue
#define EVAP_PIN 0
#define AMBIENT_PIN 1

// Digital
#define INLET_PIN 0
// also known as green pin in notes
#define WATER_VALVE_PIN 1
// ground to close valve
#define BLUE_PIN 3
// send air to face
#define RED_PIN 4
// to air to cold only face, bi level
#define BROWN_PIN 5
// send air to floor
#define YELLOW_PIN 6
// send air to screen and floor (demist?)

#define SERIESRESISTOR 10000
// the resistors in series with the thermistors to determine the actual resistance of the thermistors


// Define variables
float evapTemp; // A/C evaporator core temperature - may need this finer accuracy as is used below a temp to turn on/off.
int evapRaw; // raw voltage reading between 0 and 1023
int ambientTemp; // Outside temp - only need low resolution, ICC only displays int regardless
int ambientRaw; // raw voltage reading
int cabinTemp; // temperature from ICC CAN.
bool waterValveMode = FALSE; // whether water valve is closed (FALSE) or open (TRUE) 

unsigned int selectedTemp = 44; // Selected temp - must be between 18-30 (35 - 61, 35 is low, 61 high) with 0.5 increase - below 18 is LOW; above 30 is HIGH (real temp is selectedTemp / 2)
unsigned int climateMode = 0; // Auto (1) / Semi (2) / Off (0) - keep state as three different numbers (can't use boolean)
unsigned int selectedBlower = 0; // reports back selected blower fan speed in semi auto/manual mode; 10 fan speeds - 0-10, 0 being off
bool airInletMode = FALSE; // Air inlet mode - fresh (0 - FALSE) / recirc(1 - TRUE) 
unsigned int airOutletMode = 0; // Air outlet mode - face/floor/screen, demist
bool acOn = TRUE; // this is used only in the program, used to determine if A/C on or not. used to calculate the state of the below 3 A/C booleans.
bool acDisplayIcon; // Whether or not to display A/C icon??
bool acOnDisplayIcon; // not sure about the A/C icon deal here.. but by default A/C is enabled
bool acOffDisplayIcon; // by default A/C off so keep off
bool personDisplayIcon = TRUE; // something transmitted apparently, might as well transmit the person??? lol what
float blowerFanVoltage = 0.0; // Blower fan motor operating speed voltage, used in auto and manual mode

// Other variables
unsigned long prevTX = 0; // time elapsed since last transmission of state data
const unsigned int txInt = 50; // transmission interval for data kept in the state; HIM transmits every 50ms

unsigned long prevSave = 0; // time since last save
const unsigned int saveInt = 5000; // every 5 seconds go to save

// CAN RX Variables
long unsigned int rxId;
unsigned char len;
unsigned char rxBuf[8];

// HIM can prefix is 0x353
#define canID 0x353

// CAN Messages
#define ICC_NO_BUTTON "\x00\x00\x00\x80\x00\x00\x00\x00"
#define ICC_FRONT_DEMIST "\x02\x00\x00\x80\x00\x00\x00\x00"
#define ICC_INLET_BUTTON "\x40\x00\x00\x80\x00\x00\x00\x00"
#define ICC_AC_BUTTON "\x80\x00\x00\x80\x00\x00\x00\x00"
#define ICC_FAN_DEC "\x00\x04\x00\x80\x00\x00\x00\x00"
#define ICC_FAN_INC "\x00\x08\x00\x80\x00\x00\x00\x00"
#define ICC_OFF_BUTTON "\x00\x10\x00\x80\x00\x00\x00\x00"
#define ICC_AUTO_BUTTON "\x00\x20\x00\x80\x00\x00\x00\x00"
#define ICC_OUTLET_BUTTON "\x00\x80\x00\x80\x00\x00\x00\x00"
#define ICC_TEMP_DEC "\x00\x00\x40\x80\x00\x00\x00\x00"
#define ICC_TEMP_INC "\x00\x00\x80\x80\x00\x00\x00\x00"

void setup() {
  // Initialize MCP2515 running at 8MHz with a baudrate of 500kb/s
  // TODO: add masks and filters to only recieve specific PCM data.
  CAN0.begin(MCP_STDEXT, CAN_500KBPS, MCP_8MHZ);
  pinMode(CAN0_INT, INPUT);                       // Setting pin for interrupt input

  pinMode(INLET_PIN, OUTPUT);
  pinMode(WATER_VALVE_PIN, OUTPUT);
  pinMode(BLUE_PIN, OUTPUT);
  pinMode(RED_PIN, OUTPUT);
  pinMode(BROWN_PIN, OUTPUT);
  pinMode(YELLOW_PIN, OUTPUT);

  // only want to receive messages from ICC climate buttons (0x307) and PCM (0x)
  CAN0.init_Mask(0,0,0x07FF0000);                // Init first mask... - compares the entire ID.
  CAN0.init_Filt(0,0,0x03070000);                // Init first filter...
  CAN0.init_Filt(1,0,0x03130000);                // Init second filter, same as first as that is how the MCP works. might use this for ICC internal temp too.
  
  CAN0.init_Mask(1,0,0x07FF0000);                // Init second mask... 
  CAN0.init_Filt(2,0,0x01030000);                // Init third filter...
  CAN0.init_Filt(3,0,0x01040000);                // Init fouth filter...
  CAN0.init_Filt(4,0,0x01060000);                // Init fifth filter...
  CAN0.init_Filt(5,0,0x01070000);                // Init sixth filter...

  CAN0.setMode(MCP_NORMAL);                      // set mode to normal to be able to send messages

  // need to then retrieve state from beforehand from EEPROM
  selectedTemp = EEPROM.read(0);
  climateMode = EEPROM.read(1);
  airInletMode = EEPROM.read(2);
  airOutletMode = EEPROM.read(3);
  if (climateMode == 2){selectedBlower = EEPROM.read(4);} // no need to use blower speed if climate mode is not semi auto
}


void loop() {
  // check for interrupt from CAN chip (to then receive the data)
  if(!digitalRead(CAN0_INT)){
    CAN0.readMsgBuf(&rxId, &len, rxBuf);              // Read data: len = data length, buf = data byte(s)
    // Decide what the data is and what to do with it
    switch(rxId){
      case 0x307: // ICC message, sent every 500ms unless button pressed
        if (memcmp(rxBuf, ICC_NO_BUTTON, len) == 0){break;}
        
        if (memcmp(rxBuf, ICC_FRONT_DEMIST, len) == 0){
          // Demist
          climateMode = 2; // semi automatic
          airOutletMode = 5; // or whatever I want it to be
          changeAirOutlet();
          break;
        }
        
        if (memcmp(rxBuf, ICC_INLET_BUTTON, len) == 0){
          // change the intake mode variable then change the solenoids to suit, open fresh ground recirc
          airInletMode = !airInletMode; // we aren't changing to a specific mode, rather just changing to opposite
          climateMode = 2; // semi automatic
          changeAirInlet();
          break;
        }
        
        if (memcmp(rxBuf, ICC_AC_BUTTON, len) == 0){
          // switch A/C state. default is on.
          climateMode = 2; // semi automatic, as automatic automatically selects A/C on and off
          if (acOn == TRUE){
            acOn = FALSE;
          }
          else {
            acOn = TRUE;
          }
          // currently not running any functions as I guess A/C on off is decided by the PCM as part of the HIM broadcast.
          break;
        }

        // Fan changes are done when pressed
        if (memcmp(rxBuf, ICC_FAN_DEC, len) == 0){
          // unless at 0 (off), decrease selectedBlower by 1
          climateMode = 2;
          if (selectedBlower == 0){break;} // if the blower is already off do nothing and ignore rest
          else {
            selectedBlower--;
            changeBlowerMotor();
          }
          break;
        }

        if (memcmp(rxBuf, ICC_FAN_INC, len) == 0){
          // unless at 10 (max), increase selectedBlower by 1
          climateMode = 2;
          if (selectedBlower == 10){break;}
          else {
            selectedBlower++;
            changeBlowerMotor();
          }
          break;
        }

        if (memcmp(rxBuf, ICC_OFF_BUTTON, len) == 0){
          climateMode = 0;
          airInletMode = FALSE; // set inlet variable to fresh
          changeAirOutlet(); // actually change solenoids
          selectedBlower = 0;
          acOn = 1;
          break; // no need to process any more ifs 
        }

        if (memcmp(rxBuf, ICC_AUTO_BUTTON, len) == 0){
          climateMode = 1;
          selectedTemp = 44; // 22c is default
          // the rest of the stuff will be processed automatically later on in the loop
          break; // no need to process any more ifs 
        }

        // Outlet changes in semi auto only dealt with at time of button press.
        if (memcmp(rxBuf, ICC_OUTLET_BUTTON, len) == 0){
          // change to semi automatic if pressed, as not used in auto
          climateMode = 2;
          // Cycle through outlet modes
          if (airOutletMode != 4){ // TODO: actually decide on numbers here. go to real car and record things
            airOutletMode++;
          }
          else {
            // go to zero, starting one
            airOutletMode = 0;
          }
          // Now change solenoids etc.
          changeAirOutlet();
          break; // no need to process any more ifs 
        }

        // Temperature changes are captured later each loop for both semi and full auto.
        if (memcmp(rxBuf, ICC_TEMP_DEC, len) == 0){
          // unless at 35 (min), reduce selectedTemp by 1
          if (selectedTemp == 35){break;}
          else {selectedTemp--;break;}
        }

        if (memcmp(rxBuf, ICC_TEMP_INC, len) == 0){
          // if climate mode off, turn on
          // unless at 61 (max), increase selectedTemp by 1
          if (selectedTemp == 61){break;}
          else {selectedTemp++;break;}
        }
        break;
      case 0x313:
        // ICC Internal temperature input; recieved every 250ms
        cabinTemp = rxBuf[0]; // therefore to calculate a temperature times by two and add 100; therefore 22c = 144, 21.5 = 143; to get real temp cabinActualTemp = (cabinTemp - 100) / 2;
        break;
    }
  }

  // Retrieve information from sensors
  evapRaw = analogRead(EVAP_PIN);
  // now need to convert the input value into a resistance
  evapRaw = 1023 / evapRaw - 1;
  evapRaw = SERIESRESISTOR / evapRaw;
  evapTemp = tempCalc(evapRaw, 0);

  ambientRaw = analogRead(AMBIENT_PIN);
  // now need to convert to resistance to pass
  ambientRaw = 1023 / ambientRaw - 1;
  ambientRaw = SERIESRESISTOR / ambientRaw;
  ambientTemp = tempCalc(ambientRaw, 1);
  
  // Manual/"semi auto" climate control blend door control
  // For manual climate control, would need to associate 'temperatures' with positions of the blend door. there are 26 'temp positions', from cold at 35 to hot at 61.
  // inlet controls & blower are is done in the CAN input part
  if (climateMode == 2){
    // move door dependant on selectedTemp. 
    // also need to enable / disable water tap depending on whether we are heating or cooling.
    switch (selectedTemp){
      case 35:
        waterValveMode = FALSE;
        changeWaterValve();
        moveBlendDoor(0);
      case 61: // max temp
        waterValveMode = TRUE;
        changeWaterValve();
        moveBlendDoor(255);
    }
  }

  // Calculations and changes for automatic climate go here.
  if (climateMode == 1){
    // TODO: implement a PID algorithm to move motor dependant on everything.
    cabinActualTemp = (cabinTemp - 100) / 2;
    // Will change blend door (use moveBlendDoor), air inlet (use changeAirInlet), blower speed (as infinitely variable implement locally), air outlet (use changeAirOutlet), acOn status
  }

  // Save data to EEPROM when necessary - every 5 seconds
  if(millis() - prevSave >= saveInt){  
    prevSave = millis();
    EEPROM.update(0, selectedTemp);
    EEPROM.update(1, climateMode);
    EEPROM.update(2, airInletMode);
    EEPROM.update(3, airOutletMode);
    if (climateMode == 2){EEPROM.update(4, selectedBlower);} // only save this if the car is running in semi mode
  }

  // Send CAN data
  if(millis() - prevTX >= txInt){                    // currently sending at 50ms interval defined by txInt
  prevTX = millis();
  byte data[8];
  // each byte stores an 8 bit number from 0-255
  // byte 6 always contains 129, byte 5 always contains zero, byte 3 always contains 34
  data[6] = 129;
  data[5] = 0;
  data[3] = 34;

  // tx selectedTemp
  // tx ambientTemp
  // tx evapTemp

  // tx airInletMode
  // tx airOutletMode
  // tx personDisplayIcon

  // AC related booleans to transmit:
  // may need to only actually turn on the icons when e.g a/c turned off acOffDisplayIcon would be one, but if system off (climateMode == 0), then don't need to
  // display acOnDisplayIcon or acDisplayIcon (as they are just assumed etc)
  if (acOn == FALSE){
    acDisplayIcon = FALSE;
    acOnDisplayIcon = FALSE;
    acOffDisplayIcon = TRUE;
  }
  else{
    acDisplayIcon = TRUE;
    acOnDisplayIcon = TRUE;
    acOffDisplayIcon = FALSE;
  }
  // tx acDisplayIcon
  // tx acOnDisplayIcon
  // tx acOffDisplayIcon
  }

  if (climateMode == 0){
    data[0] = 0;
    data[1] = 0;
    data[2] = 171;
    data[4] = 0; // data[4] is the temperature setting where data[4] = selectedTemp
    data[7] = ;
  }

  byte sndStat = CAN0.sendMsgBuf(canID, 8, data); //send the packet over CAN using the HIM's CANID of 0x353
}

// Code to convert resistance to actual temperature
float tempCalc(int x_in, int mode) {
  float temp, a, b, c; 
  temp = 0.0;
  // coefficients
  if(mode==0){// evap temp coefficients
    a = 2.8316638268780913E+01; 
    b = -2.6511866112720156E+01; 
    c = -6.5157025827603281E+00;
  }
  else{ // ambient temp coefficients; based currently on 20-36c; x is resistance, y is temperature
    a = 3.8290808431370401E+01; 
    b = -2.6103766308667307E+01; 
    c = 5.6745487456546986E+00;    
  }
  temp = a + b*log(x_in) + c*pow(log(x_in), 3.0);
  return temp; 
} 

int moveBlendDoor(int pos){
  // take input and move blend door to the position requested.
  // only actually move the door if the position has changed, don't need to burn out the motor moving it like crazy
}

int changeAirOutlet(){
  switch (airOutletMode){
    case 0:
      // eyy
      break;
    case 1:
      // eyy
      break;
    case 2:
      // eyy
      break;
    case 3:
      // eyy
      break;
    case 4:
      // eyy
      break;
    case 5:
      // demist
      // Change solenoids to demist, turn on A/C, 
      // seems that for screen, the solenoids are all off?
      digitalWrite(BLUE_PIN, LOW);
      digitalWrite(RED_PIN, LOW);
      digitalWrite(BROWN_PIN, LOW);
      digitalWrite(YELLOW_PIN, LOW);
      break;
  }
  return airOutletMode;
}

int changeAirInlet(){
  if (airInletMode == FALSE){
    // Fresh air
    digitalWrite(INLET_PIN, LOW);
  }
  else{
    // Recirculate
    digitalWrite(INLET_PIN, HIGH);
  }
  return airInletMode;
}

int changeBlowerMotor(){
  // Input effectively is global variable selectedBlower; this code only used for the semi auto mode.

  // Change the output of whatever pin is controlling fan speed controller to change to the requested fan speed.
  // Also set blowerFanVoltage to respective level.
  return 0;
}

int changeWaterValve(){
  // the water valve is closed when grounded
  if (waterValveMode == FALSE){
    // water valve closed
    digitalWrite(WATER_VALVE_PIN, HIGH);
  }
  else{
    // water valve open
    digitalWrite(WATER_VALVE_PIN, LOW);
  }
  return waterValveMode;
}
