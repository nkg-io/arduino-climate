// AU<->FG Falcon climate control software
// Version 0.8 - 17/6/18
// Functions not working: Automatic Climate and Blend door control using L298N, and ensure correct calculations for external temperatures

#include <mcp_can.h>
#include <SPI.h>
#include <EEPROM.h>

// CAN0 INT and CS definitions.
#define CAN0_INT 49                              // Set INT to pin 49 - interrupt from can chip
MCP_CAN CAN0(53);                               // Set CS to pin 53 (pin 10 for arduino uno)
// pins 50-52 (mega) / 11-13 (uno) also used by the MCP

// Define pin states
// Analogue
#define EVAP_PIN A0 // voltage divider for evaporator thermistor
#define AMBIENT_PIN A1 // voltage divider for ambient temperature thermistor
#define MOTORLOC_PIN A2 // potentiometer input from blend door motor

// Digital

// When the solenoids are off; the specific thing is set to Atmosphere and does not get vacuum.

#define INLET_PIN 48
// also known as green pin
// Fresh is to atmosphere (low)
// Recirc is to vacuum (high)
// when on 'high temp' in fresh; this goes high and recircs; aka when high temp always recird

#define WATER_VALVE_PIN 47
// turquoise
// Water valve open when to atmosphere; which is when low (waterValveMode = TRUE)

#define BLUE_PIN 46
// send air to face
// To vacuum (high) when set to face

#define RED_PIN 45
// to air to cold only face, bi level
// To vacuum only when face and floor mode is on and 'full cold' is set

#define BROWN_PIN 44
// aka TAN
// send air to floor

#define YELLOW_PIN 43
// send air to screen and floor????? - there may be a secondary use to this but cannot find singular use of Brown / Yellow. both used together.

#define L298N_FSC_DIR1 42
// Used to set direction of the output from the L298N.

#define L298N_FSC_DIR2 41
// Used to set direction of the output from the L298N.

#define L298N_FSC_SPEED 2
// PWM control to the l298n for fan speed

#define L298N_BLEND_DIR1 40
// Used to set direction of the output from the L298N.

#define L298N_BLEND_DIR2 39
// Used to set direction of the output from the L298N.

#define L298N_BLEND_SPEED 3
// PWM control of the l298n for how quickly the blend door motor will move.
// Likely will be a constant speed anyway as no real need to move faster and slower.

#define SERIESRESISTOR 10000
// the resistors in series with the thermistors to determine the actual resistance of the thermistors

#define FALSE 0
#define TRUE 1


// Define variables
float evapTemp; // A/C evaporator core temperature - may need this finer accuracy as is used below a temp to turn on/off.
int evapRaw; // raw voltage reading between 0 and 1023
int ambientTemp; // Outside temp - only need low resolution, ICC only displays int regardless
int ambientRaw; // raw voltage reading
int cabinTemp; // temperature from ICC CAN.
bool waterValveMode = TRUE; // whether water valve is closed (FALSE) or open (TRUE) 
int sunload; // sunload reading sent from Body module; the actual address is currently unknown. will find out in future development

unsigned int selectedTemp = 44; // Selected temp - must be between 18-30 (35 - 61, 35 is low, 61 high) with 0.5 increase - below 18 is LOW; above 30 is HIGH (real temp is selectedTemp / 2)
unsigned int climateMode = 0; // Auto (1) / Semi (2) / Off (0) - keep state as three different numbers (can't use boolean)
unsigned int selectedBlower = 0; // reports back selected blower fan speed in semi auto/manual mode; 10 fan speeds - 0-10, 0 being off (1 = Low, 10 = High)
bool airInletMode = TRUE; // Air inlet mode - fresh (1 - true) / recirc(0 - false) 
unsigned int airOutletMode = 0; // Air outlet mode - face/floor/screen, demist
bool acOn = TRUE; // this is used only in the program, used to determine if A/C on or not. used to calculate the state of the below 3 A/C booleans.
bool acEngaged = FALSE; // this is because we want to send a message depending on whether AC clutch request is to be sent or not
//bool acDisplayIcon; // Whether or not to display A/C icon??
//bool acOnDisplayIcon; // not sure about the A/C icon deal here.. but by default A/C is enabled
//bool acOffDisplayIcon; // by default A/C off so keep off
//bool personDisplayIcon = TRUE; // something transmitted apparently. may actually be used to hide person in auto mode etc?
float blowerFanVoltage = 0.0; // Blower fan motor operating speed voltage

// Other variables
unsigned long prevTX = 0; // time elapsed since last transmission of state data
const unsigned int txInt = 50; // transmission interval for data kept in the state; HIM transmits every 50ms

unsigned long prevSave = 0; // time since last save
const unsigned int saveInt = 5000; // every 5 seconds go to save

int motorLoc = 0; // the location of the blend door motor
bool motorMoving = FALSE; // is the motor moving, used for logic in blend door
unsigned int prevTemp = 44; // previous temperature; used for logic relating to manual blend door
bool motorDirection = 0; // 0 is cooling (lowering values), 1 is heating (raising values). used for determining if we overshot

// CAN RX Variables
long unsigned int rxId;
unsigned char len;
unsigned char rxBuf[8];

// HIM can prefix is 0x353 // 851 dec
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
  CAN0.begin(MCP_STDEXT, CAN_500KBPS, MCP_8MHZ);
  pinMode(CAN0_INT, INPUT);                       // Setting pin for interrupt input

  pinMode(INLET_PIN, OUTPUT);
  pinMode(WATER_VALVE_PIN, OUTPUT);
  pinMode(BLUE_PIN, OUTPUT);
  pinMode(RED_PIN, OUTPUT);
  pinMode(BROWN_PIN, OUTPUT);
  pinMode(YELLOW_PIN, OUTPUT);

  //pinMode(FSC_PIN, OUTPUT);
  // Set up L298N pins all as output.
  pinMode(L298N_FSC_DIR1, OUTPUT);
  pinMode(L298N_FSC_DIR2, OUTPUT);
  pinMode(L298N_FSC_SPEED, OUTPUT);
  pinMode(L298N_BLEND_DIR1, OUTPUT);
  pinMode(L298N_BLEND_DIR2, OUTPUT);
  pinMode(L298N_BLEND_SPEED, OUTPUT);
  
  // Don't need to set pinMode on an analog pin if using analogRead()

  // only want to receive messages from ICC climate buttons (0x307) and PCM (0x)
  CAN0.init_Mask(0,0,0x07FF0000);                // Init first mask... - compares the entire ID.
  CAN0.init_Filt(0,0,0x03070000);                // Init first filter - ICC buttons
  CAN0.init_Filt(1,0,0x03130000);                // Init second filter - ICC internal temp
  
  CAN0.init_Mask(1,0,0x07FF0000);                // Init second mask... 
  CAN0.init_Filt(2,0,0x03070000);                // Init third filter...
  CAN0.init_Filt(3,0,0x03070000);                // Init fouth filter...
  CAN0.init_Filt(4,0,0x03070000);                // Init fifth filter...
  CAN0.init_Filt(5,0,0x03070000);                // Init sixth filter...

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
      case 0x307: // ICC message (0x307 / 775), sent every 500ms unless button pressed
        if (memcmp(rxBuf, ICC_NO_BUTTON, len) == 0){break;}

        if (memcmp(rxBuf, ICC_FRONT_DEMIST, len) == 0){
          // Demist
          climateMode = 2; // semi automatic
          // A/C is enabled when demist on
          acOn = TRUE;
          changeAirOutlet(4); // screen only (demist)
          break;
        }
          
        if (memcmp(rxBuf, ICC_INLET_BUTTON, len) == 0){
          // change the intake mode variable then change the solenoids to suit, open fresh ground recirc
          if (climateMode == 1){climateMode = 2;}; //if full auto change to semi auto as we can have recirc etc on Off

          // apparently if we are in off mode; but have fan on the system will switch to Semi auto mode.
          if (climateMode == 0 && selectedBlower != 0){climateMode = 2;}

          // another note; if you hit recirc when off; the person icon disappears and just becomes recirc; we are defaulting to face only
          if (climateMode == 0){
            airOutletMode = 0;
          }

          if (climateMode == 2){
            if (airOutletMode == 4){airOutletMode = 0;}
          }

          // Actually change the air inlet to the opposite
          if (airInletMode == FALSE){changeAirInlet(TRUE);}
          else {changeAirInlet(FALSE);}
          break;
        }
          
        if (memcmp(rxBuf, ICC_AC_BUTTON, len) == 0){
          // switch A/C state. default is on.
          if (climateMode == 0){climateMode = 1;}
          else {
            climateMode = 2; // semi automatic, as automatic automatically selects A/C on and off
            if (airInletMode == FALSE){changeAirInlet(TRUE);} // can not have recirc and A/C off apparently
            acOn = !acOn; //invert boolean
          }
          break;
        }

        // Fan changes are made when buttons pressed
        if (memcmp(rxBuf, ICC_FAN_DEC, len) == 0){
          // if off; ensure in fresh air mode and do nothing
          // this decreases selectedBlower by 1 if applicable
          // if auto, change to semi auto and dec etc
          if (climateMode == 1){climateMode = 2;}
          if (climateMode == 0 && airInletMode == FALSE){
            changeAirInlet(TRUE);
            break;
          }
          if (selectedBlower == 0 || selectedBlower == 1){break;} // if the blower is already off do nothing and ignore rest. does not turn off blower if at 1 either.
          else {
            selectedBlower--;
            changeBlowerMotor();
          }
          break;
        }

        if (memcmp(rxBuf, ICC_FAN_INC, len) == 0){
          // unless at 10 (max), increase selectedBlower by 1
          if (climateMode == 1){climateMode = 2;}
          if (climateMode == 0 && airInletMode == FALSE){
            changeAirInlet(TRUE);
            break;
          }
          if (selectedBlower == 10){break;}
          else {
            selectedBlower++;
            changeBlowerMotor();
          }
          break;
        }

        if (memcmp(rxBuf, ICC_OFF_BUTTON, len) == 0){
          climateMode = 0; 
          changeAirOutlet(0); // default is face 
          changeAirInlet(TRUE); // set inlet variable to fresh
          selectedBlower = 0;
          acOn = TRUE; // allow a/c again? TODO: we don't actaully want A/C clutch running though
          acEngaged = FALSE; // this will ensure the A/C clutch is turned off
          break; // no need to process any more ifs 
        }

        if (memcmp(rxBuf, ICC_AUTO_BUTTON, len) == 0){
          climateMode = 1;
          acOn = TRUE; // allow A/C
          selectedTemp = 44; // 22c is default
          changeAirInlet(TRUE); // auto runs fresh unless max cooling
          break; // no need to process any more ifs 
        }

        // Outlet changes in semi auto only dealt with at time of button press.
        if (memcmp(rxBuf, ICC_OUTLET_BUTTON, len) == 0){
          // change to semi automatic if pressed
          climateMode = 2;
          // Cycle through outlet modes
          if (airOutletMode <3 && airOutletMode >=0){
            changeAirOutlet(++airOutletMode);
          }
          else {
            // go to zero
            changeAirOutlet(0);
          }
          break;
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
        // 0x313 = 787 dec
        // ICC Internal temperature input; recieved every 250ms
        cabinTemp = rxBuf[0] - 100; // to calculate this form by two and add 100; eg 22c = 144, 21.5 = 143; to in reverse real temp cabinActualTemp = (cabinTemp) / 2; can sends like selectedTemp but +100
        break;
    };
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
  // inlet controls & blower are is done in the CAN input part
  if (climateMode == 2){
    // move door dependant on selectedTemp. 
    // TODO: if airOutletMode == 4 (demist); set fan to 3 (recommended in the manual); if (selectedTemp == 61){changeAirInlet(FALSE);}
    // need to implement acEngaged logic; if (acOn == FALSE) {acEngaged = FALSE;} else if (airOutletMode == 4){acEngaged = TRUE; // demist uses a/c} etc

    // If the temperature has changed:
    if (prevTemp != selectedTemp){
      // calculate direction for blend door motor to move in
      if(cabinTemp < selectedTemp){
        // if cabin temp is less than selected temp we need to heat
        motorDirection = 1; // heating
        analogWrite(L298N_BLEND_DIR1,LOW); analogWrite(L298N_BLEND_DIR2,HIGH);
        // also change water valve to assist - TODO: might need some sort of variance so the solenoid is not going crazy on and off when at temperature
        changeWaterValve(TRUE);
      }
      else {
        // we are cooling
        motorDirection = 0; // cooling
        analogWrite(L298N_BLEND_DIR1,HIGH); analogWrite(L298N_BLEND_DIR2,LOW);
        // also change water valve off
        changeWaterValve(FALSE);
      }

      // Override: Maximum cold always has the water valve closed and inlet to recirc
      if (selectedTemp == 35){changeWaterValve(FALSE); changeAirInlet(FALSE);}

      // start motor moving
      moveBlendDoor();
      // set the previous temp to selected temp as the temp has not changed since last check
      prevTemp = selectedTemp;
    }

    if (motorMoving == TRUE){
      // check location - TODO: maybe only do this every 50ms?
      motorLoc = analogRead(MOTORLOC_PIN);
      // If the motor is over the required location, stop the motor
      if (((motorLoc >= map(selectedTemp, 35, 61, closed_location, open_location)) && (motorDirection == 1)) || ((motorLoc <= map(selectedTemp, 35, 61, closed_location, open_location)) && (motorDirection == 0))){
        stopBlendDoor();
      }

      // An overshoot may occur, if we overshoot when heating we will have opened too far so the value will be larger than 2
      if ((motorDirection == 1) && motorLoc > (map(selectedTemp, 35, 61, closed_location, open_location)+2)){
        // cause the motor to run in the opposite direction for a slight time. do not want to bounce however; only need to be within the range
      }
      else if ((motorDirection == 0) && motorLoc < (map(selectedTemp, 35, 61, closed_location, open_location)-2)){
        // code here too thanks
      }
    }
    
    // might want logic for water valve when at location and temp to close again to avoid constant heat pump?

  }

  // Calculations and changes for automatic climate go here.
  if (climateMode == 1){
    // TODO: implement a PID algorithm to move motor dependant on everything.
    // use map() function to map the fan speed   
    //cabinActualTemp = cabinTemp / 2;
    if (selectedTemp == 35){changeAirInlet(FALSE);} // maximum cold = recirc
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
  if(millis() - prevTX >= txInt){// currently sending at 50ms interval defined by txInt
    prevTX = millis();
    byte data[8];
    // these bytes may all be backwards; as I am taking the byte 7 as the largest e.g. 10000000

    // each byte stores an 8 bit number from 0-255
    // byte 6 always contains 129
    data[6] = 129;
    // byte 5 always contains zero, as it is the passenger temp
    data[5] = 0;
    data[4] = selectedTemp;

    //TODO: these need to be correctly sent; it is possible these contain some modification to represent a different number e.g. (temp-100)/2 or something
    // byte 3 is the ambient temp;
    data[3] = ambientTemp;

    // byte 2 is AC evaporator temp - warning this is a Float; an example here is 171 = 35.5c which is going to be wrong
    data[2] = evapTemp;

    data[1] = blowerFanVoltage;

    //data[0] is fan speed; from 0 to 10 in decimal where 0 is off; 10 is max?; exception is when in full auto mode with fan speed set? add 144 to fan speed
    if (climateMode==1){data[0] = selectedBlower + 144;}
    else{data[0] = selectedBlower;}
    
    /* AC related booleans to transmit:
    and other odd booleans: personDisplayIcon
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
    */

    // Transmit data[7]
    // bitWrite(x, n, b) for data[7]
    // n: which bit of the number to write, starting at 0 for the least-significant (rightmost) bit
    // b: the value to write to the bit (0 or 1)
    // (a == 1) ? true: false

    bitWrite(data[7], 7, !acEngaged); // ac clutch engaged or not
    bitWrite(data[7], 6, !airInletMode); // recirc (inlet == false)
    bitWrite(data[7], 5, airInletMode); // fresh (inlet == true)

    // Face
    if (airOutletMode == 0 || airOutletMode == 1){bitWrite(data[7], 4, 1);}
    else{bitWrite(data[7], 4, 0);}
    //(airOutletMode == 0 || airOutletMode == 1) ? bitWrite(data[7], 4, 1) : bitWrite(data[7], 4, 0);

    // Feet / Floor
    if (airOutletMode == 1 || airOutletMode == 2 || airOutletMode == 3){bitWrite(data[7], 3, 1);}
    else{bitWrite(data[7],3,0);}

    // Screen / Window
    if (airOutletMode == 2 || airOutletMode == 4){bitWrite(data[7], 2, 1);}
    else{bitWrite(data[7],2,0);}

    bitWrite(data[7], 1, 1); // always one

    bitWrite(data[7], 0, 1); // unsure of this bit; was originally thinking this relates to auto fan but it does not

    if (climateMode == 0){
      // if climate mode is set to off
      data[0] = 0;
      data[1] = 0;
      data[2] = 171;
      data[4] = 0;
    }

    byte sndStat = CAN0.sendMsgBuf(canID, 8, data); //send the packet over CAN using the HIM's CANID of 0x353
  }
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

void moveBlendDoor(){
  // This function will start the motor moving. 
  motorMoving = TRUE;
  analogWrite(L298N_BLEND_SPEED, 30);
}

void stopBlendDoor(){
  // This function literally does as said; stops the motor from moving.
  motorMoving = FALSE;
  analogWrite(L298N_BLEND_SPEED, 0);
  analogWrite(L298N_BLEND_DIR1,LOW); 
  analogWrite(L298N_BLEND_DIR2,LOW);
}

void changeAirOutlet(int newOutletMode){
  airOutletMode = newOutletMode;
  switch (airOutletMode){
    case 0:
      // Face only
      digitalWrite(BLUE_PIN, HIGH);
      digitalWrite(RED_PIN, LOW);
      digitalWrite(BROWN_PIN, LOW);
      digitalWrite(YELLOW_PIN, LOW);
      break;
    case 1:
      // face and floor
      // if full cold; enable red pin for bi level
      digitalWrite(BLUE_PIN, HIGH);
      digitalWrite(BROWN_PIN, HIGH);
      digitalWrite(YELLOW_PIN, HIGH);
      if (selectedTemp == 35) {digitalWrite(RED_PIN, HIGH);}
      else{digitalWrite(RED_PIN, LOW);}
      break;

    case 2:
      // screen and floor
      digitalWrite(BLUE_PIN, LOW);
      digitalWrite(RED_PIN, LOW);
      digitalWrite(BROWN_PIN, LOW);
      digitalWrite(YELLOW_PIN, LOW);
      break;

    case 3:
      // feet only
      digitalWrite(BLUE_PIN, LOW);
      digitalWrite(RED_PIN, LOW);
      digitalWrite(BROWN_PIN, HIGH);
      digitalWrite(YELLOW_PIN, HIGH);
      break;

    case 4:
      // screen only (demist)
      digitalWrite(BLUE_PIN, LOW);
      digitalWrite(RED_PIN, LOW);
      digitalWrite(BROWN_PIN, LOW);
      digitalWrite(YELLOW_PIN, LOW);
      break;
  }
}

void changeAirInlet(int newInletMode){
  airInletMode = newInletMode;
  if (airInletMode == TRUE){
    // Fresh air
    digitalWrite(INLET_PIN, LOW);
  }
  else{
    // Recirculate
    digitalWrite(INLET_PIN, HIGH);
  }
}

void changeBlowerMotor(){
  // Input is global variable selectedBlower; this code only used for the semi auto mode.
  // TODO: will need to check the logic of the HIGH and LOW order to ensure the thing is going the correct direction.
  if (selectedBlower !=10){analogWrite(L298N_FSC_DIR1,LOW); analogWrite(L298N_FSC_DIR2,HIGH);}
  else {analogWrite(L298N_FSC_DIR1,HIGH); analogWrite(L298N_FSC_DIR2,LOW);}

  switch(selectedBlower){
    case 0:
      analogWrite(L298N_FSC_SPEED, 12);
      break;
    case 1:
      analogWrite(L298N_FSC_SPEED, 49);
      break;
    case 2:
      analogWrite(L298N_FSC_SPEED, 64);
      break;
    case 3:
      analogWrite(L298N_FSC_SPEED, 81);
      break;
    case 4:
      analogWrite(L298N_FSC_SPEED, 92);
      break;
    case 5:
      analogWrite(L298N_FSC_SPEED, 107);
      break;
    case 6:
      analogWrite(L298N_FSC_SPEED, 123);
      break;
    case 7:
      analogWrite(L298N_FSC_SPEED, 142);
      break;
    case 8:
      analogWrite(L298N_FSC_SPEED, 159);
      break;
    case 9:
      analogWrite(L298N_FSC_SPEED, 195);
      break;
    case 10:
      // Techincally this should be outputting approx -1v
      analogWrite(L298N_FSC_SPEED, 20);
      break;
    default:
      analogWrite(L298N_FSC_SPEED, 12);
      // sets the default to 12 = aka off
      break;
  }
  // TODO: Also set blowerFanVoltage is actually reported from the output of the FSC to a different pin
}

void changeWaterValve(int newValveMode){
  waterValveMode = newValveMode;
  // the water valve is closed when grounded
  if (waterValveMode == FALSE){
    // water valve closed
    digitalWrite(WATER_VALVE_PIN, HIGH);
  }
  else{
    // water valve open
    digitalWrite(WATER_VALVE_PIN, LOW);
  }
}
