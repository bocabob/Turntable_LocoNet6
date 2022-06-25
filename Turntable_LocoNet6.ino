/* DCC Stepper Motor Controller ( A4988 ) Example
 See: https://www.dccinterface.com/how-to/assemblyguide/
 
 Author: Alex Shepherd 2017-12-04, modified by Bob Gamble for LocoNet 2022-1-21
 
 This example requires two Arduino Libraries:

 1) The AccelStepper library from: http://www.airspayce.com/mikem/arduino/AccelStepper/index.html

 2) The LocoNet Library from: http://mrrwa.org/download/

 Both libraries can be installed via the Arduino IDE Library Manager 

 This sketch uses:
1) the LocoNet, Wire, AccelStepper, and EEPROM libraries;
2) a LocoNet Shield (John Plocher's design or similar)
3) A4988 stepper motor driver module
4) Hall sensor
5) I2C

Pin usage:
The LocoNet Shield uses pins 7 & 8
The A4988 module interface uses pin 4, 5, & 6
The Hall sensor uses pin D3
The I2C module uses pins D2, A4, A5, GND and +5V

The buttons and LEDs use pins as defined below
Button 1 increments to the next position
Button 2 decrements to the next position
Button 3 bumps clockwise one degree
Button 4 bumps counter-clockwise one degree
additional functions may be performed by holding down one button and then pressing and releasing another, then releasing the first button
Alt-Button 1 causes the home position to be refound
Alt-Button 2 sets the zero position to the current location
Alt-Button 3 rotates 180 degrees


*/

#define DEBUG_PRINT false

#include <Wire.h>
#include <AccelStepper.h>
#include <LocoNet.h>
#include<EEPROM.h>
#include<I2CKeyPad.h>

#define STEPPER_INTERFACE 1
#define STEPPER_STEP_PIN 4
#define STEPPER_DIR_PIN 5
// Uncomment to enable Powering-Off the Stepper if its not running 
#define STEPPER_ENABLE_PIN 13

// Home Position Sensor Input
#define HOME_SENSOR_PIN 3

// Pin for interupt from I2C module
#define KeyPad_Int_Pin 2

// Define your LocoNet TX Pin below, RX is on 8
#define  TX_PIN   7

int setupPin=A2;

static   lnMsg        *LnPacket;

#define NumberOfStations 14
 
const uint8_t KEYPAD_ADDRESS = 0x20;
I2CKeyPad keyPad(KEYPAD_ADDRESS);
char keymap[19] = "123A456B789C*0#DNF";  // N = NoKey, F = Fail (e.g. >1 keys pressed)

// two different lay out styles of a nummeric keyPad
//char phone_layout[19]      = "123A456B789C*0#DNF";  // N = NoKey, F = Fail
//char calculator_layout[19] = "789A456B123C*0#DNF";  // N = NoKey, F = Fail

// volatile for IRQ var
volatile bool keyChange = false;

#define buffLen 10
  char buff[buffLen];
  uint8_t bufferIndex = 0;

typedef struct
{
	int address;
	long stationFront;
	long stationBack;
}
DCCAccessoryAddress;
DCCAccessoryAddress gAddresses[NumberOfStations];

// for a 1.8 deg stepper, there are 200 full steps
const long FULL_STEPS_PER_REVOLUTION = 200*15;

// Uncomment the lime below for the Driver Board Settings
//const long FULL_TURN_STEPS = (FULL_STEPS_PER_REVOLUTION);     // full steps
//const long FULL_TURN_STEPS = (FULL_STEPS_PER_REVOLUTION * 2); // 1/2 steps
//const long FULL_TURN_STEPS = (FULL_STEPS_PER_REVOLUTION * 4); // 1/4 steps
//const long FULL_TURN_STEPS = (FULL_STEPS_PER_REVOLUTION * 8); // 1/8 steps
const long FULL_TURN_STEPS = (FULL_STEPS_PER_REVOLUTION * 16); // 1/16 steps

// define the position of each track

const long Bump = 10;

const long entryStation1 = ((FULL_TURN_STEPS / 36 * -2)  - 575);
const long entryStation2 = ((FULL_TURN_STEPS / 36 * -1) - 16);
const long entryStation3 = -90;       // home location
const long houseStation1 = ((FULL_TURN_STEPS / 36 * -11)  + 320);
const long houseStation2 = ((FULL_TURN_STEPS / 36 * -10)  + 255);
const long houseStation3 = ((FULL_TURN_STEPS / 36 * -9)  + 290);
const long houseStation4 = ((FULL_TURN_STEPS / 36 * -8)  + 275);
const long houseStation5 = ((FULL_TURN_STEPS / 36 * -7)  + 205);
const long houseStation6 = ((FULL_TURN_STEPS / 36 * -6)  + 170);
const long houseStation7 = ((FULL_TURN_STEPS / 36 * -5)  + 112);
const long houseStation8 = ((FULL_TURN_STEPS / 36 * -4) + 30);
const long houseStation9 = ((FULL_TURN_STEPS / 36 * -3)  + 10);
const long houseStation10 = ((FULL_TURN_STEPS / 36 * -2)  + 140);
const long houseStation11 = ((FULL_TURN_STEPS / 36 * 0) + 38);

volatile bool bInterruptDetected = false;
bool bHomePositionFound = false;

bool lastIsRunningState;

// Now we'll wrap the stepper in an AccelStepper object
AccelStepper stepper1(STEPPER_INTERFACE, STEPPER_STEP_PIN, STEPPER_DIR_PIN);

uint16_t lastAddr = 0xFFFF;
uint8_t lastDirection = 0xFF;

int CurrentStation = 2;
int CurrentDirection = 0;

//
void ConfigureStations()
{
	// this is home
	gAddresses[0].address = 500;
	gAddresses[0].stationFront = entryStation1;
	gAddresses[0].stationBack = entryStation1 + (FULL_TURN_STEPS / 2);

	gAddresses[1].address = 501;
	gAddresses[1].stationFront = entryStation2;
	gAddresses[1].stationBack = entryStation2 + (FULL_TURN_STEPS / 2);

	gAddresses[2].address = 502;
	gAddresses[2].stationFront = entryStation3;
	gAddresses[2].stationBack = entryStation3 + (FULL_TURN_STEPS / 2);

	gAddresses[3].address = 503;
	gAddresses[3].stationFront = houseStation1;
	gAddresses[3].stationBack = houseStation1 + (FULL_TURN_STEPS / 2);

	gAddresses[4].address = 504;
	gAddresses[4].stationFront = houseStation2;
	gAddresses[4].stationBack = houseStation2 + (FULL_TURN_STEPS / 2);

	gAddresses[5].address = 505;
	gAddresses[5].stationFront = houseStation3;
	gAddresses[5].stationBack = houseStation3 + (FULL_TURN_STEPS / 2);

	gAddresses[6].address = 506;
	gAddresses[6].stationFront = houseStation4;
	gAddresses[6].stationBack = houseStation4 + (FULL_TURN_STEPS / 2);

	gAddresses[7].address = 507;
	gAddresses[7].stationFront = houseStation5;
	gAddresses[7].stationBack = houseStation5 + (FULL_TURN_STEPS / 2);

	gAddresses[8].address = 508;
	gAddresses[8].stationFront = houseStation6;
	gAddresses[8].stationBack = houseStation6 + (FULL_TURN_STEPS / 2);

	gAddresses[9].address = 509;
	gAddresses[9].stationFront = houseStation7;
	gAddresses[9].stationBack = houseStation7 + (FULL_TURN_STEPS / 2);

	gAddresses[10].address = 510;
	gAddresses[10].stationFront = houseStation8;
	gAddresses[10].stationBack = houseStation8 + (FULL_TURN_STEPS / 2);

	gAddresses[11].address = 511;
	gAddresses[11].stationFront = houseStation9;
	gAddresses[11].stationBack = houseStation9 + (FULL_TURN_STEPS / 2);

	gAddresses[12].address = 512;
	gAddresses[12].stationFront = houseStation10;
	gAddresses[12].stationBack = houseStation10 + (FULL_TURN_STEPS / 2);

	gAddresses[13].address = 513;
	gAddresses[13].stationFront = houseStation11;
	gAddresses[13].stationBack = houseStation11+ (FULL_TURN_STEPS / 2);
}

// This call-back function is called from LocoNet.processSwitchSensorMessage
// for all Switch Request messages

void setupStepperDriver()
{
#ifdef STEPPER_ENABLE_PIN
	stepper1.setPinsInverted(false, false, true); // Its important that these commands are in this order
	stepper1.setEnablePin(STEPPER_ENABLE_PIN);    // otherwise the Outputs are NOT enabled initially
#endif

	stepper1.setMaxSpeed(2000.0);
	stepper1.setAcceleration(1000);
	stepper1.setSpeed(1000);

	lastIsRunningState = stepper1.isRunning();
}

void keyChanged()
{
  keyChange = true;
}

void interruptEvent()
{
	detachInterrupt(digitalPinToInterrupt(HOME_SENSOR_PIN));
	bInterruptDetected = true;
  
  attachInterrupt(digitalPinToInterrupt(KeyPad_Int_Pin), keyChanged, FALLING);
  keyChange = false;
}

void setupLocoNet()
{
  Serial.println(F("Setting up LocoNet node..."));

    // First initialize the LocoNet interface, specifying the TX Pin
  LocoNet.init(TX_PIN);
}

void setup()
{
	Serial.begin(115200);
	while (!Serial);   // Wait for the USB Device to Enumerate
//  Serial.println(__FILE__);
	Serial.println(F("Turntable Program Started"));
 
  pinMode(HOME_SENSOR_PIN, INPUT_PULLUP);
    
	ConfigureStations();

	setupStepperDriver();
 
  // NOTE: PCF8574 will generate an interrupt on key press and release.
  pinMode(KeyPad_Int_Pin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(KeyPad_Int_Pin), keyChanged, FALLING);
  keyChange = false;


  Wire.begin();
  Wire.setClock(100000);
  if (keyPad.begin() == false)
  {
    Serial.println("\nERROR: cannot communicate to keypad.\nPlease reboot.\n");
    while (1);
  }
  keyPad.loadKeyMap(keymap);

	Serial.println(F("Finding home...."));
	moveToHomePosition();
}

void notifySwitchRequest( uint16_t Addr, uint8_t OutputPower, uint8_t Direction ) {

  int outputInPair = (Direction == 32);
#ifdef DEBUG_PRINT
// the following prints to the serial monitor for debugging purposes
  Serial.print(F("notifySwitchRequestOutput: "));
  Serial.print(Addr, DEC);
  Serial.print(',');
  Serial.print(Direction, DEC);
  Serial.print(',');
  Serial.print(OutputPower, DEC);
  Serial.print(',');
  Serial.print(F(" outputInPair = "));
  Serial.println(outputInPair, DEC);
#endif  

  for (int i = 0; i < (sizeof(gAddresses) / sizeof(DCCAccessoryAddress)); i++)
  {
    if ((Addr == gAddresses[i].address) && ((Addr != lastAddr) || (Direction != lastDirection)) && OutputPower)
    {     
       
      LN_STATUS lnStatus = LocoNet.reportSensor(lastAddr,0);
#ifdef DEBUG_PRINT      
      Serial.print(F("Tx:  Sensor Off: "));
      Serial.print(gAddresses[i].address, DEC);
      Serial.print(F(" Status: "));
      Serial.println(LocoNet.getStatusStr(lnStatus));
#endif      
      MoveToStation(i,Direction);
    }
  }
}

void moveToHomePosition()
{  
  detachInterrupt(digitalPinToInterrupt(KeyPad_Int_Pin));
  
  attachInterrupt(digitalPinToInterrupt(HOME_SENSOR_PIN), interruptEvent, RISING);

  bInterruptDetected = false;

  Serial.println(F("Performing 2 complete turns to find home."));
     
#ifdef STEPPER_ENABLE_PIN
      stepper1.enableOutputs();
#endif

  stepper1.move(FULL_TURN_STEPS * 2);
 for (int i = 0; i < (sizeof(gAddresses) / sizeof(DCCAccessoryAddress)); i++)
 { 
      LN_STATUS lnStatus = LocoNet.reportSensor(gAddresses[i].address,0);
#ifdef DEBUG_PRINT      
      Serial.print(F("Tx:  Sensor Off: "));
      Serial.print(gAddresses[i].address, DEC);
      Serial.print(F(" Status: "));
      Serial.println(LocoNet.getStatusStr(lnStatus));   
#endif       
  }
}

void MoveToStation(int i,uint8_t Direction){
      if (i > NumberOfStations) return;
      EEPROM.put(1,i);
      EEPROM.put(3,Direction);
      
      LN_STATUS lnStatus = LocoNet.reportSensor(gAddresses[CurrentStation].address,0);
#ifdef DEBUG_PRINT      
      Serial.print(F("Tx:  Sensor Off: "));
      Serial.print(gAddresses[CurrentStation].address, DEC);
      Serial.print(F(" Status: "));
      Serial.println(LocoNet.getStatusStr(lnStatus));
#endif      
      lastAddr = gAddresses[i].address;
      lastDirection = Direction;
      CurrentStation = i;
      CurrentDirection = Direction;

      Serial.print(F("Moving to Station : "));
      Serial.println(i, DEC);

      lnStatus = LocoNet.reportSensor(gAddresses[i].address,1);
#ifdef DEBUG_PRINT      
      Serial.print(F("Tx:  Sensor On: "));
      Serial.print(gAddresses[i].address, DEC);
      Serial.print(F(" Status: "));
      Serial.println(LocoNet.getStatusStr(lnStatus));
#endif     
#ifdef STEPPER_ENABLE_PIN
      stepper1.enableOutputs();
#endif
      if (Direction)
      {        
       stepper1.moveTo(gAddresses[i].stationFront);
#ifdef DEBUG_PRINT        
        Serial.print(F("Moving to Front Position : "));
        Serial.println(gAddresses[i].stationFront, DEC);
#endif        
 //       break;
      }
      else
      {
       stepper1.moveTo(gAddresses[i].stationBack);
#ifdef DEBUG_PRINT       
        Serial.print(F("Moving to Back Position : "));
        Serial.println(gAddresses[i].stationBack, DEC);
#endif          
      }
}

void IncrementStation(){
                       // Go to higher position
              Serial.println(F("Increment Station"));
              EEPROM.get(1,  CurrentStation);
              EEPROM.get(3,  CurrentDirection);
              if(CurrentStation + 1 > NumberOfStations)
                {
                  MoveToStation(0,CurrentDirection);
                }
              else
                {
                  MoveToStation(CurrentStation + 1,CurrentDirection);
                }
}
void DecrementStation(){
                       //      Go to lower position
              Serial.println(F("Decrement Station"));            
              EEPROM.get(1,  CurrentStation);
              EEPROM.get(3,  CurrentDirection);
              if(CurrentStation - 1 < 0)
                {
                  MoveToStation(NumberOfStations,CurrentDirection);
                }
              else
                {
                  MoveToStation(CurrentStation - 1,CurrentDirection);
                }
}
void BumpForeward(int x){
                        // bump clockwise           
              Serial.println(F("Bump foreward"));     
                #ifdef STEPPER_ENABLE_PIN
                      stepper1.enableOutputs();
                #endif
              stepper1.move(Bump * x);
}
void BumpBack(int x){
                         // bump counter-clockwise          
              Serial.println(F("Bump back"));     
                #ifdef STEPPER_ENABLE_PIN
                      stepper1.enableOutputs();
                #endif
              stepper1.move(-Bump * x);
}
void ResetHome(){
                         // reset home position
            Serial.println(F("Reset Home Position...."));
            bHomePositionFound = false;
            moveToHomePosition();
}
void SetHome(){
                        // set home to current position         
              Serial.println(F("Set Home Here"));    
              stepper1.setCurrentPosition(-90);                 
}
void Turn180(){
                          //          flip 180
              Serial.println(F("Turn 180 degrees"));       
                #ifdef STEPPER_ENABLE_PIN
                      stepper1.enableOutputs();
                #endif
              stepper1.move(FULL_TURN_STEPS / 2);                  
}
void Case8(){
                       //          change direction
              Serial.println(F("Change Direction"));    
                
}
void keyPadCommand()
{
#ifdef DEBUG_PRINT
    Serial.print(F("BufferIndex: "));
    Serial.println(bufferIndex);
#endif    
  if (bufferIndex <= 1)
  {
    switch (buff[0]) {
  case '#':
    //
    IncrementStation();
    break;
  case '*':
    // 
    DecrementStation();
    break;
  case 'A':
    //
    BumpForeward(1);
    break;
  case 'B':
    // 
    BumpBack(1);
    break;
  case 'C':
    //
    ResetHome();
    break;
  case 'D':
    // 
    Turn180();
    break;
  default:
    // statements
    keyChange = false;
    return;
    break;
    }

  }
  if (bufferIndex == 2)
  {
  switch (buff[1]) {
  case '#':
    // move to station foreward
    MoveToStation(buff[0] - 48,32);
    break;
  case '*':
    // move to station backward
    MoveToStation(buff[0] - 48,0);
    break;    
  case 'A':
    //
    BumpForeward(buff[0] - 48);
    break;
  case 'B':
    // 
    BumpBack(buff[0] - 48);
    break;
  case 'C':
    //
    SetHome();
    break;
  case 'D':
    // 
    Turn180();
    break;
  default:
    // statements
    keyChange = false;
    return;
    break;
    }

  }   
  if (bufferIndex == 3)
  {
  switch (buff[2]) {
  case '#':
    // move to station foreward
    MoveToStation(((buff[0] - 48) * 10) + (buff[1] - 48),32);
    break;
  case '*':
    // move to station backward
    MoveToStation(((buff[0] - 48) * 10) + (buff[1] - 48),0);
    break; 
  case 'A':
    //
    BumpForeward(((buff[0] - 48) * 10) + (buff[1] - 48));
    break;
  case 'B':
    // 
    BumpBack(((buff[0] - 48) * 10) + (buff[1] - 48));
    break;
  default:
    // statements
//    bufferIndex = 0;
    break;
    }
  } 
#ifdef DEBUG_PRINT  
    Serial.print(F("Buffer: "));
    Serial.println(buff);
#endif    
      for (int i = 0; i < (sizeof(buff)); i++)
      {
        buff[i]   = 0;
      }

    bufferIndex = 0;
    keyChange = false;
}

void loop()
{
	if (bInterruptDetected)
	{
		bInterruptDetected = false;
		bHomePositionFound = true;

		Serial.println(F("Found Home - Setting Current Position to 0"));

		stepper1.setCurrentPosition(0);
//    EEPROM.get(3,  CurrentDirection);

    MoveToStation(2,1);
    
    setupLocoNet(); 
	}
 
	if (bHomePositionFound)
	{
		// Check LocoNet for pending switch commands:
    LnPacket=LocoNet.receive();
    if(LnPacket){
      LocoNet.processSwitchSensorMessage(LnPacket);
    }
	}

	// Process the Stepper Library
	stepper1.run();

#ifdef STEPPER_ENABLE_PIN
	if (stepper1.isRunning() != lastIsRunningState)
	{
		lastIsRunningState = stepper1.isRunning();
		if (!lastIsRunningState)
		{
			stepper1.disableOutputs();
			Serial.println(F("Disable Stepper Outputs"));
		}
	}
#endif  

if (keyChange)
  {
    uint8_t index = keyPad.getKey();

    if (index < 16)
    {
#ifdef DEBUG_PRINT      
      Serial.print("press: ");
      Serial.println(keymap[index]);
#endif      
      buff[bufferIndex++] = keymap[index];
      if (keymap[index] == '#' || keymap[index] == '*' || keymap[index] == 'A' || keymap[index] == 'B' || keymap[index] == 'C' || keymap[index] == 'D')
      {
        keyPadCommand();
      }  
      else
      {
        delay(10);
        keyChange = false;  
      }    
    }  
    else
    {
      keyChange = false;  
    }
  }
}
//
//// This call-back function is called from LocoNet.processSwitchSensorMessage
//// for all Sensor messages
//void notifySensor( uint16_t Address, uint8_t State ) {
//  Serial.print("Sensor: ");
//  Serial.print(Address, DEC);
//  Serial.print(" - ");
//  Serial.println( State ? "Active" : "Inactive" );
//}
//
//
//// This call-back function is called from LocoNet.processSwitchSensorMessage
//// for all Switch Output Report messages
//void notifySwitchOutputsReport( uint16_t Address, uint8_t ClosedOutput, uint8_t ThrownOutput) {
//  Serial.print("Switch Outputs Report: ");
//  Serial.print(Address, DEC);
//  Serial.print(": Closed - ");
//  Serial.print(ClosedOutput ? "On" : "Off");
//  Serial.print(": Thrown - ");
//  Serial.println(ThrownOutput ? "On" : "Off");
//}
//
//// This call-back function is called from LocoNet.processSwitchSensorMessage
//// for all Switch Sensor Report messages
//void notifySwitchReport( uint16_t Address, uint8_t State, uint8_t Sensor ) {
//  Serial.print("Switch Sensor Report: ");
//  Serial.print(Address, DEC);
//  Serial.print(':');
//  Serial.print(Sensor ? "Switch" : "Aux");
//  Serial.print(" - ");
//  Serial.println( State ? "Active" : "Inactive" );
//}
//
//// This call-back function is called from LocoNet.processSwitchSensorMessage
//// for all Switch State messages
//void notifySwitchState( uint16_t Address, uint8_t Output, uint8_t Direction ) {
//  Serial.print("Switch State: ");
//  Serial.print(Address, DEC);
//  Serial.print(':');
//  Serial.print(Direction ? "Closed" : "Thrown");
//  Serial.print(" - ");
//  Serial.println(Output ? "On" : "Off");
//}
