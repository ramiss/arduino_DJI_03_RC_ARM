/******************************************************************
  @file       arduino_DJI_03_RC_ARM.ino
  @brief      Send DJI Arm or Disarm over MSP based on RC Input. 
  @author     Richard Amiss
  
  Code:        Richard Amiss
  Version:     1.0.2
  Date:        05/29/23
******************************************************************/

#include <AltSoftSerial.h>
#include <ReefwingMSP.h>
#include <PulseInput.h>   

#define ANALOG_IN                A1
#define VOLT_DIVIDER             51    // Set to 1024/full scale voltage

ReefwingMSP msp;
#define MSP_STATUS_EX            150   //out message		 For OSD ‘Fly mode', For OSD ‘Disarmed’
#define MSP_BATTERY_STATE        130

bool always_arm = false;          // set this to true to ignore RC input and continue sending the arm signal.

msp_api_version_t api;
msp_ident_t identReply;
msp_packet_t packet;
msp_fc_variant_t variant;

AltSoftSerial SerialMSP;

// DJI STUFF
// MSP_STATUS_DJI
struct msp_status_DJI_t {
  uint16_t cycleTime;
  uint16_t i2cErrorCounter;
  uint16_t sensor;                    
  uint32_t flightModeFlags;           
  uint8_t  configProfileIndex;
  uint16_t averageSystemLoadPercent;  // 0...100
  uint16_t armingFlags;   //0x0103 or 0x0301
  uint8_t  accCalibrationAxisFlags;  //0
  uint8_t  DJI_ARMING_DISABLE_FLAGS_COUNT; //25
  uint32_t djiPackArmingDisabledFlags; //(1 << 24)
} __attribute__ ((packed));

struct msp_status_battery_t {
  uint8_t  cellCount;       // 0 = no battery
  uint16_t capacity;        // in mAh
  uint8_t  legacyVoltage;   // in 0.1V       
  uint16_t mAhDrawn;           
  uint16_t current;         // in 0.01A
  uint8_t  alertFlags;
  uint16_t voltage;         // in 0.01V
} __attribute__ ((packed));

uint32_t flightModeFlags = 0x00000002;
msp_status_DJI_t status_DJI = { 0 };
msp_status_battery_t status_batt = { 0 };

volatile unsigned int rcInput;    /* each signal requires a variable */

void setup() {
  //  Start USB Serial
  Serial.begin(115200);

  //  Start MSP Serial
  SerialMSP.begin(115200);   // to AltSoftSerial RX

  // Assign RC Input
  attachPulseInput(2, rcInput);  // pin 5 (D2) as INPUT_PULLUP

  //  Allocate stream and timeout (default timeout = 500)
  msp.begin(SerialMSP);

  //  Original MSP message response
  identReply.multiWiiVersion = 0;
  identReply.multiType = QUADX;
  identReply.mspVersion = MSP_PROTOCOL_VERSION;
  identReply.capability = MSP_FEATURE_VBAT;

  pinMode(LED_BUILTIN, OUTPUT);

  //  Betaflight message variant
  strcpy(variant.flightControlIdentifier, REEFWING_IDENTIFIER);
  Serial.println("Initialized");
}

void loop() {
  // poll rcInput
  Serial.print(rcInput);

  if (rcInput > 1500 || always_arm) {
    flightModeFlags = 0x00000003;    // armed 
    Serial.println(": Armed");
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on 
  } else {
    flightModeFlags = 0x00000002;    // disarmed
    Serial.println(": Disarmed");
    digitalWrite(LED_BUILTIN, LOW);    // turn the LED off 
  }

  status_DJI.flightModeFlags = flightModeFlags;
  status_DJI.armingFlags = 0x0303;
  msp.send(MSP_STATUS_EX, &status_DJI, sizeof(status_DJI));
  status_DJI.armingFlags = 0x0000;
  msp.send(MSP_STATUS, &status_DJI, sizeof(status_DJI));  

  status_batt.legacyVoltage = analogRead(ANALOG_IN)*10/VOLT_DIVIDER;
  msp.send(MSP_BATTERY_STATE, &status_batt, sizeof(status_batt));

  delay(100);
}
