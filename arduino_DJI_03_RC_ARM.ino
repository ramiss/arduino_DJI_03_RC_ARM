/************************************************************************************
  @file       arduino_DJI_03_RC_ARM.ino
  @brief      Send DJI Arm using XIAO Seeeduino over MSP to keep power level at full.
  @author     Richard Amiss
  
  Release Note:  
  Complete rework for XIAO Seeeduino and MSP libraries. RC input is no longer used in 
  this version.  The AU will simply arm a few seconds after being turned on, as long 
  as the goggles are on.
  
  Code:        Richard Amiss
  Version:     1.1.0
  Date:        06/24/23

  Credits:
  This software is based on and uses software published by 2022 David Payne:
  QLiteOSD, which is based on work by Paul Kurucz (pkuruz):opentelem_to_bst_bridge 
  as well as software from d3ngit : djihdfpv_mavlink_to_msp_V2 and 
  crashsalot : VOT_to_DJIFPV

THIS SOFTWARE IS PROVIDED IN AN "AS IS" CONDITION. NO WARRANTIES, WHETHER EXPRESS, 
IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF 
MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE 
COMPANY SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR 
CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
************************************************************************************/

#include "MSP.h"
#include "MSP_OSD.h"
#include "OSD_positions_config.h"

#define ANALOG_IN                A0    // Voltage Read pin (notice this is now Pin 0, instead of Pin 1)
#define VOLT_DIVIDER             48    // Set to 1024/full scale voltage
//#define DEBUG                          //uncomment to see diagnostics from USB serial

#define FC_FIRMWARE_NAME          "Betaflight"
#define FC_FIRMWARE_IDENTIFIER    "BTFL"

HardwareSerial &mspSerial = Serial1;
MSP msp;

// Arm Logic
uint32_t unarmedMillis = 3000;   // number of milliseconds to wait before arming, after AU is initialized. Recommend at least 3000 (3 seconds).

//Voltage and Battery Reading
float averageVoltage = 0;
int sampleVoltageCount = 0;
int lastCount = 0;

boolean lightOn = true;

uint8_t *messageID;
void *payload;
uint8_t maxSize;
uint8_t *recvSize;

//Other
char fcVariant[5] = "BTFL";
char craftname[15] = "Craft";
uint32_t previousMillis_MSP = 0;
uint32_t activityDetectedMillis_MSP = 0;
bool activityDetected = false;
const uint32_t next_interval_MSP = 100;
uint32_t general_counter = next_interval_MSP;
uint32_t custom_mode = 0;  //flight mode
uint8_t vbat = 0;
uint32_t flightModeFlags = 0x00000002;
uint8_t batteryCellCount = 3;
uint32_t previousFlightMode = custom_mode;

msp_battery_state_t battery_state = { 0 };
msp_name_t name = { 0 };
msp_fc_version_t fc_version = { 0 };
msp_fc_variant_t fc_variant = { 0 };
//msp_status_BF_t status_BF = {0};
msp_status_DJI_t status_DJI = { 0 };
msp_analog_t analog = { 0 };

void setup() {
  #ifdef DEBUG
    SerialUSB.begin(115200);
    //while (!SerialUSB);
  #endif
  Serial1.begin(115200);
  while (!Serial1);

  analogReadResolution(12); // SAMD21 12 bit resolution 0 - 4096 range on Analog pin

  msp.begin(mspSerial);
  pinMode(LED_BUILTIN, OUTPUT);

  delay(1000);

  status_DJI.cycleTime = 0x0080;
  status_DJI.i2cErrorCounter = 0;
  status_DJI.sensor = 0x23;
  status_DJI.configProfileIndex = 0;
  status_DJI.averageSystemLoadPercent = 7;
  status_DJI.accCalibrationAxisFlags = 0;
  status_DJI.DJI_ARMING_DISABLE_FLAGS_COUNT = 20;
  status_DJI.djiPackArmingDisabledFlags = (1 << 24);
  flightModeFlags = 0x00000002;

  #ifdef DEBUG
    Serial.println("Initialized");
  #endif
}

void loop() { 
   

  if (!activityDetected) {
    #ifdef DEBUG
      Serial.println("Waiting for AU...");
    #endif
    digitalWrite(LED_BUILTIN, LOW);

    // Wait for Air Unit to send data
    while(!msp.activityDetected()) {    
    };
    activityDetected = true;
    activityDetectedMillis_MSP = millis();    
    #ifdef DEBUG
      Serial.println("AU Detected, waiting (unarmedMillis) time till arm");
    #endif
  }

  digitalWrite(LED_BUILTIN, HIGH);
  
  uint32_t currentMillis_MSP = millis();

  if ((uint32_t)(currentMillis_MSP - previousMillis_MSP) >= next_interval_MSP) {
    previousMillis_MSP = currentMillis_MSP;

    if (general_counter % 300 == 0) {  // update the altitude and voltage values every 300ms
      getVoltageSample();
      if (lightOn) {
        digitalWrite(LED_BUILTIN, LOW);
      } else {
        digitalWrite(LED_BUILTIN, HIGH);
      }
      lightOn = !lightOn;
    }

    if (currentMillis_MSP < (activityDetectedMillis_MSP + unarmedMillis)) {
      set_flight_mode_flags(false);
    } else {
      set_flight_mode_flags(true);
    }

#ifdef DEBUG
    //debugPrint();
#endif
    send_msp_to_airunit();
    general_counter += next_interval_MSP;
  }
  if (custom_mode != previousFlightMode) {
    previousFlightMode = custom_mode;
    display_flight_mode();
  }

  if (batteryCellCount == 0 && vbat > 0) {
    set_battery_cells_number();
  }

  //display flight mode every 10s
  if (general_counter % 10000 == 0) {
    display_flight_mode();
  }
}

msp_osd_config_t msp_osd_config = { 0 };

void send_osd_config() {

#ifdef IMPERIAL_UNITS
  msp_osd_config.units = 0;
#else
  msp_osd_config.units = 1;
#endif

  msp_osd_config.osd_item_count = 56;
  msp_osd_config.osd_stat_count = 24;
  msp_osd_config.osd_timer_count = 2;
  msp_osd_config.osd_warning_count = 16;  // 16
  msp_osd_config.osd_profile_count = 1;   // 1
  msp_osd_config.osdprofileindex = 1;     // 1
  msp_osd_config.overlay_radio_mode = 0;  //  0

  msp_osd_config.osd_rssi_value_pos = osd_rssi_value_pos;
  msp_osd_config.osd_main_batt_voltage_pos = osd_main_batt_voltage_pos;
  msp_osd_config.osd_crosshairs_pos = osd_crosshairs_pos;
  msp_osd_config.osd_artificial_horizon_pos = osd_artificial_horizon_pos;
  msp_osd_config.osd_horizon_sidebars_pos = osd_horizon_sidebars_pos;
  msp_osd_config.osd_item_timer_1_pos = osd_item_timer_1_pos;
  msp_osd_config.osd_item_timer_2_pos = osd_item_timer_2_pos;
  msp_osd_config.osd_flymode_pos = osd_flymode_pos;
  msp_osd_config.osd_craft_name_pos = osd_craft_name_pos;
  msp_osd_config.osd_throttle_pos_pos = osd_throttle_pos_pos;
  msp_osd_config.osd_vtx_channel_pos = osd_vtx_channel_pos;
  msp_osd_config.osd_current_draw_pos = osd_current_draw_pos;
  msp_osd_config.osd_mah_drawn_pos = osd_mah_drawn_pos;
  msp_osd_config.osd_gps_speed_pos = osd_gps_speed_pos;
  msp_osd_config.osd_gps_sats_pos = osd_gps_sats_pos;
  msp_osd_config.osd_altitude_pos = osd_altitude_pos;
  msp_osd_config.osd_roll_pids_pos = osd_roll_pids_pos;
  msp_osd_config.osd_pitch_pids_pos = osd_pitch_pids_pos;
  msp_osd_config.osd_yaw_pids_pos = osd_yaw_pids_pos;
  msp_osd_config.osd_power_pos = osd_power_pos;
  msp_osd_config.osd_pidrate_profile_pos = osd_pidrate_profile_pos;
  msp_osd_config.osd_warnings_pos = osd_warnings_pos;
  msp_osd_config.osd_avg_cell_voltage_pos = osd_avg_cell_voltage_pos;
  msp_osd_config.osd_gps_lon_pos = osd_gps_lon_pos;
  msp_osd_config.osd_gps_lat_pos = osd_gps_lat_pos;
  msp_osd_config.osd_debug_pos = osd_debug_pos;
  msp_osd_config.osd_pitch_angle_pos = osd_pitch_angle_pos;
  msp_osd_config.osd_roll_angle_pos = osd_roll_angle_pos;
  msp_osd_config.osd_main_batt_usage_pos = osd_main_batt_usage_pos;
  msp_osd_config.osd_disarmed_pos = osd_disarmed_pos;
  msp_osd_config.osd_home_dir_pos = osd_home_dir_pos;
  msp_osd_config.osd_home_dist_pos = osd_home_dist_pos;
  msp_osd_config.osd_numerical_heading_pos = osd_numerical_heading_pos;
  msp_osd_config.osd_numerical_vario_pos = osd_numerical_vario_pos;
  msp_osd_config.osd_compass_bar_pos = osd_compass_bar_pos;
  msp_osd_config.osd_esc_tmp_pos = osd_esc_tmp_pos;
  msp_osd_config.osd_esc_rpm_pos = osd_esc_rpm_pos;
  msp_osd_config.osd_remaining_time_estimate_pos = osd_remaining_time_estimate_pos;
  msp_osd_config.osd_rtc_datetime_pos = osd_rtc_datetime_pos;
  msp_osd_config.osd_adjustment_range_pos = osd_adjustment_range_pos;
  msp_osd_config.osd_core_temperature_pos = osd_core_temperature_pos;
  msp_osd_config.osd_anti_gravity_pos = osd_anti_gravity_pos;
  msp_osd_config.osd_g_force_pos = osd_g_force_pos;
  msp_osd_config.osd_motor_diag_pos = osd_motor_diag_pos;
  msp_osd_config.osd_log_status_pos = osd_log_status_pos;
  msp_osd_config.osd_flip_arrow_pos = osd_flip_arrow_pos;
  msp_osd_config.osd_link_quality_pos = osd_link_quality_pos;
  msp_osd_config.osd_flight_dist_pos = osd_flight_dist_pos;
  msp_osd_config.osd_stick_overlay_left_pos = osd_stick_overlay_left_pos;
  msp_osd_config.osd_stick_overlay_right_pos = osd_stick_overlay_right_pos;
  msp_osd_config.osd_display_name_pos = osd_display_name_pos;
  msp_osd_config.osd_esc_rpm_freq_pos = osd_esc_rpm_freq_pos;
  msp_osd_config.osd_rate_profile_name_pos = osd_rate_profile_name_pos;
  msp_osd_config.osd_pid_profile_name_pos = osd_pid_profile_name_pos;
  msp_osd_config.osd_profile_name_pos = osd_profile_name_pos;
  msp_osd_config.osd_rssi_dbm_value_pos = osd_rssi_dbm_value_pos;
  msp_osd_config.osd_rc_channels_pos = osd_rc_channels_pos;

  msp.send(MSP_OSD_CONFIG, &msp_osd_config, sizeof(msp_osd_config));
}


void invert_pos(uint16_t *pos1, uint16_t *pos2) {
  uint16_t tmp_pos = *pos1;
  *pos1 = *pos2;
  *pos2 = tmp_pos;
}

void set_flight_mode_flags(bool arm) {
    if ((flightModeFlags == 0x00000002) && arm) {
      flightModeFlags = 0x00000003;    // arm
      #ifdef DEBUG
        Serial.println("ARMING");
      #endif
    } else if ((flightModeFlags == 0x00000003) && !arm) {        
      flightModeFlags = 0x00000002;    // disarm 
      #ifdef DEBUG
        Serial.println("DISARMING");
      #endif
    }
}

void display_flight_mode() {
  show_text(&craftname);
}

void send_msp_to_airunit() {
  
  //MSP_FC_VARIANT
  memcpy(fc_variant.flightControlIdentifier, fcVariant, sizeof(fcVariant));
  msp.send(MSP_FC_VARIANT, &fc_variant, sizeof(fc_variant));

  //MSP_FC_VERSION
  fc_version.versionMajor = 4;
  fc_version.versionMinor = 1;
  fc_version.versionPatchLevel = 1;
  msp.send(MSP_FC_VERSION, &fc_version, sizeof(fc_version));

  //MSP_NAME
  memcpy(name.craft_name, craftname, sizeof(craftname));
  msp.send(MSP_NAME, &name, sizeof(name));

  //MSP_STATUS
  status_DJI.flightModeFlags = flightModeFlags;
  status_DJI.armingFlags = 0x0303;
  msp.send(MSP_STATUS_EX, &status_DJI, sizeof(status_DJI));
  status_DJI.armingFlags = 0x0000;
  msp.send(MSP_STATUS, &status_DJI, sizeof(status_DJI));

  //MSP_BATTERY_STATE
  battery_state.amperage = 0;
  battery_state.batteryVoltage = vbat * 10;
  battery_state.mAhDrawn = 0;
  battery_state.batteryCellCount = batteryCellCount;
  battery_state.batteryCapacity = 0;
  battery_state.batteryState = 0;
  battery_state.legacyBatteryVoltage = vbat;
  msp.send(MSP_BATTERY_STATE, &battery_state, sizeof(battery_state));
 
  //MSP_OSD_CONFIG
  send_osd_config();
}

void show_text(char (*text)[15]) {
  memcpy(craftname, *text, sizeof(craftname));
}

void set_battery_cells_number() {
  if (vbat < 43) batteryCellCount = 1;
  else if (vbat < 85) batteryCellCount = 2;
  else if (vbat < 127) batteryCellCount = 3;
  else if (vbat < 169) batteryCellCount = 4;
  else if (vbat < 211) batteryCellCount = 5;
  else if (vbat < 255) batteryCellCount = 6;
}



void getVoltageSample() {
  vbat = analogRead(ANALOG_IN)*10/VOLT_DIVIDER;
}

//*** USED ONLY FOR DEBUG ***
void debugPrint() {
  SerialUSB.println("**********************************");
  SerialUSB.print("Flight Mode: ");
  SerialUSB.println(flightModeFlags);
  SerialUSB.print("Voltage: ");
  SerialUSB.println(((double)vbat / 10), 1);
  SerialUSB.print("Sample Count / transmit: ");
  SerialUSB.println(lastCount);
  SerialUSB.print("Battery Cell Count: ");
  SerialUSB.println(batteryCellCount);
}



