#include <stdint.h>

#define MSP_OSD_CONFIG            84        //out message         Get osd settings - betaflight
#define MSP_NAME                  10
#define MSP_BATTERY_STATE         130       //out message         Connected/Disconnected, Voltage, Current Used

struct msp_osd_config_t {
    uint8_t osdflags;
    uint8_t video_system;
    uint8_t units;
    uint8_t rssi_alarm;
    uint16_t cap_alarm;
    uint8_t old_timer_alarm;
    uint8_t osd_item_count;                     //56
    uint16_t alt_alarm;
    uint16_t osd_rssi_value_pos;
    uint16_t osd_main_batt_voltage_pos;
    uint16_t osd_crosshairs_pos;
    uint16_t osd_artificial_horizon_pos;
    uint16_t osd_horizon_sidebars_pos;
    uint16_t osd_item_timer_1_pos;
    uint16_t osd_item_timer_2_pos;
    uint16_t osd_flymode_pos;
    uint16_t osd_craft_name_pos;
    uint16_t osd_throttle_pos_pos;
    uint16_t osd_vtx_channel_pos;
    uint16_t osd_current_draw_pos;
    uint16_t osd_mah_drawn_pos;
    uint16_t osd_gps_speed_pos;
    uint16_t osd_gps_sats_pos;
    uint16_t osd_altitude_pos;
    uint16_t osd_roll_pids_pos;
    uint16_t osd_pitch_pids_pos;
    uint16_t osd_yaw_pids_pos;
    uint16_t osd_power_pos;
    uint16_t osd_pidrate_profile_pos;
    uint16_t osd_warnings_pos;
    uint16_t osd_avg_cell_voltage_pos;
    uint16_t osd_gps_lon_pos;
    uint16_t osd_gps_lat_pos;
    uint16_t osd_debug_pos;
    uint16_t osd_pitch_angle_pos;
    uint16_t osd_roll_angle_pos;
    uint16_t osd_main_batt_usage_pos;
    uint16_t osd_disarmed_pos;
    uint16_t osd_home_dir_pos;
    uint16_t osd_home_dist_pos;
    uint16_t osd_numerical_heading_pos;
    uint16_t osd_numerical_vario_pos;
    uint16_t osd_compass_bar_pos;
    uint16_t osd_esc_tmp_pos;
    uint16_t osd_esc_rpm_pos;
    uint16_t osd_remaining_time_estimate_pos;
    uint16_t osd_rtc_datetime_pos;
    uint16_t osd_adjustment_range_pos;
    uint16_t osd_core_temperature_pos;
    uint16_t osd_anti_gravity_pos;
    uint16_t osd_g_force_pos;
    uint16_t osd_motor_diag_pos;
    uint16_t osd_log_status_pos;
    uint16_t osd_flip_arrow_pos;
    uint16_t osd_link_quality_pos;
    uint16_t osd_flight_dist_pos;
    uint16_t osd_stick_overlay_left_pos;
    uint16_t osd_stick_overlay_right_pos;
    uint16_t osd_display_name_pos;
    uint16_t osd_esc_rpm_freq_pos;
    uint16_t osd_rate_profile_name_pos;
    uint16_t osd_pid_profile_name_pos;
    uint16_t osd_profile_name_pos;
    uint16_t osd_rssi_dbm_value_pos;
    uint16_t osd_rc_channels_pos;
    uint8_t osd_stat_count;                     //24
    uint8_t osd_stat_rtc_date_time;
    uint8_t osd_stat_timer_1;
    uint8_t osd_stat_timer_2;
    uint8_t osd_stat_max_speed;
    uint8_t osd_stat_max_distance;
    uint8_t osd_stat_min_battery;
    uint8_t osd_stat_end_battery;
    uint8_t osd_stat_battery;
    uint8_t osd_stat_min_rssi;
    uint8_t osd_stat_max_current;
    uint8_t osd_stat_used_mah;
    uint8_t osd_stat_max_altitude;
    uint8_t osd_stat_blackbox;
    uint8_t osd_stat_blackbox_number;
    uint8_t osd_stat_max_g_force;
    uint8_t osd_stat_max_esc_temp;
    uint8_t osd_stat_max_esc_rpm;
    uint8_t osd_stat_min_link_quality;
    uint8_t osd_stat_flight_distance;
    uint8_t osd_stat_max_fft;
    uint8_t osd_stat_total_flights;
    uint8_t osd_stat_total_time;
    uint8_t osd_stat_total_dist;
    uint8_t osd_stat_min_rssi_dbm;
    uint16_t osd_timer_count;
    uint16_t osd_timer_1;
    uint16_t osd_timer_2;
    uint16_t enabledwarnings;
    uint8_t osd_warning_count;              // 16
    uint32_t enabledwarnings_1_41_plus;
    uint8_t osd_profile_count;              // 1
    uint8_t osdprofileindex;                // 1
    uint8_t overlay_radio_mode;             //  0
} __attribute__ ((packed));

struct msp_name_t {
    char craft_name[15];                    //15 characters max possible displayed in the goggles
} __attribute__ ((packed));

struct msp_battery_state_t {
  uint8_t batteryCellCount;
    uint16_t batteryCapacity;
    uint8_t legacyBatteryVoltage;
    uint16_t mAhDrawn;
    uint16_t amperage;
    uint8_t batteryState;
    uint16_t batteryVoltage;
} __attribute__ ((packed));

// MSP_STATUS_DJI reply
struct msp_status_DJI_t {
  uint16_t cycleTime;
  uint16_t i2cErrorCounter;
  uint16_t sensor;                    // MSP_STATUS_SENSOR_...
  uint32_t flightModeFlags;           // see getActiveModes()
  uint8_t  configProfileIndex;
  uint16_t averageSystemLoadPercent;  // 0...100
  uint16_t armingFlags;   //0x0103 or 0x0301
  uint8_t  accCalibrationAxisFlags;  //0
  uint8_t  DJI_ARMING_DISABLE_FLAGS_COUNT; //25
  uint32_t djiPackArmingDisabledFlags; //(1 << 24)
} __attribute__ ((packed));