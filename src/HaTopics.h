#pragma once

// Single source of truth for Home Assistant MQTT entity object_ids.
//
// Each telemetry field is referenced in three places that must stay in sync:
//   - discovery registration  (publishHaSensor(..., ha::SOC, ...))
//   - the publish loop         (pubF(ha::SOC, ...))
//   - per-car suppression      (CarInterface::supportsHaTelemetry(ha::SOC))
// Defining the object_id once here removes the slash/no-slash duplication and
// the risk of a typo silently breaking an entity. Values carry NO leading '/';
// the publish helper prepends it when building the state topic.

namespace ha
{
  // Battery
  constexpr const char *SOC = "soc";
  constexpr const char *SOC_KWH = "soc_kwh";
  constexpr const char *SOH = "soh";
  constexpr const char *BAT_POWER = "bat_power";
  constexpr const char *BAT_CURRENT = "bat_current";
  constexpr const char *BAT_VOLTAGE = "bat_voltage";
  constexpr const char *BAT_TEMP = "bat_temp";
  constexpr const char *CELL_TEMP_MIN = "cell_temp_min";
  constexpr const char *CELL_TEMP_MAX = "cell_temp_max";
  constexpr const char *CELL_VOLTAGE_MIN = "cell_voltage_min";
  constexpr const char *CELL_VOLTAGE_MAX = "cell_voltage_max";

  // 12V Aux
  constexpr const char *AUX_VOLTAGE = "aux_voltage";
  constexpr const char *AUX_CURRENT = "aux_current";
  constexpr const char *AUX_SOC = "aux_soc";

  // Charging
  constexpr const char *CHARGING_ON = "charging_on";
  constexpr const char *CHARGER_AC_CONNECTED = "charger_ac_connected";
  constexpr const char *CHARGER_DC_CONNECTED = "charger_dc_connected";
  constexpr const char *CHARGER_POWER = "charger_power";
  constexpr const char *CHARGER_VOLTAGE = "charger_voltage";
  constexpr const char *CHARGER_CURRENT = "charger_current";
  constexpr const char *CHARGED_SESSION_ENERGY = "charged_session_energy";
  constexpr const char *CUMULATIVE_ENERGY_CHARGED = "cumulative_energy_charged";

  // Driving & drivetrain
  constexpr const char *IGNITION_ON = "ignition_on";
  constexpr const char *SPEED = "speed";
  constexpr const char *ODOMETER = "odometer";
  constexpr const char *TRIP_DISTANCE = "trip_distance";
  constexpr const char *AVG_SPEED = "avg_speed";
  constexpr const char *CONSUMPTION = "consumption";
  constexpr const char *DISCHARGED_SESSION_ENERGY = "discharged_session_energy";
  constexpr const char *CUMULATIVE_ENERGY_DISCHARGED = "cumulative_energy_discharged";
  constexpr const char *MOTOR1_RPM = "motor1_rpm";
  constexpr const char *MOTOR2_RPM = "motor2_rpm";
  constexpr const char *MOTOR1_TORQUE = "motor1_torque";
  constexpr const char *MOTOR2_TORQUE = "motor2_torque";
  constexpr const char *MOTOR_TEMP = "motor_temp";
  constexpr const char *INVERTER_TEMP = "inverter_temp";

  // Tires
  constexpr const char *TIRE_PRESSURE_FL = "tire_pressure_fl";
  constexpr const char *TIRE_PRESSURE_FR = "tire_pressure_fr";
  constexpr const char *TIRE_PRESSURE_RL = "tire_pressure_rl";
  constexpr const char *TIRE_PRESSURE_RR = "tire_pressure_rr";

  // Environment
  constexpr const char *OUTDOOR_TEMP = "outdoor_temp";

  // GPS
  constexpr const char *GPS_LAT = "gps_lat";
  constexpr const char *GPS_LON = "gps_lon";
  constexpr const char *GPS_SPEED = "gps_speed";
  constexpr const char *GPS_ALT = "gps_alt";
  constexpr const char *GPS_HEADING = "gps_heading";

  // Diagnostics / heartbeat
  constexpr const char *STATUS = "status";
  constexpr const char *CAR_CONNECTED = "car_connected";
  constexpr const char *WIFI_RSSI = "wifi_rssi";
  constexpr const char *UPTIME = "uptime";
  // Per-subsystem diagnostic status (short text), published on the periodic
  // heartbeat only. comm covers the active car link (BLE or CAN).
  constexpr const char *DIAG_COMM = "diag_comm";
  constexpr const char *DIAG_WIFI = "diag_wifi";
  constexpr const char *DIAG_MODEM = "diag_modem";
  constexpr const char *DIAG_SDCARD = "diag_sdcard";
} // namespace ha
