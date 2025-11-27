/*
 * ecu_config.h
 *
 * Created on: 21 de ago de 2025
 */

#ifndef INC_ECU_CONFIG_H_
#define INC_ECU_CONFIG_H_

#include <stdint.h>

// Enum for sensor types
typedef enum {
    SENSOR_TYPE_NONE = 0, SENSOR_TYPE_TPS, SENSOR_TYPE_CLT, SENSOR_TYPE_IAT,
    SENSOR_TYPE_MAP, SENSOR_TYPE_O2, SENSOR_TYPE_CKP, SENSOR_TYPE_CMP
} SensorType_e;

// Enum for position sensor types
typedef enum {
    SENSOR_INPUT_VR = 0, SENSOR_INPUT_HALL = 1
} PositionSensorType_e;

// Structure for 1D table
typedef struct {
    int len;             // Number of points in the table
    const float *x_axis; // Pointer to X axis (e.g., Temperature)
    const float *values; // Pointer to values (e.g., Multiplier)
} Table1D_t;

// Structure for 2D table
typedef struct {
    int rows;            // Number of rows (X axis)
    int cols;            // Number of columns (Y axis)
    const float *x_axis; // Pointer to X axis (e.g., RPM)
    const float *y_axis; // Pointer to Y axis (e.g., MAP)
    const float *values; // Pointer to value matrix (linear access: [i * cols + j])
} Table2D_t;

// --- SUB-STRUCTURES FOR CONFIGURATION ---

// Structure for trigger wheel config
typedef struct {
    uint16_t teeth_per_rev; // Teeth per revolution (excluding gap, e.g., 60)
    float    tdc_angle_after_gap_deg; // TDC angle after the gap
    int num_teeth_on_wheel;
} TriggerWheelConfig_t;

// Structure for closed loop O2 config
typedef struct {
    float kp;
    float ki;
    float min_ect_c;
    uint16_t min_rpm;
    uint16_t max_rpm;
    float min_map_kpa;
    float max_map_kpa;
    float stft_limit;  // Limit for Short Term Fuel Trim (e.g., 0.25 for +/- 25%)
    float ltft_limit;  // Limit for Long Term Fuel Trim
    float ltft_err_gate; // Error limit to allow LTFT integration

    float stft;
    float ltft;

    float min_gain;
    float max_gain;
} ClosedLoopO2Config_t;

// Structure for DFCO config
typedef struct {
    float max_tps_percent;
    float max_map_kpa;
    uint16_t min_rpm;
    float min_ect_c;
} DfcoConfig_t;


// --- MAIN CONFIGURATION STRUCTURE ---

// Main ECU configuration structure
typedef struct {
    uint32_t config_version;

    // --- Engine and Injectors Configuration ---
    uint8_t num_cylinders;
    float engine_displacement_l;
    float injector_flow_gps;
    float afr_stoich;
    float required_fuel_const;
    uint16_t rpm_max;

    // --- Sensors Mapping ---
    SensorType_e sensor_channel_map[12];
    PositionSensorType_e ckp_type;
    TriggerWheelConfig_t ckp;

    // --- Limits and Failsafes ---
    float afr_min, afr_max;
    float pw_min_s, pw_max_s;
    float ign_min_deg, ign_max_deg;
    float baro_ref_kpa; // Reference barometric pressure
    float gamma_min, gamma_max;

    // --- Control Modules ---
    ClosedLoopO2Config_t clo; // Configuration for Closed Loop (O2)
    DfcoConfig_t         dfco; // Configuration for Fuel Cutoff

    // --- Calibration Tables ---
    Table2D_t table_ve;
    Table2D_t table_afr_target;
    Table2D_t table_ign_advance; // Renamed from 'spark_advance' for consistency
    Table1D_t table_warmup_enrichment;
    Table1D_t table_accel_enrichment;
    Table1D_t table_injector_deadtime; // Renamed for consistency

} EcuConfig_t;


// --- Real-Time Data Structures ---
// Structure for engine data
typedef struct {
    float tps_percent;
    float ect_celsius;
    float iat_celsius;
    float map_kpa;
    float battery_voltage;
    float lambda;
} EngineData_t;

// Structure for engine state
typedef struct {
    uint32_t rpm;
    float    engine_angle_deg;
    uint8_t  current_cylinder;
    float    air_mass_per_cyl_g;
    float    final_fuel_pw_ms;
    float    final_spark_deg;
    int      is_in_dfco;
    float    baro_kpa; // Barometer measured at startup (KOEO)
} EngineState_t;


#endif /* INC_ECU_CONFIG_H_ */