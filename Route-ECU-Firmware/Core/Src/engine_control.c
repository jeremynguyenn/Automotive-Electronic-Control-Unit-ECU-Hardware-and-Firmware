/*
 * engine_controll.c
 *
 */

/*
 *Algorithmic Implementation of Speed-Density
 *The Speed-Density algorithm is executed by the ECU every engine cycle to determine the mass of fuel to be injected. The process can be summarized in the following steps:
 *Sensor Sampling: The ECU reads the instantaneous values from the MAP, IAT, and RPM sensors.
 *VE Table Lookup: Using the current MAP and RPM values as coordinates, the ECU looks up the 2D VE table. If the exact operating point (e.g., 2550 RPM and 85 kPa) does not match a grid point in the table, the ECU uses a bilinear interpolation algorithm (detailed in Section 6.1) to calculate the precise VE value between the four surrounding grid points.
 *Effective Volume Calculation (Veffective): The effective volume of admitted air is calculated by multiplying the displacement volume of a single cylinder by the obtained VE value.
 *
 *Veffective = CylinderDisplacement * VE(MAP,RPM)
 *Air Mass Calculation (mar): With the pressure (p from MAP), temperature (T from IAT, converted to Kelvin)
 *and effective volume (V effective), the ECU applies the derived form of the Ideal Gas Law to calculate the mass of air that filled the cylinder during the intake cycle.
 */

/*
 * engine_control.c
 *
 */

//In this script several values from g_engineData are missing
#include <math.h>
#include <string.h>
#include "engine_control.h"
#include "ecu_config.h"
#include "cmsis_os.h"

// --- Definition of Global Variables ---
extern EcuConfig_t g_ecuConfig;
extern EngineData_t g_engineData;
extern EngineState_t g_engineState;

// Semaphore to synchronize the task with the CKP ISR
osSemaphoreId engineSyncSemaphoreHandle;

// ----------------- TIME UTILITIES (based on RTOS) -----------------
static inline uint32_t now_us(void) {
	// Convert scheduler ticks to microseconds (avoids needing HAL_TIM when not available)
	uint32_t ticks = osKernelSysTick();
	uint32_t hz = osKernelSysTickFrequency;  // ticks/sec
	if (hz == 0)
		hz = 1000;
	// careful with overflow: for RPM calculations we use short delta between teeth
	return (uint32_t) ((((uint64_t) ticks) * 1000000ULL) / (uint64_t) hz);
}

// ----------------- CLAMPS AND INTERPOLATIONS -----------------
static inline float clampf(float v, float lo, float hi) {
	return (v < lo) ? lo : (v > hi) ? hi : v;
}

// Binary search “lower index” (returns i such that axis[i] <= x < axis[i+1])
static int lower_index(const float *axis, int len, float x) {
	if (x <= axis[0])
		return 0;
	if (x >= axis[len - 2])
		return len - 2; // ensures i+1 is valid
	int lo = 0, hi = len - 1;
	while (hi - lo > 1) {
		int mid = (lo + hi) >> 1;
		if (x >= axis[mid])
			lo = mid;
		else
			hi = mid;
	}
	return lo;
}

// 1D Interpolation
static float interp1d(const float *x_axis, const float *y_vals, int len,
		float x) {
	if (len < 2)
		return y_vals[0];
	int i = lower_index(x_axis, len, x);
	float x0 = x_axis[i], x1 = x_axis[i + 1];
	float y0 = y_vals[i], y1 = y_vals[i + 1];
	float t = (x1 != x0) ? (x - x0) / (x1 - x0) : 0.0f;
	return y0 + t * (y1 - y0);
}

// ----------------- 2D BILINEAR INTERPOLATION -----------------
static float interpolate_bilinear(const Table2D_t *table, float rpm,
		float map_kpa) {
	const int nx = table->rows;
	const int ny = table->cols;

	// Clamp
	float xr = clampf(rpm, table->x_axis[0], table->x_axis[nx - 1]);
	float ym = clampf(map_kpa, table->y_axis[0], table->y_axis[ny - 1]);

	int i = lower_index(table->x_axis, nx, xr);
	int j = lower_index(table->y_axis, ny, ym);

	float x0 = table->x_axis[i], x1 = table->x_axis[i + 1];
	float y0 = table->y_axis[j], y1 = table->y_axis[j + 1];

	// linear access: [i * cols + j]
	float q11 = table->values[i * ny + j];
	float q21 = table->values[(i + 1) * ny + j];
	float q12 = table->values[i * ny + (j + 1)];
	float q22 = table->values[(i + 1) * ny + (j + 1)];

	float fx = (x1 != x0) ? (xr - x0) / (x1 - x0) : 0.0f;
	float fy = (y1 != y0) ? (ym - y0) / (y1 - y0) : 0.0f;

	float v1 = q11 * (1.0f - fx) + q21 * fx;
	float v2 = q12 * (1.0f - fx) + q22 * fx;
	return v1 * (1.0f - fy) + v2 * fy;
}

// ----------------- AIR MASS CALCULATION (SD) -----------------
static float calculate_air_mass_sd(void) {
	// Charge temperature mixture (optional): here we use pure IAT to simplify
	float charge_temp_k = g_engineData.iat_celsius + 273.15f;

	float ve_percent = interpolate_bilinear(&g_ecuConfig.table_ve,
			g_engineState.rpm, g_engineData.map_kpa);

	float v_cyl_l = g_ecuConfig.engine_displacement_l
			/ (float) g_ecuConfig.num_...(truncated 3496 characters)....table_injector_deadtime.len, vbat);
}

// ----------------- DFCO -----------------
static int DFCO_active_conditions(void) {
	if (g_ecuConfig.num_cylinders > 4)
		return 0;
	if (g_engineData.tps_percent > g_ecuConfig.dfco.max_tps_percent)
		return 0;
	if (g_engineData.map_kpa > g_ecuConfig.dfco.max_map_kpa)
		return 0;
	if (g_engineState.rpm < g_ecuConfig.dfco.min_rpm)
		return 0;
	if (g_engineData.ect_celsius < g_ecuConfig.dfco.min_ect_c)
		return 0;
	return 1;
}

void EngineControl_Init(void) {
	osSemaphoreDef(engineSync);
	engineSyncSemaphoreHandle = osSemaphoreCreate(osSemaphore(engineSync), 1);
	osSemaphoreWait(engineSyncSemaphoreHandle, 0);

	// Initial Baro (KOEO) – if there is a specific routine, call it on boot and save in g_engineState.baro_kpa
	if (g_engineState.baro_kpa < 10.0f)
		g_engineState.baro_kpa = g_ecuConfig.baro_ref_kpa;

	o2_controller_init();
}

void EngineControl_RunCalculations(void) {
	// 1. Air mass per cylinder (SD)
	g_engineState.air_mass_per_cyl_g = calculate_air_mass_sd();

	// 2. Target AFR (2D table RPM x MAP)
	float afr_target = interpolate_bilinear(&g_ecuConfig.table_afr_target,
			g_engineState.rpm, g_engineData.map_kpa);
	afr_target = clampf(afr_target, g_ecuConfig.afr_min, g_ecuConfig.afr_max);

	// 3. Fuel mass per cylinder
	float fuel_mass_g = g_engineState.air_mass_per_cyl_g / afr_target;

	// 4. Base PW (s)
	float inj_flow_gps = fmaxf(g_ecuConfig.injector_flow_gps, 0.0001f);
	float pw_base_s = fuel_mass_g / inj_flow_gps;

	// 5. Enrichments (Gamma)
	float gamma = calculate_enrichments_multipliers();
	float pw_enriched_s = pw_base_s * gamma;

	// 6. Closed loop O2 (STFT/LTFT)
	float pw_closedloop_s = pw_enriched_s;
	static uint32_t last_us = 0;
	uint32_t now = now_us();
	float dt_s =
			(last_us == 0) ? (1.0f / 1000.0f) : ((now - last_us) / 1000000.0f);
	last_us = now;

	if (closed_loop_enabled() && g_engineData.lambda > 0.1f) {
		float afr_meas = g_engineData.lambda * g_ecuConfig.afr_stoich;
		o2_controller_update(afr_meas, afr_target, dt_s);
		float clo_gain = 1.0f + g_o2.stft + g_o2.ltft;     // fraction
		clo_gain = clampf(clo_gain, g_ecuConfig.clo.min_gain,
				g_ecuConfig.clo.max_gain);
		pw_closedloop_s *= clo_gain;
	}

	// 7. Deadtime (ms -> s)
	float deadtime_ms = injector_deadtime_ms(g_engineData.battery_voltage);
	float pw_final_s = pw_closedloop_s + (deadtime_ms * 0.001f);

	// 8. DFCO
	if (DFCO_active_conditions() == 1) {
		//pw_final_s = 0.0f;

		// Safety limits
		pw_final_s = clampf(pw_final_s, g_ecuConfig.pw_min_s,
				g_ecuConfig.pw_max_s);

		g_engineState.final_fuel_pw_ms = pw_final_s * 1000.0f;

		// 10. Schedule Injection (actuators module)
		Injector_SchedulePulse(g_engineState.current_cylinder,
				g_engineState.final_fuel_pw_ms);

		// 11. Ignition (example: look up advance and schedule)
		float ign_deg = interpolate_bilinear(&g_ecuConfig.table_ign_advance,
				g_engineState.rpm, g_engineData.map_kpa);
		ign_deg = clampf(ign_deg, g_ecuConfig.ign_min_deg,
				g_ecuConfig.ign_max_deg);
		float dwell_ms = 3.2f;
		Ignition_ScheduleSpark(g_engineState.current_cylinder, ign_deg,
				dwell_ms);
	}
}

// This is the function that the CKP Interrupt (ISR) will call
void EngineControl_CrankEventCallback(void) {
	// **FAST**: calculate RPM/angle and wake up the task

	static uint32_t last_edge_us = 0;
	uint32_t t_us = now_us();

	// RPM calculation using time between teeth
	if (last_edge_us != 0) {
		uint32_t dt_us = t_us - last_edge_us;
		if (dt_us > 0) {
			// useful teeth per revolution (excluding gaps/missing): define in g_ecuConfig.ckp.teeth_per_rev
			float teeth_per_rev = (float) g_ecuConfig.ckp.teeth_per_rev;
			float rev_per_s = (1000000.0f / (float) dt_us) / teeth_per_rev;
			g_engineState.rpm = (uint16_t) clampf(60.0f * rev_per_s, 0.0f,
					(float) g_ecuConfig.rpm_max);
		}
	}
	last_edge_us = t_us;

	// Update engine angle (increment per tooth)
	float deg_per_tooth = 360.0f / (float) g_ecuConfig.ckp.teeth_per_rev;
	g_engineState.engine_angle_deg += deg_per_tooth;
	if (g_engineState.engine_angle_deg >= 720.0f) { // 4-stroke
		g_engineState.engine_angle_deg -= 720.0f;
		// advance current cylinder (sequential)
		g_engineState.current_cylinder = (g_engineState.current_cylinder
				% g_ecuConfig.num_cylinders) + 1;
	}

	// 2. Wake up the main task for calculations outside the ISR
	osSemaphoreRelease(engineSyncSemaphoreHandle);
}

// The RTOS task that orchestrates the control
void Task_EngineControl(void const *argument) {
	EngineControl_Init();

	for (;;) {
		if (osSemaphoreWait(engineSyncSemaphoreHandle, osWaitForever) == osOK) {
			EngineControl_RunCalculations();
		}
	}
}