# TECHNICAL DOCUMENTATION
## Automotive Electronic Control Unit (ECU)

[![ECU-Render](https://i.ibb.co/vrg7kG9/ECU-Render.png)](https://i.ibb.co/vrg7kG9/ECU-Render.png)

---

**Version:** 1.0  
**Date:** September 2025  
**Classification:** Technical Document  
**Status:** In Development  

---

## 1. GENERAL INFORMATION

### 1.1 Application
ECU intended for the control of internal combustion engines with electronic injection system, with capacity for:
- 4 cylinders/injectors
- CAN bus communication
- Serial communication
- Data and map storage

---

## 2. TECHNICAL SPECIFICATIONS

### 2.1 Electrical Specifications

| Parameter | Minimum | Typical | Maximum | Unit |
|-----------|---------|---------|---------|----------|
| Input Voltage | 9 | 12 | 18.81 | V |
| Current (Standby) | - | To Be Determined | To Be Determined | mA |
| Current (Operation) | - | To Be Determined | To Be Determined | mA |
| Current per Injector | - | To Be Determined | To Be Determined | A |
| Logic Voltage (MCU) | - | 3.3 | - | V |
| Peripheral Voltage | - | 5.0 | - | V |

The peripheral voltage is a voltage derived from the main regulator, but with extra filters for sensor power supply.

### 2.2 General Specifications

| Characteristic | Value | Unit |
|----------------|--------|----------|
| MCU Frequency | 100 | MHz |
| Time Resolution | To Be Determined | µs |
| CAN Bus Rate | To Be Determined | kbps |
| Injection Channels | 4 | - |
| Ignition Channels | 4 | - |
| Flash Memory | TBD | KB |
| RAM Memory | TBD | KB |

---

## 3. HARDWARE ARCHITECTURE

### 3.1 Main Microcontroller
- **Family**: STM32H743VIT6 (ARM Cortex-M)
- **Main Clock**: 16MHz (external crystal) x2
- **RTC Clock**: 32.768kHz
- **Interfaces**: SPI, I2C, UART, CAN, GPIO

### 3.2 Clock System
```
Oscilador Principal (16MHz) ──▶ PLL ──▶ System Clock (100MHz)
                                  │
RTC Crystal (32.768kHz) ─────────┴──▶ RTC Clock
```


### 3.3 Memory Map
- **Flash**: Program code
- **FRAM**: Large data with fast access
- **SD Card**: Data log and Maps

### 3.4 Injection System
- **Actuator**: TIP142
- **Driver**: LM1949 (Peak-and-hold mode)
- **Indicator**: Individual indicator LEDs for each injection channel

#### 3.4.2 Injection Channels
| Channel |
|-------|
| INJ 1 |
| INJ 2 |
| INJ 3 |
| INJ 4 |

[![INJECTOR](https://i.ibb.co/99wgFJhB/INJECTOR.png)](https://i.ibb.co/99wgFJhB/INJECTOR.png)

### 3.5 Ignition System
- **Actuator**: ISL9V5036P3-F085
- **Driver**: TC4424CPA (Controls two ignitions)
- **Peak Current**: 3A

#### 3.5.2 Ignition Channels
| Channel |
|------|
| IG 1 |
| IG 2 |
| IG 3 |
| IG 4 |

[![IGNITION](https://i.ibb.co/ycDnkN8f/IGNITION.png)](https://i.ibb.co/ycDnkN8f/IGNITION.png)

### 3.6 Position Sensors
- **CKP**: Crankshaft Position
- **CMP**: Camshaft Position (If applicable)
- **Input Types**: Variable Reluctance and Hall
- **VR Signal Conditioner**: MAX9924

[![CKP](https://i.ibb.co/mFYfzqwf/CKP.png)](https://i.ibb.co/mFYfzqwf/CKP.png)

### 3.7 MAP Sensor
- **Sensor**: MPXA4250AC6U
- **Working Range**: 20 to 250 kPa
- **Calculation Ratio**: VOUT = VCC x (P × 0.004 – 0.04)

[![MAP](https://i.ibb.co/p6Ym96gr/MAP.png)](https://i.ibb.co/p6Ym96gr/MAP.png)

### 3.8 LAMBDA Sensor
- **Controller**: CJ125
- **Heating Control**: Yes
- **Sensors**: LSU4.X

[![LAMBDA](https://i.ibb.co/d0LBV45P/LAMBDA.png)](https://i.ibb.co/d0LBV45P/LAMBDA.png)

### 3.9 Analog Sensors
- **Input Converter (map)**: Input Voltage at MCU = 3.205V When V_Sensor = 5V

#### 3.9.2 Low Power Sensors

| Input | Maximum Voltage | TVS | Input Type |
|----------------|--------|----------|----------|
| Input 1 | 5V | Yes | Analog |
| Input 2 | 5V | Yes | Analog |
| Input 3 | 5V | Yes | Analog |
| Input 4 | 5V | Yes | Analog |
| Input 5 | 5V | Yes | Analog |
| Input 6 | 5V | Yes | Analog |
| Input 7 | 5V | Yes | Analog |
| Input 8 | 5V | Yes | Analog |
| Input 9 | 5V | Yes | Analog |
| Input 10 | 5V | Yes | Analog |
| Input 11 | 5V | Yes | Analog |
| Input 12 | 5V | Yes | Analog |

[![LOW-POWER-SENSOR](https://i.ibb.co/27B8pFJF/LOW-POWER-SENSOR.png)](https://i.ibb.co/27B8pFJF/LOW-POWER-SENSOR.png)

#### 3.9.3 High Power Sensors

| Input | Maximum Voltage | Optocoupled | Input Type |
|----------------|--------|----------|----------|
| Input 1 | 12V | Yes | Analog |
| Input 2 | 12V | Yes | Analog |
| Input 3 | 12V | Yes | Analog |
| Input 4 | 12V | Yes | Analog |

[![HIGH-POWER-SENSORS](https://i.ibb.co/LzSqFQfp/HIGH-POWER-SENSORS.png)](https://i.ibb.co/LzSqFQfp/HIGH-POWER-SENSORS.png)

### 3.10 Outputs

| Output | Maximum Voltage | PWM |
|----------------|--------|----------|
| Output 1 | 5V | Yes |
| Output 2 | 5V | Yes |
| Output 3 | 5V | Yes |
| Output 4 | 5V | Yes |

[![PWM](https://i.ibb.co/xtqXdg2R/PWM.png)](https://i.ibb.co/xtqXdg2R/PWM.png)

---

## 4. SUBSYSTEMS

### 4.1 Power Supply System

#### 4.1.1 DC-DC Conversion
- **Input**: 12V automotive (9 to 18.81V)
- **Outputs**: 
  - 5V 2A (peripherals)
  - 3.3V 500mA (MCU and logic)

## 5. INTERFACES AND CONNECTORS

### 5.1 Main Connector (J1)
- **Type**: TE 796739-2
- **Function**: Main vehicle interface
- **Signals**: Power, CAN, control signals

### 5.2 Programming Headers

| Designator | Type | Function |
|------------|------|---------|
| J2 | TSM-104-01-L-MT | Debug/SWD |

### 5.3 SD Card Interface (J9)
- **Connector**: 5007620483
- **Protocol**: SPI
- **Voltage**: 3.3V
- **Capacity**: Up to 32GB

### 5.4 Main Connector Pinout

| Pin | Signal |
|-----|---------------|
| **A1** | INJ 1          |
| **A2** | INJ 4          |
| **A3** | INJ 3          |
| **A4** | INJ 2          |
| **B1** | Sensor_In_1    |
| **B2** | CANL           |
| **B3** | CANH           |
| **B4** | PWM_MCU_OUT_4  |
| **C1** | Sensor_In_2    |
| **C2** | 5V Sensor      |
| **C3** | 3.3V           |
| **C4** | PWM_MCU_OUT_3  |
| **D1** | Sensor_In_3    |
| **D2** | CMP_IN_1       |
| **D3** | CMP_HALL_IN    |
| **D4** | PWM_MCU_OUT_2  |
| **E1** | Sensor_In_4    |
| **E2** | CMP_IN_2       |
| **E3** | GND            |
| **E4** | PWM_MCU_OUT    |
| **F1** | Sensor_In_5    |
| **F2** | CKP_IN         |
| **F3** | CKP_HALL_IN    |
| **F4** | CJ125_UP       |
| **G1** | Sensor_In_6    |
| **G2** | CKP_IN_1       |
| **G3** | CJ125_IA       |
| **G4** | CJ125_UA       |
| **H1** | Sensor_In_7    |
| **H2** | Sensor_In_10   |
| **H3** | CJ125_US       |
| **H4** | CJ125_IP       |
| **J1** | Sensor_In_8    |
| **J2** | Sensor_In_11   |
| **J3** | CJ125_VM       |
| **J4** | CJ125_UN       |
| **K1** | Sensor_In_9    |
| **K2** | Sensor_In_12   |
| **K3** | MAP_OUT        |
| **K4** | Sensor_OPT_3   |
| **L1** | Sensor_OPT_2   |
| **L2** | Sensor_OPT_1   |
| **L3** | Sensor_OPT_0   |
| **L4** | Heater OUT     |
| **M1** | Ig 1           |
| **M2** | Ig 2           |
| **M3** | Ig 3           |
| **M4** | Ig 4           |