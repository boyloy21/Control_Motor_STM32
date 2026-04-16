# 🤖 Control Motor with STM32

<p align="center">
  <img src="https://img.shields.io/badge/STM32-HAL_Library-blue?style=for-the-badge&logo=stmicroelectronics&logoColor=white"/>
  <img src="https://img.shields.io/badge/Language-C-lightgrey?style=for-the-badge&logo=c&logoColor=white"/>
  <img src="https://img.shields.io/badge/IDE-STM32CubeIDE-green?style=for-the-badge"/>
  <img src="https://img.shields.io/badge/License-MIT-yellow?style=for-the-badge"/>
</p>

<p align="center">
  A comprehensive collection of motor control implementations for STM32 microcontrollers — covering <strong>DC Motor</strong>, <strong>Brushless Motor (BLDC)</strong>, <strong>Stepper Motor</strong>, and <strong>Servo Motor</strong> with full example projects.
</p>

---

## 📋 Table of Contents

- [Overview](#-overview)
- [Repository Structure](#-repository-structure)
- [Hardware Requirements](#-hardware-requirements)
- [Motor Types](#-motor-types)
  - [DC Motor](#1--dc-motor)
  - [Brushless Motor (BLDC)](#2--brushless-motor-bldc)
  - [Stepper Motor](#3--stepper-motor)
  - [Servo Motor](#4--servo-motor)
- [Getting Started](#-getting-started)
- [PWM Configuration Reference](#-pwm-configuration-reference)
- [PID & CAN Bus Control](#-pid--can-bus-control)
- [Wiring Diagrams](#-wiring-diagrams)
- [Contributing](#-contributing)
- [License](#-license)

---

## 🔍 Overview

This repository provides STM32-based motor control firmware written in **C** using the **STM32 HAL (Hardware Abstraction Layer)** library. It is designed for students, engineers, and hobbyists who want to learn and implement motor control techniques on the STM32 platform.

| Motor Type     | Control Method         | Interface      | Folder             |
|----------------|------------------------|----------------|--------------------|
| DC Motor       | PWM + PID + CAN Bus    | Timer / CAN    | `Motor_PID_CAN/`   |
| Brushless (BLDC)| PWM (ESC signal)      | Timer PWM      | `Brushless/`       |
| Stepper Motor  | Step/Dir Pulses        | GPIO / Timer   | `stepper_motor/`   |
| Servo Motor    | PWM (50Hz)             | Timer PWM      | `Servo/`           |

---

## 📁 Repository Structure

```
Control_Motor_STM32/
│
├── Brushless/              # Brushless DC motor (BLDC) via ESC
│   ├── Core/
│   │   ├── Src/
│   │   └── Inc/
│   └── ...
│
├── Motor_PID_CAN/          # DC motor with PID control over CAN Bus
│   ├── Core/
│   │   ├── Src/
│   │   └── Inc/
│   └── ...
│
├── Servo/                  # Servo motor PWM control
│   ├── Core/
│   │   ├── Src/
│   │   └── Inc/
│   └── ...
│
├── stepper_motor/          # Stepper motor step/direction control
│   ├── Core/
│   │   ├── Src/
│   │   └── Inc/
│   └── ...
│
├── Stepper_Motor.pdf       # Stepper motor reference document
└── README.md
```

---

## 🔧 Hardware Requirements

### Microcontroller
- **STM32F4xx** series (e.g., STM32F401, STM32F407) — recommended
- Or any STM32 series with Timer PWM and CAN peripheral support

### Software Tools
- [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html) (v1.10+)
- STM32CubeMX (for `.ioc` pin configuration)
- STM32 HAL Drivers (included via CubeMX)
- ST-Link V2 or JTAG debugger for flashing

### Common Components

| Component | Description |
|-----------|-------------|
| L298N / L293D | H-Bridge driver for DC motor |
| ESC (Electronic Speed Controller) | For BLDC motor |
| A4988 / DRV8825 | Stepper motor driver |
| Servo Motor (e.g., SG90, MG996R) | Directly from PWM pin |
| CAN Transceiver (e.g., TJA1050) | For CAN bus communication |
| External Power Supply (5V–24V) | Depending on motor voltage rating |

---

## 🚀 Motor Types

---

### 1. 🔵 DC Motor

**Folder:** `Motor_PID_CAN/`

A **DC Motor** is the simplest type of electric motor. It converts DC electrical energy into mechanical rotation. Speed and direction are controlled using **PWM signals** sent to an H-Bridge driver (e.g., L298N).

#### Features in This Project
- PWM-based speed control using STM32 Timer
- Direction control via GPIO pins (IN1, IN2)
- **PID Controller** for closed-loop speed regulation
- **CAN Bus** communication for sending/receiving motor commands

#### How It Works

```
STM32 → PWM (TIMx_CHx) → L298N (ENA) → DC Motor
STM32 → GPIO (IN1/IN2)  → L298N       → Direction
Encoder → GPIO/TIM (Encoder Mode)     → Speed Feedback → PID
CAN Transceiver ↔ STM32 CAN Peripheral ↔ CAN Bus
```

#### Key Code Snippet

```c
// Set motor speed via PWM duty cycle (0–999)
__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, speed);

// Set motor direction
HAL_GPIO_WritePin(GPIOA, IN1_Pin, GPIO_PIN_SET);
HAL_GPIO_WritePin(GPIOA, IN2_Pin, GPIO_PIN_RESET);
```

#### PID Speed Control

```c
// PID calculation
float error = setpoint - current_speed;
integral   += error * dt;
derivative  = (error - prev_error) / dt;
output      = Kp * error + Ki * integral + Kd * derivative;
prev_error  = error;
```

---

### 2. 🟢 Brushless Motor (BLDC)

**Folder:** `Brushless/`

A **Brushless DC Motor (BLDC)** uses electronic commutation instead of mechanical brushes, making it more efficient and durable. In hobby and drone applications, a **ESC (Electronic Speed Controller)** handles the commutation logic and is driven by a **1–2ms PWM signal** at **50Hz**.

#### Features in This Project
- ESC calibration on startup
- Speed control via 50Hz PWM (1ms = min throttle, 2ms = full throttle)
- Uses STM32 HAL Timer in PWM mode

#### How It Works

```
STM32 Timer (50Hz PWM) → ESC Signal Wire → ESC → BLDC Motor
                         (1000µs – 2000µs pulse width)
```

#### ESC PWM Signal

| Pulse Width | Throttle |
|-------------|----------|
| 1000 µs     | 0% (stopped) |
| 1500 µs     | 50% |
| 2000 µs     | 100% (full speed) |

#### Key Code Snippet

```c
// Timer configured for 50Hz (Period = 20000µs)
// Pulse in microseconds mapped to timer compare value

// Arm ESC at minimum throttle
__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 1000); // 1ms pulse
HAL_Delay(2000); // Wait for ESC to arm

// Set 50% throttle
__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 1500); // 1.5ms pulse
```

> ⚠️ **Safety Warning:** Always arm the ESC with minimum throttle before increasing speed. Keep propellers or load detached during initial testing.

---

### 3. 🟡 Stepper Motor

**Folder:** `stepper_motor/`  
**Reference:** `Stepper_Motor.pdf`

A **Stepper Motor** moves in discrete angular steps, making it ideal for precise positioning (3D printers, CNC machines, robotic arms). It is controlled by sending step pulses and a direction signal to a stepper driver (e.g., A4988 or DRV8825).

#### Features in This Project
- Full-step and microstepping support
- Direction control (CW / CCW)
- Configurable speed via step delay
- GPIO-based STEP/DIR control

#### How It Works

```
STM32 GPIO → STEP Pin (pulse per step)
STM32 GPIO → DIR  Pin (direction: HIGH=CW, LOW=CCW)
STM32 GPIO → EN   Pin (enable driver: LOW=enabled)
```

#### Typical Step Angles

| Motor Type | Step Angle | Steps/Revolution |
|------------|------------|-----------------|
| 1.8° motor | 1.8°       | 200 steps       |
| 0.9° motor | 0.9°       | 400 steps       |
| With 1/16 microstepping | 0.1125° | 3200 steps |

#### Key Code Snippet

```c
// Rotate 200 steps (1 full revolution for 1.8° motor)
void StepMotor_Rotate(uint32_t steps, GPIO_PinState direction) {
    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, direction);
    for (uint32_t i = 0; i < steps; i++) {
        HAL_GPIO_WritePin(STEP_GPIO_Port, STEP_Pin, GPIO_PIN_SET);
        HAL_Delay(1); // Step pulse width
        HAL_GPIO_WritePin(STEP_GPIO_Port, STEP_Pin, GPIO_PIN_RESET);
        HAL_Delay(1); // Step delay (controls speed)
    }
}

// Usage: Rotate 200 steps clockwise
StepMotor_Rotate(200, GPIO_PIN_SET);
```

---

### 4. 🔴 Servo Motor

**Folder:** `Servo/`

A **Servo Motor** is a closed-loop actuator used for precise angular positioning (0°–180° typical range). It is controlled with a **50Hz PWM signal** where pulse width determines the target angle.

#### Features in This Project
- Angle control from 0° to 180°
- Smooth sweep function
- Uses STM32 HAL Timer PWM

#### How It Works

```
STM32 Timer (50Hz PWM) → Servo Signal Pin (Orange/Yellow wire)
Pulse width 0.5ms–2.5ms → maps to 0°–180°
```

#### PWM Pulse to Angle Mapping

| Pulse Width | Angle |
|-------------|-------|
| 500 µs (0.5ms) | 0°  |
| 1500 µs (1.5ms) | 90° |
| 2500 µs (2.5ms) | 180° |

#### Key Code Snippet

```c
// Set servo angle (0 to 180 degrees)
void Servo_SetAngle(uint8_t angle) {
    // Map angle (0-180) to pulse width (500-2500 µs)
    uint32_t pulse = 500 + ((uint32_t)angle * 2000 / 180);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pulse);
}

// Sweep from 0° to 180°
void Servo_Sweep(void) {
    for (uint8_t angle = 0; angle <= 180; angle += 5) {
        Servo_SetAngle(angle);
        HAL_Delay(20);
    }
}
```

---

## 🚀 Getting Started

### 1. Clone the Repository

```bash
git clone https://github.com/boyloy21/Control_Motor_STM32.git
cd Control_Motor_STM32
```

### 2. Open in STM32CubeIDE

1. Launch **STM32CubeIDE**
2. Go to `File` → `Open Projects from File System`
3. Browse to the specific motor folder (e.g., `Servo/`)
4. Click **Finish** to import

### 3. Configure for Your Board

- Open the `.ioc` file to review or modify pin assignments in **STM32CubeMX**
- Regenerate code if you change the MCU or pin configuration
- Adjust timer `Prescaler` and `Period` values in `main.c` to match your clock speed

### 4. Build and Flash

1. Click **Build** (🔨) in STM32CubeIDE
2. Connect your STM32 board via **ST-Link**
3. Click **Debug / Run** to flash the firmware

---

## 📐 PWM Configuration Reference

For a 72 MHz APB1 clock (common on STM32F1/F4):

### 50Hz PWM (Servo & BLDC ESC)
```
Prescaler  = 71    (divides 72MHz → 1MHz tick)
Period     = 19999 (1MHz / 20000 = 50Hz)
Resolution = 1µs per tick
```

### Fast PWM for DC Motor (e.g., 10kHz)
```
Prescaler = 0
Period    = 7199   (72MHz / 7200 = 10kHz)
```

---

## 📡 PID & CAN Bus Control

The `Motor_PID_CAN/` project demonstrates **closed-loop DC motor speed control** communicated over a **CAN Bus** network.

### CAN Bus Setup
- **Baud Rate:** 1 Mbps (configurable)
- **CAN Transceiver:** TJA1050 or MCP2551
- Wiring: `CAN_TX (PA12)` → Transceiver CANH/CANL → Target Node

### CAN Message Format (Example)

| Byte | Description        |
|------|--------------------|
| 0    | Command ID         |
| 1–2  | Target Speed (RPM) |
| 3–4  | PID Setpoint       |
| 5    | Direction (0/1)    |

### PID Tuning Tips

| Parameter | Effect |
|-----------|--------|
| Increase **Kp** | Faster response, may overshoot |
| Increase **Ki** | Eliminates steady-state error |
| Increase **Kd** | Reduces overshoot, adds damping |

Start with `Kp = 1.0, Ki = 0.0, Kd = 0.0` and tune incrementally.

---

## 🔌 Wiring Diagrams

### DC Motor (L298N)

```
STM32          L298N          DC Motor
------         -----          --------
PA0 (PWM) --→ ENA
PA1 (GPIO)--→ IN1
PA2 (GPIO)--→ IN2
                OUT1 --------→ Motor (+)
                OUT2 --------→ Motor (-)
3.3V      --→ Logic VCC
GND       --→ GND
               12V  ←-------- External Power
```

### Servo Motor

```
STM32              Servo
------             -----
PA8 (TIM1_CH1)--→ Signal (Orange)
5V             --→ VCC    (Red)
GND            --→ GND    (Brown)
```

### Stepper Motor (A4988)

```
STM32              A4988         Stepper Motor
------             -----         -------------
PA4 (STEP)  --→  STEP
PA5 (DIR)   --→  DIR
PA6 (EN)    --→  EN
                  1A,1B --------→ Coil A
                  2A,2B --------→ Coil B
3.3V        --→  VDD (logic)
GND         --→  GND
                  VMOT ←-------- 8V–35V Power
```

### Brushless Motor (ESC)

```
STM32                 ESC             BLDC Motor
------                ---             ----------
PA8 (TIM1_CH1) --→  Signal
5V             --→  VCC (BEC output, optional)
GND            --→  GND (shared)
                      3-phase wires → Motor phases
                      Battery ←----- LiPo / Power Supply
```

---

## 🤝 Contributing

Contributions are welcome! Here's how you can help:

1. **Fork** the repository
2. Create a new branch: `git checkout -b feature/your-feature`
3. **Commit** your changes: `git commit -m "Add: your feature description"`
4. **Push** to your branch: `git push origin feature/your-feature`
5. Open a **Pull Request**

### Contribution Ideas
- Add encoder feedback example for DC motor
- Add BLDC field-oriented control (FOC)
- Add FreeRTOS-based multi-motor control
- Add more detailed wiring diagrams or schematics

---

## 📄 License

This project is licensed under the **MIT License** — feel free to use, modify, and distribute.

---

## 👤 Author

**Yin Chheanyun**  
GitHub: [@boyloy21](https://github.com/boyloy21)

---

<p align="center">
  ⭐ If this project helped you, please consider giving it a <strong>star</strong>!
</p>
