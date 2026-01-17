# Aurora Tracker with STM32 Motor Control

**Embedded Systems Project - ECE5140**

A real-time position tracking and motor control system that integrates NDI Aurora electromagnetic tracking with STM32-based motor control for precise 2-DOF robotic arm manipulation.

---

## 📋 Table of Contents

- [Overview](#overview)
- [Demo](#demo)
- [System Architecture](#system-architecture)
- [Hardware Components](#hardware-components)
- [Features](#features)
- [Software Implementation](#software-implementation)
- [Control System](#control-system)
- [Getting Started](#getting-started)
- [Results](#results)
- [Future Improvements](#future-improvements)
- [References](#references)
- [Author](#author)

---

## 🎯 Overview

This project combines electromagnetic position tracking with closed-loop motor control to create a precise robotic positioning system. The system uses Aurora electromagnetic sensors to track objects in 3D space and translates that data into motor commands for a 2-DOF robotic arm controlled by an STM32 microcontroller.

### Key Capabilities

- **Real-time tracking** of two electromagnetic sensors at ~100 Hz
- **Inverse kinematics** calculations for motor positioning
- **PID control** for precise linear motor movement
- **Multi-threaded architecture** for responsive operation
- **UART communication** between PC and STM32
- **CSV data logging** for analysis and debugging
- **Visual feedback** via OpenMV H7 camera

---

## 🎬 Demo

| Robot Arm Movement | User Interface |
|-------------------|----------------|
| [Watch Demo](https://drive.google.com/file/d/1VKm69nz09LB0u3aUUcMf79Sfyy_caBB2/view?usp=sharing) | [Watch Demo](https://drive.google.com/file/d/18wPMM6UHqs1dqIHGXfHYBYghuFead4py/view?usp=sharing) |

---

## 🏗️ System Architecture

```
┌─────────────────────────┐
│  Aurora Field Generator │
└────────┬───────┬─────────┘
         │       │
    ┌────▼──┐ ┌─▼─────┐
    │Sensor1│ │Sensor2│
    └────┬──┘ └─┬─────┘
         │      │
    ┌────▼──────▼─────┐
    │   Host PC        │
    │ (C++ Control App)│
    └────────┬─────────┘
             │ UART
    ┌────────▼─────────┐
    │    STM32L4       │
    │  (PID Control)   │
    └────┬──────┬──────┘
         │      │
    ┌────▼──┐ ┌▼─────┐
    │Linear│ │Servo │
    │Motor │ │Motor │
    └──────┘ └──────┘
```

### System Flow

1. **Initialization**: Aurora tracker, STM32, and all threads are initialized
2. **Calibration**: System calibrates initial position reference
3. **Target Input**: User inputs target X, Y coordinates
4. **Control Loop**: 
   - Sensors continuously tracked at 100 Hz
   - Inverse kinematics calculates required motor angles
   - Commands sent to STM32 via UART
   - STM32 executes PID control at 100 Hz
5. **Data Logging**: All sensor data logged to CSV at 20 Hz

---

## 🔧 Hardware Components

| Component | Model | Purpose |
|-----------|-------|---------|
| **Position Tracker** | NDI Aurora | Electromagnetic tracking with 1mm accuracy |
| **Microcontroller** | STM32L4 | Motor control with PID feedback |
| **Linear Actuator** | GB37Y3530-12V-251R | Translational movement (0.25mm/rev) |
| **Servo Motor** | MG996R | Rotational control (45° - 135°) |
| **Camera** | OpenMV H7 | Visual feedback and monitoring |
| **Motor Driver** | L298N H-Bridge | Bidirectional DC motor control |

### STM32 Peripheral Configuration

- **TIM1**: PWM generation for DC motor (prescaler=1, ARR=999)
- **TIM2**: Encoder input for position feedback (32-bit counter)
- **TIM3**: Servo PWM at 50 Hz (prescaler=79, ARR=999)
- **USART2**: Serial communication at 9600 baud
- **GPIO PB4/PB5**: H-bridge direction control

---

## ✨ Features

### Multi-threaded PC Application

1. **Sensor Thread** (~100 Hz)
   - Continuous Aurora sensor polling
   - Data validation and filtering
   - Thread-safe shared memory updates

2. **CSV Logger Thread** (20 Hz)
   - Real-time data logging
   - Timestamped sensor positions
   - Analysis and debugging support

3. **Control Thread** (Every 5 seconds)
   - Inverse kinematics computation
   - Motor angle calculation
   - UART command transmission

### Robust Communication Protocol

Simple and reliable UART protocol:
```
PC → STM32: L:<linear_angle>,S:<servo_angle>\n
STM32 → PC: 1\n (completion acknowledgment)
```

Example: `L:-45.23,S:90.00\n`

---

## 💻 Software Implementation

### Inverse Kinematics

The system calculates motor angles using iterative trigonometry:

**Servo Angle Calculation:**
```
θ_servo = arccos((Δx + L·cos(θ_current)) / L)
```

**Linear Motor Calculation:**
```
θ_linear = -(Δy / 0.25mm) × 360°
```

Where:
- `L` = arm length (39 mm)
- `Δx`, `Δy` = target displacement
- `0.25mm` = lead screw pitch

### C++ Code Example

```cpp
// Iterative servo angle calculation
double cos_arg = (delta_x + SENSOR2PIVOT * 
    cos(g_controlData.current_servo_angle * PI / 180.0)) / SENSOR2PIVOT;
    
// Clamp to valid range
cos_arg = std::max(-1.0, std::min(1.0, cos_arg));

angle_servo = acos(cos_arg) * 180.0 / PI;

// Apply physical limits
angle_servo = std::max(SERVO_ANGLE_MIN, 
                      std::min(SERVO_ANGLE_MAX, angle_servo));

// Linear motor from Y displacement
double rev_to_rotate = delta_y / REV_MM;
angle_linear = -rev_to_rotate * 360.0;
```

### STM32 Command Parsing

```c
int Parse_Dual_Angle_Command(char* buffer, 
                             float* linear_angle, 
                             float* servo_angle) {
    int result = sscanf(buffer, "L:%f,S:%f", linear_angle, servo_angle);
    return (result == 2) ? 1 : 0;
}
```

---

## 🎮 Control System

### PID Controller Design

Implemented on STM32 for precise linear motor positioning:

| Parameter | Value | Function |
|-----------|-------|----------|
| **Kp** | 1.0 | Proportional gain - main correction |
| **Ki** | 0.5 | Integral gain - eliminates steady-state error |
| **Kd** | 0.1 | Derivative gain - dampens oscillations |
| **Sample Time** | 10 ms | Control loop frequency (100 Hz) |
| **Dead Zone** | ±10 counts | Position tolerance threshold |
| **Min PWM** | 15% | Minimum duty cycle to overcome friction |

### PID Implementation

```c
float PID_Compute(PID_Controller *pid, float setpoint, float measured) {
    float error = setpoint - measured;
    
    // Dead zone check
    if (fabs(error) < pid->dead_zone) {
        pid->integral = 0;
        return 0;
    }
    
    // Proportional term
    float p_term = pid->Kp * error;
    
    // Integral term with anti-windup
    pid->integral += error * pid->dt;
    pid->integral = fmax(fmin(pid->integral, pid->integral_max), 
                        -pid->integral_max);
    float i_term = pid->Ki * pid->integral;
    
    // Derivative term
    float derivative = (error - pid->prev_error) / pid->dt;
    float d_term = pid->Kd * derivative;
    pid->prev_error = error;
    
    // Compute output
    float output = p_term + i_term + d_term;
    
    // Clamp and apply minimum PWM threshold
    output = fmax(fmin(output, 100.0), -100.0);
    if (output != 0 && fabs(output) < pid->min_pwm_threshold)
        output = (output > 0) ? pid->min_pwm_threshold : -pid->min_pwm_threshold;
    
    return output;
}
```

### System Timing

- **Sensor Polling**: ~100 Hz
- **CSV Logging**: 20 Hz
- **Control Commands**: 0.2 Hz (every 5 seconds)
- **STM32 PID Loop**: 100 Hz

---

## 🚀 Getting Started

### Prerequisites

**Hardware:**
- NDI Aurora tracking system
- STM32L4 development board
- L298N H-bridge motor driver
- GB37Y3530-12V-251R linear motor with encoder
- MG996R servo motor
- OpenMV H7 camera (optional)
- 12V DC power supply

**Software:**
- STM32CubeIDE
- C++ compiler with C++11 support
- NDI API (v1.4.0 or later)
- Serial terminal software

### Installation

1. **Clone the repository**
   ```bash
   git clone https://github.com/yourusername/aurora-stm32-motor-control.git
   cd aurora-stm32-motor-control
   ```

2. **Build the PC application**
   ```bash
   cd pc-application
   make
   ```

3. **Flash STM32 firmware**
   - Open the STM32 project in STM32CubeIDE
   - Build and flash to the STM32L4 board

4. **Connect hardware**
   - Connect Aurora sensors
   - Wire motors to STM32 via L298N driver
   - Connect STM32 to PC via USB (USART2)

5. **Run the application**
   ```bash
   ./aurora_control
   ```

### Usage

1. Power on the Aurora field generator and wait for initialization
2. Place sensors in tracking volume
3. Run the PC application - it will calibrate automatically
4. Enter target coordinates when prompted: `X: <value> Y: <value>`
5. System will calculate angles and move motors to position
6. Monitor CSV logs for debugging: `sensor_data.csv`

---

## 📊 Results

### Achievements

✅ **Reliable sensor tracking** with quality filtering (removes invalid readings)  
✅ **Smooth motor control** with minimal overshoot thanks to tuned PID  
✅ **Robust communication** - simple protocol with acknowledgment  
✅ **Comprehensive logging** - all sensor data captured for analysis  
✅ **Sub-millimeter positioning** accuracy with Aurora system  

### Performance Metrics

- **Tracking Accuracy**: ~1 mm (Aurora specification)
- **Position Control**: ±10 encoder counts dead zone
- **Communication Latency**: <50 ms round-trip
- **System Response Time**: ~5 seconds per movement command

---

## 🔮 Future Improvements

- [ ] Increase control loop frequency (currently 0.2 Hz → target 10+ Hz)
- [ ] Integrate OpenMV camera for computer vision feedback
- [ ] Implement trajectory planning for smooth paths
- [ ] Add GUI for easier operation and visualization
- [ ] Support for additional degrees of freedom
- [ ] Real-time plotting of position and motor data
- [ ] Wireless communication option (ESP32/Bluetooth)
- [ ] Adaptive PID tuning based on load conditions

---

## 📚 References

1. [NDI Aurora User Guide & API Sample](https://github.com/Oct19/NDI-API-Sample-v1.4.0)
2. [STM32L4 Reference Manual (RM0394)](https://www.st.com/resource/en/reference_manual/rm0394-stm32l41xxx42xxx43xxx44xxx45xxx46xxx-advanced-armbased-32bit-mcus-stmicroelectronics.pdf)
3. [OpenMV H7 Documentation](https://docs.openmv.io/)
4. [PID Control Implementation in C](https://www.steppeschool.com/blog/pid-implementation-in-c)

---

## 👨‍💻 Author

**Tuan Kiet Le**  
Student ID: T00404130  
Course: ECE5140 - Embedded Systems  
Instructor: Dr. Tarek Elfouly

---

## 📄 License

This project is available for educational purposes. Please cite this work if you use it in your research or projects.

---

## 🤝 Contributing

Contributions, issues, and feature requests are welcome! Feel free to check the issues page.

---

## ⭐ Acknowledgments

- Dr. Tarek Elfouly for guidance and support
- NDI for Aurora tracking system documentation
- STMicroelectronics for comprehensive STM32 resources
- The embedded systems community for helpful references

---

**Project Status**: ✅ Completed and Operational

*Last Updated: January 2026*
