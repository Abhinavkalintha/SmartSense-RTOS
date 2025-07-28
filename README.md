# 🚀 SmartSenseRTOS – RTOS-Based Home Automation System on STM32

**SmartSenseRTOS** is a FreeRTOS-powered embedded project built on the STM32 platform. It brings together real-time environmental monitoring, smart energy tracking, and automated device control in a compact, modular system. Designed with reliability and scalability in mind, the project uses STM32 HAL drivers and is tailored with custom GPIO configurations and multitasking logic for seamless performance.

> 💡 *Note:* This project is a modular prototype designed with flexibility in mind. While it demonstrates core embedded features using STM32 and FreeRTOS, it serves as a foundational concept that can be expanded or integrated into more complex systems and real-world applications.

---

## 🔧 Features

- 🎛️ **Servo Motor Control** — Controlled via PWM signal for position adjustment.  
- 📏 **Ultrasonic Sensor** — Measures distance and updates the LCD display.  
- 👀 **Motion Detection** — Uses ADC to read motion sensor values.  
- 📺 **LCD Display** — Real-time updates from sensors shown on a 16x2 character LCD.  
- ⏱️ **RTOS** — Multiple concurrent tasks managed using FreeRTOS.  
- 💡 **LED Blinkers** — Onboard LEDs blink with different delays using RTOS tasks.

---

## 🛠️ Hardware Setup

This project is designed using the **Nucleo-F446RE** development board — a powerful and beginner-friendly STM32 MCU with excellent peripheral support and built-in debugging.

### 📋 Required Components

| Component         | Description                                      |
| ----------------- | ------------------------------------------------ |
| Nucleo-F446RE     | STM32F4 development board (Cortex-M4 @180MHz)    |
| Servo Motor       | SG90 or compatible (PWM controlled)              |
| Ultrasonic Sensor | HC-SR04                                           |
| Motion Sensor     | PIR sensor or analog IR sensor                     |
| 16x2 LCD Display  | I2C-based character LCD                            |
| LEDs              | 1 onboard (PC13) + 1 external (optional)          |
| Breadboard + Wires| For wiring the peripherals                         |
| 5V Power Supply   | For stable servo operation                          |

---

## 💻 Software Tools

| Software / Tool     | Purpose                                   | Version            |
| ------------------- | ----------------------------------------- | ------------------ |
| STM32CubeIDE        | All-in-one IDE for STM32 development      | v1.15.0 or latest  |
| STM32CubeMX         | Peripheral configuration and code gen     | (Built-in)         |
| FreeRTOS            | Real-time OS for task scheduling           |                    |
| CMSIS RTOS2         | RTOS API via CubeMX                         |                    |
| HAL Drivers (STM32) | Hardware abstraction for peripheral control|                    |
| STM32F4 HAL LCD Lib | 16x2 LCD display control via I2C            |                    |
| Custom us_delay.c/h | Microsecond delay utilities for ultrasonic sensor |             |

---

## ⚙️ How It Works

This project uses FreeRTOS to run multiple tasks simultaneously on the STM32 Nucleo-F446RE board. Each sensor or device works in its own task, so everything runs smoothly without waiting for other things to finish.

### 🧵 What Each Task Does

- **Servo Task:** Moves the servo motor back and forth every few seconds using PWM.  
- **Ultrasonic Task:** Sends a sound wave and waits for the echo to measure distance. It repeats every 2 seconds.  
- **Motion Sensor Task:** Reads the motion sensor using the ADC and updates the value every 0.5 seconds.  
- **LCD Task:** Shows the motion sensor and distance values on a 16x2 LCD screen. Updates every 1 second.  
- **LED Tasks:** Two tasks blink two LEDs at different speeds to show how FreeRTOS handles multitasking.

### ⚙️ How It All Works Together

- The RTOS scheduler makes sure each task runs at the right time.  
- PWM controls the servo motor.  
- Timers and Input Capture measure the ultrasonic sensor.  
- ADC reads analog data from the motion sensor.  
- I2C sends data to the LCD screen.  

> 💡 Everything runs in parallel without blocking each other — that’s the power of RTOS!

---

## 📁 Project Structure

The project is organized in a modular way to keep each peripheral and task separate, making it easy to understand, modify, or extend.

```STM32_RTOS_Peripherals/
├── Core/
│   ├── Inc/
│   │   ├── main.h                # Main header
│   │   ├── lcd.h                 # LCD display functions
│   │   ├── us_delay.h            # Microsecond delay utility
│   │   └── tasks.h               # Task function declarations
│   ├── Src/
│   │   ├── main.c                # Entry point (creates tasks)
│   │   ├── lcd.c                 # LCD control code
│   │   ├── us_delay.c            # Delay functions for ultrasonic sensor
│   │   ├── freertos.c            # RTOS task definitions & handles
│   │   └── tasks/
│   │       ├── servo_task.c          # Servo motor control logic
│   │       ├── ultrasonic_task.c    # Ultrasonic distance calculation
│   │       ├── motion_task.c         # Motion sensor ADC reading
│   │       ├── lcd_task.c            # LCD display updates
│   │       └── led_blink_task.c      # LED blinking threads
├── Drivers/                      # HAL libraries (auto-generated)
├── Middlewares/                  # FreeRTOS kernel files
├── .ioc                         # STM32CubeMX config file
├── README.md                    # Project documentation
└── Makefile / .project          # Build system files```

