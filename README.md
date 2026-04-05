# Smart-lawn-mower-
# 🌿 ESP32 Smart Lawn Mower

An **autonomous smart lawn mower** powered by **ESP32**, designed to mow a specified area efficiently without using encoders. This rover combines **autonomous navigation, obstacle avoidance, and manual control** into a sleek, rechargeable system.

---

## 🎯 Features

| Feature | Description |
|---------|-------------|
| 🌱 **Autonomous Area Coverage** | Input length and width, and the mower covers the entire lawn systematically. |
| ⚙️ **Motion Motors** | 2 DC motors drive forward, backward, and turn without encoders. |
| 🌀 **Blade Motor** | Third motor spins the cutting blades, with dedicated **On/Off control button**. |
| 🚧 **Obstacle Detection** | Ultrasonic sensor stops the mower if a person or object is detected. |
| 🔋 **Rechargeable Batteries** | Powers all motors and electronics; supports onboard USB-C charging. |
| 📡 **Bluetooth Control** | Optional manual override (Forward, Backward, Left, Right, Stop). |
| 💡 **Simple yet Smart** | Achieves full coverage and safety without complex feedback systems. |

---

## ⚡ Technologies & Components

- **ESP32** microcontroller  
- **DC Motors + Motor Driver** (for motion)  
- **Blade Motor** with On/Off button  
- **Ultrasonic Sensor** (HC-SR04 or similar)  
- **Rechargeable battery pack** powering the rover entirely  
- **USB-C Charging Module with BMS** (Battery Management System)  
- **Bluetooth Serial** for manual control  
- **Arduino IDE** for programming  

---

## 🔋 Power System & Charging

The mower is powered by **three rechargeable Lithium-ion batteries** arranged in a pack. The system ensures safe, efficient operation of all electronics and motors.

| Component | Specification / Description |
|-----------|----------------------------|
| **Battery Pack** | 3 × Lithium-ion rechargeable cells (3.7V each) |
| **Buck Converter** | Steps down voltage to stable 5V/12V for ESP32 and motors |
| **Charging Module** | USB-C input with integrated **BMS** for safe charging |
| **BMS Functions** | Overcharge, over-discharge, short-circuit protection, and cell balancing |
| **Type-C Input** | Convenient single-port charging for the whole rover |

**Operation:**  
1. Connect USB-C charger to the module.  
2. BMS manages safe charging of all batteries.  
3. Buck converter regulates voltage for motors and electronics.  
4. Rover can optionally operate while charging if current allows.  

---

## 🖼️ Conceptual Diagram

**

---

## 🚀 How It Works

1. Set the **length and width** of the area to mow.  
2. Press **Start**:  
   - Rover moves autonomously using the two motion motors.  
   - Blade motor spins when toggled on via button.  
3. Ultrasonic sensor constantly monitors for obstacles:  
   - Stops the mower if an object is detected.  
   - Resumes movement when the path is clear.  
4. Optional: manual override via Bluetooth commands.

---

## 🎨 Visuals & Colors

- Use **green headings** and **leaf emojis** for a garden feel.  
- Include images of the rover, wiring, and layout for a visual guide.  
- Example placeholders:  
  ![Rover Front]()  
  ![Wiring Diagram]()  

---

## 💡 Why This Project?

This project demonstrates a **professional-grade autonomous lawn mower** that is:

- Simple to build without encoders  
- Fully rechargeable with safe BMS  
- Safe and obstacle-aware  
- Expandable for PWM speed control or extra sensors  

Perfect for **robotics enthusiasts, IoT developers, and hobbyists** looking to build a practical smart garden automation system.

---
