# 🤖 Night Patrol Robot - Control System

## 👨‍💻 Author

Adarsh V

---

## 📌 Overview

This module implements the **Control, Safety, and User Interface layer** of the Night Patrol Robot Simulation project.

It ensures:

* Manual control of robot movement
* Emergency stop functionality
* Safe recovery system
* User interaction via terminal and UI

---

## 🧱 System Architecture

```
User Input (Keyboard / UI)
            ↓
     Teleop Node (/cmd_vel_input)
            ↓
     Control Node (Safety Layer)
            ↓
        /cmd_vel
            ↓
         Robot
```

---

## 📁 Folder Structure

```
patrol_control/
├── setup.py
├── package.xml
├── README.md
└── patrol_control/
    ├── __init__.py
    ├── control_node.py     # Main control + safety logic
    ├── teleop_node.py      # Keyboard input control
    └── emergency_node.py   # Emergency stop & resume

dashboard.py               # Streamlit UI (outside ROS package)
```

---

## ⚙️ Modules Description

### 🔹 control_node.py

* Central control system
* Publishes `/cmd_vel`
* Subscribes:

  * `/cmd_vel_input`
  * `/emergency_stop`
* Features:

  * Emergency override
  * Timeout safety (auto-stop)

---

### 🔹 teleop_node.py

* Keyboard-based control
* Publishes movement commands to `/cmd_vel_input`

Controls:

* W → Forward
* S → Backward
* A → Left
* D → Right
* Q → Quit

---

### 🔹 emergency_node.py

* Sends emergency commands

Commands:

* `e` → Emergency Stop
* `r` → Resume

---

### 🔹 dashboard.py

* Streamlit-based UI
* Buttons:

  * Move robot
  * Emergency Stop
  * Resume

---

## 🚀 How to Run

### 1️⃣ Build Workspace

```
cd ~/ros2_ws
colcon build
source install/setup.bash
```

---

### 2️⃣ Run Nodes (3 terminals)

#### Terminal 1

```
ros2 run patrol_control control_node
```

#### Terminal 2

```
ros2 run patrol_control teleop_node
```

#### Terminal 3

```
ros2 run patrol_control emergency_node
```

---

### 3️⃣ Run UI (Optional)

```
pip install streamlit
streamlit run dashboard.py
```

---

## 🛑 Safety Features

* Emergency Stop overrides all commands
* Timeout-based auto stop (if no input)
* Safe recovery using resume command

---

## 🔗 Integration Notes

* Connect `/cmd_vel` to robot in simulation
* Works with ROS2 Navigation stack
* Compatible with TurtleBot3

---

## 🎯 Demo Flow

1. Start control node
2. Move robot via keyboard/UI
3. Trigger emergency stop
4. Robot halts immediately
5. Resume operation

---

## 📌 Notes

* Designed to work independently
* Can be integrated with any ROS2 robot system
* Focuses on reliability and user control

---

