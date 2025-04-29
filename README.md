# MRILib

**Docs: ** https://rubytanuki.github.io/MRILib
**Modular Robotics Infrastructure Library** for FTC  
A flexible, simple Java library for creating clean, abstracted, and reusable robot code for TeleOp and Autonomous modes in FTC.

---

## 🚀 Overview

MRILib is a modular codebase for FTC robots that enables rapid development, clean structure, and advanced functionality without boilerplate.

It provides:

- A layered robot class system (`Bot`, `ArmBot`, `AutoBot`) to manage all subsystems
- Extensible state machines for both autonomous and TeleOp
- A refined `DebouncedGamepad` for smooth, safe input handling
- Smooth PID-based motion control using `PIDController`
- Clean abstractions for drive control, arms, and sensor integrations
- Complex Feed-Forward Motion Profile pivoting arm control

---

## 📦 Library Structure

### 🔧 Robot Management Classes (`managers/`)
- `Bot`: Base class for drivetrain and odometry
- `ArmBot`: Extends `Bot` with pivot, slide, and claw components
- `AutoBot`: Extends `ArmBot` with vision and sensor processing
- These classes encapsulate both **initialization** and **processing** logic for their respective subsystems.

### 🤖 State Machines (`statemachine/`)
- `DriveFSM`: Controls sequential drive tasks and ArmFSM calls during Autonomous
- `ArmFSM`: Manages states of the arm during both TeleOp and Auto
- `BotState`: Base class for state machine steps
- Includes private subclassing of `BotState` in `ArmFSM` for encapsulated arm logic

### 🧠 Motion Control (`motion/`)
- `PID`: Helper Class to calculate a single vector PID control
- `HeadingPID`: PID Control class for specifically anglewrapped Odometry Heading
- `PIDController`: 2D motion profiling with `x`, `y`, and `theta` components
- Used by `DriveFSM` to execute smooth, accurate motion steps

### 🎮 Input Utilities (`util/`)
- `DebouncedGamepad`: Drop-in replacement for FTC’s Gamepad that handles input edge detection cleanly
- Simplifies toggles and conditional actions in TeleOp

---

## 💡 Key Features

- Modular robot building through layered base classes
- Autonomous drive control via FSM and PID
- Reliable arm and claw state management
- Gamepad debouncing for edge-triggered control
- Readable, minimal syntax for adding conditions and actions
