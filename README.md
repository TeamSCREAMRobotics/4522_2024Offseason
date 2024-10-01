# 4522 2024 Offseason Code 
Rewritten code for AXL during the 2024 offseason.

## Features
* Generic TalonFXSubsystem interface using goal-based control system
* CTRE generated swerve
* Full simulation support
* Logging with [DogLog](https://doglog.dev/)
* Choreo and Pathplanner support
<br>

### TalonFXSubsystem
* Inspired by 254's [ServoMotorSubsytem]()
* Built-in simulation support
* Designed to be as user-friendly and reusable as possible
<br>

### Simulation
* Full game piece simulation
  * Visualize, pickup, and score
* Simulation support for every subsystem
* CAD imported for realistic visualization

![Simulation](.github/docs/sim-readme.gif)

## Why?
While our competition code performed well, it was our first time using the command-based framework. This lead to a multitude of code cleanliness and structure problems. With a better understanding of the framework, we decided to rewrite our code with a focus on user-friendliness and reusability. In addition, we sought out to improve on some of our shortcomings in the past season's software, such as:
* Logging support
* Reusable and clean code
* Using resources where possible