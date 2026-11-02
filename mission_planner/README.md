# ARES - Mission Planner

## Overview

The Mission Planner is a software subsystem of the ARES (Agricultural Robotic Exploration System) senior design project.

Its primary responsibility is to use predefined field boundaries and the robot's current GPS position to determine a suitable starting location and generate a general scouting route within the designated field.

The generated route will eventually be provided to the autonomous navigation system to guide the robot during field scouting operations.

## Functional Requirement

**Mission Planning:**

The system shall use pre-designated field boundaries to determine a starting location and general scouting route on the field.

## Responsibilities

The Mission Planner is responsible for:

1. Receiving predefined field boundary coordinates.
2. Obtaining the robot's current GPS position.
3. Determining a suitable starting location inside the field.
4. Generating a systematic scouting route.
5. Validating that the planned route remains within the field boundaries.
6. Providing ordered GPS waypoints to the autonomous navigation system.

## Project Structure

```text
mission_planner/
│
├── include/
│   ├── Mission.h
│   ├── Waypoint.h
│   └── RobotConfig.h
│
├── src/
│   ├── Mission.cpp
│   └── main.cpp
│
├── CMakeLists.txt
└── README.md
```

### Source Files

| File | Responsibility |
|---|---|
| Mission.h | Declares the Mission class and its functions. |
| Mission.cpp | Implements mission planning functionality. |
| Waypoint.h | Defines GPS coordinates and waypoint order. |
| RobotConfig.h | Stores robot configuration parameters. |
| main.cpp | Runs and tests the Mission Planner. |

## Current Implementation

The initial software framework has been implemented and successfully compiled.

Current functionality includes:

- Creating a Mission object.
- Storing predefined field boundary coordinates.
- Storing the robot's GPS position.
- Assigning the robot's current position as a temporary starting point.
- Creating an initial route containing the starting waypoint.
- Retrieving and displaying field boundary coordinates.
- Retrieving the generated route.

**Note:** Starting point selection, route generation, and boundary validation currently use temporary implementations. The actual planning algorithms are still under development.

## Planned Features

- [ ] Validate whether the robot's GPS position is inside the field.
- [ ] Determine a suitable starting location.
- [ ] Generate a systematic coverage route using a lawnmower-style pattern.
- [ ] Validate route waypoints against field boundaries.
- [ ] Export ordered GPS waypoints.
- [ ] Integrate field boundary input from the ARES operator application.
- [ ] Integrate the generated route with the autonomous navigation subsystem.

## Building the Project

The project uses C++ and CMake.

### Requirements

- C++ compiler
- CMake
- Visual Studio Build Tools (MSVC)

### Compile Using PowerShell

Navigate to the mission planner directory:

```powershell
cd mission_planner
```

Create and enter the build directory:

```powershell
mkdir build
cd build
```

Configure the project:

```powershell
cmake ..
```

Compile:

```powershell
cmake --build .
```

### Run the Executable

For the Visual Studio Debug configuration:

```powershell
.\Debug\mission_planner.exe
```

## Testing

The initial implementation uses hardcoded GPS coordinates representing a rectangular field.

The program displays the field boundary coordinates and reports the number of waypoints generated.

These test coordinates will eventually be replaced with field boundary information received from the ARES operator application.

## Future Integration

The Mission Planner will interface with:

- **ARES Operator Application:** Provides designated field boundaries and mission information.
- **Autonomous Navigation System:** Receives the generated GPS waypoint sequence and uses it to guide robot movement.

The expected data flow is:

Field Boundaries + Robot GPS Position  
→ Mission Planner  
→ Starting Location Selection  
→ Scouting Route Generation  
→ Boundary Validation  
→ Ordered GPS Waypoints  
→ Autonomous Navigation