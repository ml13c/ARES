# ARES
### Autonomous Reconnaissance and Evaluation System

ARES is a mobile agricultural scouting robot designed to autonomously detect, locate, and report hazards and other objects within a field.

The robot will use cameras, environmental sensors, GPS localization, wireless communication, and an FPGA-accelerated convolutional neural network (CNN) to process its surroundings. ARES is intended to assist with field scouting by identifying objects such as rocks, weeds, and other potential hazards before or during agricultural operations.

The goal of the project is to combine autonomous movement, object detection, localization, mapping, and communication into a single mobile robotic system.

---

## Project Modes

ARES will support two primary operating modes.

### Pre-Scan Mode

In Pre-Scan Mode, ARES will autonomously survey a designated area before agricultural equipment enters the field.

During the scan, the system will:

- Navigate through the designated field area
- Detect supported objects and hazards
- Determine the location of detected objects
- Store detected objects and their locations
- Generate information that can later be reviewed by the operator

This allows an operator to identify possible hazards before beginning field operations.

### Real-Time Scout Mode

In Real-Time Scout Mode, ARES will operate ahead of agricultural equipment while continuously scanning the surrounding area.

When a supported hazard is detected, the system will:

- Identify the object using the vision system
- Determine the object's location
- Send the detection and location to the operator
- Continue following the active scouting path
- Regulate its speed to maintain a safe lead distance ahead of the equipment

---

## System Overview(make diagram later)

ARES is divided into several major subsystems.

```text
                        ARES

                 ┌───────────────┐
                 │    Camera     │
                 └───────┬───────┘
                         │
                         ▼
                 ┌───────────────┐
                 │     FPGA      │
                 │               │
                 │ Image / CNN   │
                 │ Processing    │
                 └───────┬───────┘
                         │
                    Detections
                         │
                         ▼
      ┌──────────┐   ┌───────────────┐   ┌─────────────┐
      │   GPS    │──▶│    Vehicle    │◀──│   Sensors   │
      └──────────┘   │  Controller   │   └─────────────┘
                     └───────┬───────┘
                             │
              ┌──────────────┼──────────────┐
              │              │              │
              ▼              ▼              ▼
         Navigation       Mapping        Wireless
                                        Communication
                                             │
                                             ▼
                                         Operator