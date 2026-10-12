# ARES Requirements Traceability Matrix

## Purpose

This document tracks each ARES system requirement from definition through
implementation and testing.

The traceability matrix links requirements to:

- responsible team member
- subsystem
- implementation location
- planned verification method
- test location
- current project status

This document should be updated throughout development as requirements,
implementations, and test results change.

---

# Functional Requirements

| ID | Requirement | Owner | Subsystem | Planned Implementation | Verification / Test | Test Location | Status |
|---|---|---|---|---|---|---|---|
| FR1 | The robot shall autonomously operate in a field without the user actively controlling it. | Marcus Lara Sanchez | Navigation / Vehicle Control | Vehicle controller will use sensor, localization, and path data to determine movement commands and control the motors. | Place the robot on a predefined test course and verify that it completes the route without continuous manual control. | `tests/field/fr1_autonomous_operation/` | Not Started |
| FR2 | The system shall use pre-designated field boundaries to determine a starting location and general scouting route within the field. | Alexavier Mendez | Navigation / Mission Planning | Field boundary information will be loaded into the navigation system and used to generate or constrain the scouting route. | Provide a known field boundary and verify that the generated route remains inside the designated operating area. | `tests/navigation/fr2_field_boundaries/` | Not Started |
| FR3 | The system shall identify and follow crop rows while maintaining the intended scouting path during operation. | Jackson Schroader | Vision / Navigation | Camera and image-processing data will be used to identify crop rows and provide path information to the navigation controller. | Use representative crop-row images or a controlled test environment and verify that the robot identifies and follows the intended row. | `tests/field/fr3_row_following/` | Not Started |
| FR4 | The system shall perform edge detection on captured image data to identify significant boundaries and features for further processing. | Jackson Schroader | FPGA / Vision | FPGA image-processing logic will perform edge detection on incoming camera frames. | Provide known test images to the FPGA and compare detected edges against expected image features or a reference implementation. | `tests/fpga/fr4_edge_detection/` | In Progress |
| FR5 | The system shall classify relevant field objects and hazards within its sensing area. | Cynai Hayes | FPGA / CNN / Vision | Camera images will be processed by the object-detection/classification system to identify supported field objects and hazards. | Run a validation dataset and controlled physical-object tests and record classification results. | `tests/vision/fr5_object_classification/` | Not Started |
| FR6 | The system shall distinguish between desired crops and weeds for precise pesticide spraying. | Jackson Schroader | FPGA / CNN / Vision | Image-classification logic will identify whether detected vegetation is a desired crop or weed. | Test the classifier against labeled crop and weed images and record classification results. | `tests/vision/fr6_crop_weed_classification/` | Not Started |
| FR7 | The system shall track the robot's location and the location of detected objects. | Marcus Lara Sanchez | Localization / GPS | Robot position will be obtained from GPS. FPGA detections will be sent to the vehicle controller and associated with the robot's position to determine or record the location of detected objects. | Place known objects at measured locations, detect them with ARES, and compare the reported positions against known locations. | `tests/localization/fr7_object_localization/` | In Progress |
| FR8 | The system shall support a Pre-Scan Mode for surveying a designated field area and recording detected objects. | Alexavier Mendez | Mission Control / Mapping | Pre-Scan Mode will command the robot through the designated scouting area while detections and position data are stored for later review. | Run a simulated or controlled Pre-Scan mission and verify that the required area is surveyed and detected objects are recorded. | `tests/field/fr8_prescan_mode/` | Not Started |
| FR9 | The system shall support Real-Time Scout Mode where it scans for objects/hazards and provides their locations. | Marcus Lara Sanchez | Mission Control / FPGA / Communications | Camera frames will be processed while the robot is moving. Detected hazards and associated location information will be transmitted to the operator. | Operate ARES in Real-Time Scout Mode and verify that known hazards are detected and their location information is sent to the operator. | `tests/field/fr9_realtime_scout/` | Not Started |
| FR10 | The system shall record in real time the location and relevant information of detected objects to support generation of a map for post-operation viewing and analysis. | Alexavier Mendez | Mapping / Logging | Detected-object information and GPS coordinates will be stored by the vehicle computer and converted into map data. | Complete a test mission containing known objects and verify that each detection appears in the saved mission data/map. | `tests/mapping/fr10_detection_mapping/` | In Progress |
| FR11 | The system shall notify an operator of relevant hazards detected during real-time scouting. | Cynai Hayes | Communications / Operator Interface | Detection messages containing hazard information will be transmitted from ARES to the operator interface. | Introduce a known hazard during operation and verify that an appropriate operator notification is received. | `tests/communications/fr11_hazard_alert/` | Not Started |
| FR12 | The system shall provide an interface through which an operator can configure scouting operations and review scouting results. | Cynai Hayes | Operator Interface | An operator application/interface will allow mission configuration and display stored or real-time scouting information. | Configure a test mission through the interface and verify that stored scouting results can later be opened and reviewed. | `tests/ui/fr12_operator_interface/` | In Progress |
| FR13 | The system shall make stored results and identified objects available for later review locally. | Jackson Schroader | Data Storage / Operator Interface | Mission results, object detections, and associated information will be stored locally in a format accessible after the mission. | Run a mission, shut down the active scouting operation, and verify that previous results remain available for review. | `tests/storage/fr13_result_storage/` | Not Started |
| FR14 / FR15 | The system shall maintain a lead distance ahead of agricultural equipment by regulating its speed along the active path. | Marcus Lara Sanchez | Navigation / Vehicle Control | The vehicle controller will compare the position of ARES with the position of the agricultural equipment and adjust robot speed to maintain a specified separation distance. | Simulate or operate with a moving reference vehicle and measure whether ARES maintains the required lead-distance range. | `tests/field/fr15_lead_distance/` | Not Started |

---

# Non-Functional Requirements

| ID | Requirement | Owner | Subsystem | Planned Implementation | Verification / Test | Test Location | Status |
|---|---|---|---|---|---|---|---|
| NFR1 | The system shall meet the required single-charge operating duration under nominal scouting conditions. | Cynai Hayes | Power System | Battery capacity, regulators, and system power consumption will be selected to support the required operating duration. | Fully charge the robot, operate under representative conditions, and measure total operating time until the defined battery limit is reached. | `tests/power/nfr1_battery_runtime/` | Not Started |
| NFR2 | The system shall support an effective scouting width during normal field operation. | Jackson Schroader | Sensors / Vision / Navigation | Camera and sensor placement will provide sufficient coverage around the robot for the defined scouting width. | Place detectable objects at known lateral distances from the robot's path and determine the usable detection width. | `tests/field/nfr2_scouting_width/` | Not Started |
| NFR3 | The system shall detect a hazard and transmit an alert with an end-to-end latency below 300 ms. | Marcus Lara Sanchez | FPGA / Vehicle Controller / Communications | Timestamps will be recorded at image availability, FPGA processing, controller reception, and operator reception to determine total system latency. | Run repeated hazard detections and calculate the time from image-frame availability to receipt of the operator alert. Verify total latency is less than 300 ms. | `tests/latency/nfr3_end_to_end_latency/` | Not Started |
| NFR4 | The FPGA-based classifier shall demonstrate improved computational performance relative to a CPU-based reference implementation using metrics including inference latency, throughput, power consumption, and energy per inference. | Jackson Schroader | FPGA / CNN | The same or equivalent classifier workload will be executed using FPGA and CPU implementations so their performance can be compared. | Benchmark both implementations using the same dataset and record latency, throughput, power, and energy-per-inference measurements. | `tests/performance/nfr4_fpga_cpu_comparison/` | Not Started |
| NFR5 | The system shall support a repeatable live demonstration in a controlled environment representing typical field conditions and allowing major scouting functions to be observed and evaluated. | Alexavier Mendez | System Integration | A repeatable demonstration course will be established containing representative paths, crops, objects, and hazards. | Run the complete demonstration multiple times using the same test setup and verify that major scouting functions can consistently be demonstrated. | `tests/system/nfr5_demo_repeatability/` | Not Started |
| NFR6 | The system shall maintain reliable operation and communication throughout a normal scouting mission without requiring a manual restart or intervention. | Alexavier Mendez | System Integration / Communications | Watchdogs, communication recovery, fault handling, and software state management will be used to prevent system lockups during normal operation. | Run extended scouting missions while logging resets, lost communication, program failures, and required user interventions. | `tests/reliability/nfr6_mission_reliability/` | Not Started |



# Traceability Conventions

Each requirement should have a corresponding implementation location and
verification test whenever possible.

Example:

FR7
→ Localization implementation
→ GPS driver
→ Detection-position association
→ Integration test
→ Field verification

Code, tests, issues, and pull requests should reference the requirement ID.

Examples:

    FR7: Add GPS coordinate parser

    FR7: Associate FPGA detections with robot position

    NFR3: Add FPGA processing timestamp

    NFR3: Measure radio transmission latency

GitHub commits may also reference requirements:

    FR7: add GPS position packet parser

    FR9: implement real-time hazard message

    NFR3: add end-to-end latency logging

