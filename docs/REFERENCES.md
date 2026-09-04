# Reference Documentation

One-stop reference hub. Key facts are extracted inline so the lab doesn't depend on WiFi or vendor sites; deep links for everything else. Local PDF copies live in [`docs/vendor/`](vendor/) (private-repo use — recheck redistribution terms before making the repo public).

## UR7e (arm)

**Key facts:** e-Series; payload 7.5 kg; reach 850 mm; repeatability ±0.03 mm; 6 DoF; joint speed max 180°/s; weight 20.6 kg; footprint Ø151 mm; IP54; 17 configurable safety functions (PLd Cat 3). Our unit runs **PolyScope 5.23** (verified at pendant 2026-09-04); driver minimum is 5.9.4.

**Control-box network interfaces** (all TCP servers on the robot):

| Port | Interface | Notes |
|---|---|---|
| 30001 / 30011 | Primary (rw / read-only) | 10 Hz state + URScript injection |
| 30002 / 30012 | Secondary (rw / read-only) | 10 Hz |
| 30003 / 30013 | Real-time client (rw / read-only) | |
| **30004** | **RTDE** | 500 Hz — the ROS driver's data channel |
| **29999** | **Dashboard server** | load/play/stop programs, brake release, unlock protective stop |

Driver-side ports opened on the Jetson: 50001 (reverse interface), **50002 (script sender — the URCap's "Custom port")**, 50003 (trajectory), 50004 (script command).

**Tool flange:** ISO 9409-1-50-4-M6, M8 8-pin; 2 DI + 2 DO; 2 AI **or** 1× RS-485; 12/24 V supply, 1–1.5 A. The RS-485 is how we drive the RG2 (D6).

**Links:**
- Product page: https://www.universal-robots.com/products/ur7e/
- Technical sheet (PDF, local copy in `vendor/`): https://www.universal-robots.com/manuals/EN/TechSheets/UR7e_techsheet_pdf_online/UR7e_techsheet_en.pdf
- User manual (HTML, SW 5.25): https://www.universal-robots.com/manuals/ — select UR7e + your SW line
- TCP/IP remote-control article (port semantics): https://www.universal-robots.com/articles/ur/interface-communication/remote-control-via-tcpip/

## ROS 2 driver stack

- `Universal_Robots_ROS2_Driver` (humble branch): https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver
- Driver docs (startup, controllers, URSim workflow): https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation/
- `ur_calibration` usage: https://docs.ros.org/en/humble/p/ur_calibration/
- External Control URCap: https://github.com/UniversalRobots/Universal_Robots_ExternalControl_URCap
- RS485 Daemon URCap (tool-comm forwarder for the gripper): https://github.com/UniversalRobots/Universal_Robots_ToolComm_Forwarder_URCap
- Client library (RT threading, PolyScope minimums): https://github.com/UniversalRobots/Universal_Robots_Client_Library
- URSim Docker (pin `:5.23` to match our robot): https://hub.docker.com/r/universalrobots/ursim_e-series
- Gazebo sim: https://github.com/UniversalRobots/Universal_Robots_ROS2_GZ_Simulation

## OnRobot RG2 v2 (gripper)

**Key facts:** stroke 0–110 mm (resolution 0.1 mm, repeatability 0.1–0.2 mm); grip force 3–40 N (±25%); speed 38–127 mm/s; payload 2 kg force-fit / 5 kg form-fit; weight 0.78 kg (+ ~0.2 kg Quick Changer); 20–25 V DC, 70–600 mA (3 A/6 ms release spikes — within e-Series tool supply); IP54; **maintains grip force on power loss**. Speaks **Modbus RTU over tool RS-485 @ 1 Mbaud** (even parity, 1 stop bit) — the v2 hardware revision is what enables Compute-Box-less tool-connector operation.

**TCP/CoG (RG2 on Quick Changer, 0° mount):** TCP = [0, 0, 200 mm]; CoG = [0, 0, 64 mm]; mass 0.78 kg. Finger extensions derate force (50 mm → 66.7%, 100 mm → 50%).

**Critical integration rule (D6):** the **OnRobot URCap must be disabled** when ROS controls the gripper — it seizes Tool I/O ("Controlled by OnRobot") and its RS-485 daemon conflicts with UR's forwarder URCap. Set Tool I/O to "Controlled by User", 24 V.

**Links:**
- Product page: https://onrobot.com/en/products/rg2-finger-gripper
- User manual for UR robots, Quick Changer + RG2 v1.17.0 (PDF, local copy in `vendor/`): https://media.ecosphere-solutions.de/instructions/de/onrobot/RG2/User_Manual_For_UR_Robots_Quick_Changer_RG2_v1.17.0_EN.pdf
- ROS2 drivers: https://github.com/tonydle/OnRobot_ROS2_Driver (serial/TCP, ros2_control) · https://github.com/tonydle/UR_OnRobot_ROS2 (combined UR+RG2 URDF/MoveIt) · https://github.com/ABC-iRobotics/onrobot-ros2 (Compute Box Modbus TCP)

## ZED 2i (camera) — quick links

- SDK downloads (need 5.2+ for JetPack 6.2): https://www.stereolabs.com/developers/release
- ROS2 wrapper: https://github.com/stereolabs/zed-ros2-wrapper
- Depth modes & settings: https://www.stereolabs.com/docs/depth-sensing/depth-settings

## Jetson Orin Nano — quick links

- JetPack 6.2 (Super mode): https://developer.nvidia.com/embedded/jetpack-sdk-62
- jetson-containers (NanoOWL/NanoSAM/ollama images): https://github.com/dusty-nv/jetson-containers
- NanoOWL: https://github.com/NVIDIA-AI-IOT/nanoowl · NanoSAM: https://github.com/NVIDIA-AI-IOT/nanosam
