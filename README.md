🏗️ Autonomous Construction Site Inspection Drone (Digital Twin)
<img width="1024" height="1024" alt="chad_a100f2d96a5d4f65a67465af606aaf98" src="https://github.com/user-attachments/assets/47ec9106-edd1-4862-9f7e-cd7d60b8de75" />

                                                                             Project Overview
This project implements a **Digital Twin** of a construction site using **Webots** robotics simulator. The system features an autonomous drone (`Mavic 2 Pro`) equipped with computer vision (`YOLOv8`) that patrols a construction site, detects safety hazards (like people or fire extinguishers), and responds to them. The 3D construction site model was designed in **Blender** and imported into Webots.
                                                                                Key Features
- **Autonomous Drone Flight:** PID-controlled quadcopter capable of autonomous takeoff, waypoint patrol, and landing.
- **AI-Powered Computer Vision:** Integration with a local Flask server running `YOLOv8` for real-time object detection.
- **Smart Response Logic:** The drone dynamically alters its flight path and hovers upon detecting objects (e.g., `person`, `fire hydrant`).
- **3D Modeling Pipeline:** Custom 3D construction site imported from Blender (`.stl` / `.obj` / `.glb`).
- **Industry 4.0 Simulation:** Simulates real-world construction monitoring and safety inspection.

                                                                                 Tech Stack
- **Simulation:** Webots (Cyberbotics)
- **Robotics:** Mavic 2 Pro (Quadcopter), UR5e (Manipulator - future integration)
- **AI & Computer Vision:** YOLOv8 (Ultralytics), OpenCV, Flask
- **3D Modeling:** Blender (ArchiCAD workflow)
- **Programming:** C++ (for robotic controllers), Python (for AI server integration)

                                                                           Repository Structure
```text
├── controllers/
│   ├── drone_yolo_controller.py    # Autonomous drone logic + YOLO integration
│   └── my_controller111.cpp        # (Optional) C++ controller examples
├── worlds/
│   └── construction_site.wbt       # Main simulation world file
├── objects/
│   └── house.stl                   # 3D model imported from Blender
├── yolo_server.py                  # Flask server running YOLOv8
└── README.md

                                                                            How to Run the Project

Start the YOLO Server:

pip install ultralytics opencv-python flask requests
python yolo_server.py
         (Server runs at http://127.0.0.1:5000)

Open Webots:

Launch Webots and open the construction_site.wbt file.

Ensure the drone's controller is set to drone_yolo_controller.py.

Run Simulation:

Press the Play button. The drone will take off, patrol the site, and scan for objects.

Watch the Webots console for AI detections: " YOLO обнаружил: fire hydrant!".

 Results & Performance
Drone successfully patrols a 5x5 meter area at 3-5 meters altitude.

YOLOv8 processes images at ~30 FPS on a local CPU (2ms preprocess, 33ms inference).

The system successfully detects construction site objects and autonomously adjusts behavior.

                                  Author & Portfolio
Author: [Viacheslav]

Core Skills: Robotics, Digital Twins, BIN Automation, Revit API, Python, C++, Blender.

 Future Enhancements
Training a custom YOLO model specifically for construction hazards (hard hats, scaffolding, wet floors).

Integrating the UR5e manipulator to physically interact with the environment (Pick & Place).

Streaming simulation data to a web-based dashboard for remote monitoring.
