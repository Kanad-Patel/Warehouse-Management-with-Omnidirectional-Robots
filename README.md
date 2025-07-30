# Warehouse Management System Using Omnidirectional Robots

## Overview

This project leverages **IoT** and **robotics** to optimize warehouse operations using **omnidirectional robots**. These robots, equipped with **Aruco markers** and controlled by a centralized system, offer real-time inventory tracking, autonomous navigation, and efficient object handling. The implementation is designed to enhance productivity and safety in warehouses by automating repetitive tasks.

## Features

- **Omnidirectional Movement**: Robots equipped with omnidirectional wheels for seamless mobility in tight spaces.
- **Aruco Marker-Based Localization**: Utilizes computer vision for precise navigation and object identification.
- **IoT-Driven Control**: Centralized coordination using ESP32 and Raspberry Pi, with real-time wireless communication.
- **Object Handling**: Gripper mechanism for picking and placing inventory.

## Project Structure

```plaintext
📂 Warehouse-Management-Robots
├── 📁 code
│   ├── Object_Tracking.py   # Python script for robot and object tracking
│   ├── IOT_TRUE.ino         # Arduino code for ESP32 communication and servo control
├── 📁 docs
│   ├── Final_Report.pdf   # Detailed project report with system architecture
├── 📁 resources
│   ├── calibration_data.npz # Camera calibration data
├── 📁 images
│   ├── IOT_Robot.jpg    # Image of the robot in action
├── README.md                # Project overview and instructions

```
## Installation

### Prerequisites

1. **Hardware**:
   - Omnidirectional robot chassis with MG995 servo motors.
   - ESP32 microcontroller with Wi-Fi capabilities.
   - Logitech C270 Webcam for marker detection.

2. **Software**:
   - Python 3.x with OpenCV installed.
   - Arduino IDE for microcontroller programming.

3. **Libraries**:
   - OpenCV for Aruco marker detection.
   - Flask for the web interface (optional).
   - ESP32Servo for servo motor control.

### Steps

1. Clone this repository:
   ```bash
   git clone https://github.com/yourusername/Warehouse-Management-Robots.git
   cd Warehouse-Management-Robots
    ```
2. Set up the environment:
   - Install Python dependencies:
     ```bash
     pip install opencv-python flask requests
     ```

3. Upload `IOT_TRUE.ino` to the ESP32 using the Arduino IDE.

4. Run `Object_Tracking.py` to start tracking:
   ```bash
   python code/Object_Tracking.py
    ```
5. Place Aruco markers in the warehouse for localization and mapping.

## Usage

1. Power on the ESP32 and connect it to the Wi-Fi network.
2. Launch the Python tracking script to begin real-time robot monitoring.
3. Use the Flask web interface (optional) to control the robots and view live updates.

## Contributing

We welcome contributions! Please open an issue or submit a pull request if you'd like to enhance this project.

## License

This project is licensed under the [MIT License](LICENSE).
