# ITAI 2374 - Robots

**Course:** ITAI 2374 - Robots  
**Semester:** Fall 2025  
**Student:** Martin Demel  
**Institution:** Houston Community College (HCC)

---

## 📋 Repository Overview

This repository contains coursework, assignments, and projects completed during the ITAI 2374 Robotics course. The repository showcases practical applications of robotics concepts, including sensor technology, autonomous navigation, and embedded systems programming with Raspberry Pi 5 and the SunFounder PiCar-X robot.

---

## 📁 Repository Structure

```
ITAI 2374 - Robots/
├── Assignment1 - Robotics Sensors _DEMEL.docx
├── Assignment1_GLOSSARY_DEMEL.docx
├── Raspberry Pi5 HW_Demel_Musil.pptx
├── raspberry-pi-5-product-brief.pdf
├── final/
│   ├── auto_park_between_squares.py
│   ├── AUtonomus Parking_Martin_Demel_ITAI2374.pptx
│   └── README.md
└── README.md (this file)
```

---

## 📚 Contents

### Assignment 1: Robotics Sensors
- **File:** `Assignment1 - Robotics Sensors _DEMEL.docx`
- **Description:** Comprehensive assignment covering various robotics sensors, their applications, and technical specifications.

### Assignment 1: Glossary
- **File:** `Assignment1_GLOSSARY_DEMEL.docx`
- **Description:** A detailed glossary of robotics terminology and concepts covered in the course.

### Raspberry Pi 5 Hardware Presentation
- **Files:**
  - `Raspberry Pi5 HW_Demel_Musil.pptx` - Presentation on Raspberry Pi 5 hardware
  - `raspberry-pi-5-product-brief.pdf` - Official product documentation

- **Description:** Hardware presentation exploring the Raspberry Pi 5's specifications, capabilities, and applications in robotics projects.

### Final Project: Autonomous Parking System
- **Directory:** `final/`
- **Main File:** `auto_park_between_squares.py`
- **Presentation:** `AUtonomus Parking_Martin_Demel_ITAI2374.pptx`

#### Project Description
A vision-based autonomous parking system implemented on the SunFounder PiCar-X robot using Raspberry Pi 5. The system uses computer vision with OpenCV and Picamera2 to detect two black square markers on the floor and autonomously parks the robot centered between them.

**Key Features:**
- Real-time computer vision processing using Picamera2
- Marker detection and tracking using OpenCV
- Proportional steering control for precise navigation
- Distance estimation based on marker size
- Safety features and graceful error handling

**Technologies Used:**
- Python 3
- OpenCV (cv2)
- NumPy
- Picamera2
- Raspberry Pi 5
- SunFounder PiCar-X Robot Platform

For detailed setup instructions, requirements, and usage, see the [final project README](./final/README.md).

---

## 🛠️ Technologies & Tools

- **Hardware:**
  - Raspberry Pi 5
  - SunFounder PiCar-X Robot Kit
  - Picamera2 Camera Module
  - Ultrasonic Sensors

- **Software:**
  - Python 3
  - OpenCV
  - NumPy
  - Picamera2
  - Raspberry Pi OS (Bookworm)
  - libcamera-apps

- **Development Environment:**
  - SSH/VNC Remote Access
  - Git Version Control
  - Visual Studio Code / Cursor IDE

---

## 🚀 Getting Started

### Prerequisites
To run the projects in this repository, you'll need:

1. **Hardware:**
   - Raspberry Pi 5 (or compatible Raspberry Pi model)
   - SunFounder PiCar-X robot kit
   - MicroSD card (16GB+ recommended)
   - Power supply for Raspberry Pi

2. **Software:**
   ```bash
   # Update system packages
   sudo apt update && sudo apt upgrade -y
   
   # Install required Python packages
   sudo apt install -y python3-opencv python3-numpy python3-picamera2 libcamera-apps
   
   # Install PiCar-X libraries (follow SunFounder documentation)
   ```

### Clone This Repository
```bash
git clone https://github.com/YOUR_USERNAME/ITAI-2374-Robots.git
cd ITAI-2374-Robots
```

---

## 📖 Course Learning Objectives

Through this course, the following competencies were developed:

1. **Sensor Technology:** Understanding and implementing various robotics sensors including ultrasonic, infrared, and camera-based systems
2. **Computer Vision:** Real-time image processing and object detection for autonomous navigation
3. **Embedded Systems:** Programming and interfacing with Raspberry Pi hardware
4. **Control Systems:** Implementing proportional control algorithms for robot steering
5. **Python Programming:** Advanced Python programming for robotics applications
6. **Problem Solving:** Debugging and troubleshooting hardware/software integration issues

---

## 🎯 Key Projects Highlights

### Autonomous Parking System
The flagship project demonstrates practical application of:
- Computer vision algorithms
- Real-time image processing
- PID control concepts (proportional steering)
- Safety-critical systems design
- Marker-based navigation

**Performance Metrics:**
- Processing Speed: ~30 FPS
- Parking Accuracy: Centered within 6% tolerance
- Detection Range: Variable based on marker size and lighting

---

## 📝 License

This repository contains academic coursework. The code is provided for educational purposes and portfolio demonstration. Feel free to use the code as reference material for your own learning, but please maintain academic integrity by not directly copying for course submissions.

---

## 👤 Author

**Martin Demel**  
Houston Community College  
ITAI 2374 - Robots, Fall 2025

---

## 🙏 Acknowledgments

- Houston Community College - Department of Information Technology
- SunFounder for PiCar-X documentation and libraries
- Raspberry Pi Foundation for comprehensive hardware documentation
- OpenCV community for computer vision resources

---

## 📧 Contact

For questions or collaboration opportunities, please reach out through GitHub.

---

*Last Updated: December 2025*

