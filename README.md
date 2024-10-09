<img width="1109" alt="Screenshot 2024-10-09 at 11 30 42 AM" src="https://github.com/user-attachments/assets/1dbf9735-6e10-4548-aafc-f8c411aa222f">### README for Autonomous ROSbot Navigation Project

**Project Name**: Approximate Object Location and Tracking with UAVs on ROSbot 2 Pro  
**Course**: Programming Autonomous Robots (COSC2781)  
**Group Name**: 03-XAI  
**Contributors**: 
- Andria Nicholas (S3995645)
- Maryam Musallam Al-Howaiti (S3856144)
- Amay Viswanathan Iyer (S3970066)
- Basavaraj Somashekhar Sanshi (S3975993)

---

### Project Overview:
This project focuses on developing an autonomous navigation system for the **ROSbot 2 Pro**. The system is designed to navigate through an unknown, dynamic environment, detect Aruco markers, and perform specific actions based on detected markers. The project is inspired by the **IMAV 2024 UAV competition**, but adapts the challenges to a ground-based platform.

**Key Objectives**:
- Enable ROSbot 2 Pro to autonomously locate and track objects.
- Implement obstacle avoidance using the **Dynamic Window Approach (DWA)**.
- Incorporate **Simultaneous Localization and Mapping (SLAM)** for environment mapping.
- Perform marker-specific actions when an Aruco marker is detected.
- Demonstrate modular software architecture to support future scalability and adaptability.

---

### Project Structure:
1. **NavigationPublisher Node**:
   - Originally implemented as a basic navigation system that controls the ROSbot’s movements and follows walls based on proximity to obstacles.

2. **DWANavigator Node**:
   - Advanced version of the navigation system using the **Dynamic Window Approach (DWA)**.
   - Integrated with ROS2 topics (`/scan`, `/objects`) to enable dynamic, obstacle-avoiding navigation.
   - Can detect Aruco markers and execute specific actions (e.g., turn, move forward).

3. **ArUcoMarker Node**:
   - Handles Aruco marker detection using the **Find Object 2D** package.
   - Executes marker-specific behaviors upon detection.

---

### Methodology:
- **Three-Tiered Software Architecture**: 
   - **Deliberative Layer**: High-level decision-making and planning using DWANavigator.
   - **Sequencing Layer**: Converts high-level commands into actionable steps (e.g., spin, move forward).
   - **Reactive Layer**: Direct control of the robot’s motors and real-time response to sensor data.
<img width="1077" alt="Screenshot 2024-10-09 at 11 27 22 AM" src="https://github.com/user-attachments/assets/c788ba69-25a9-4d91-97fd-01085ce408f7">


- **Sense-Plan-Act Architecture**:
   - ROSbot follows a sense-plan-act cycle, where sensor data is gathered, decisions are made, and actions are executed. This process is concurrent, enabling real-time navigation and obstacle avoidance.
<img width="1417" alt="Screenshot 2024-10-09 at 11 28 41 AM" src="https://github.com/user-attachments/assets/9641491f-49b8-40b3-9864-004121fb216e">


- **Dynamic Window Approach (DWA)**:
   - The primary navigation algorithm employed for obstacle avoidance and path planning. It calculates the optimal velocity commands by analyzing the environment and robot dynamics.
<img width="976" alt="Screenshot 2024-10-09 at 11 29 38 AM" src="https://github.com/user-attachments/assets/16b6c89b-2380-44b4-9ea2-afb51b956297">

---

### Key Results:
- The ROSbot successfully navigates maze-like environments, detects all 7 Aruco markers, and performs the required actions for each.
<img width="1109" alt="Screenshot 2024-10-09 at 11 31 31 AM" src="https://github.com/user-attachments/assets/a0182819-e010-4d72-8de5-ffb175679290">


- The **DWA-based navigator** proved superior to the previous wall-following algorithm, with:
   - 90% success rate in obstacle avoidance.
   - 90% success rate in navigating to goals.
- Markers are detected with high accuracy, and the ROSbot interacts with them based on pre-programmed actions.

---

### File Structure:
- `navigation_publisher.py`: Initial implementation of the wall-following navigation.
- `dwa_navigator.py`: Advanced navigation using the Dynamic Window Approach.
- `aruco_marker.py`: Marker detection and action execution.
- `launch/`: Contains ROS launch files for running the nodes together.
- `data/`: Sample data files and logs from testing.

### How to Run:
1. Clone the project and navigate to the root directory.
2. Launch the simulation with:  
   `ros2 launch rosbot_navigation simulation_launch.py`
3. Ensure that the ROSbot is subscribed to the necessary topics for marker detection and navigation.
4. Test the Aruco marker detection using the provided environment by executing:  
   `ros2 run rosbot_navigation aruco_marker_node`

---

### Future Improvements:
- Integration of **advanced SLAM techniques** to enhance map creation.
- Improve marker pointing functionality using enhanced vision-based algorithms.
- Incorporate **machine learning techniques** for better obstacle recognition and decision-making.

---

### Acknowledgments:
- IMAV 2024 UAV competition rules provided the foundation for the challenges in this project.
- Research contributions from various authors on object detection, SLAM, and navigation, referenced throughout the report.

---

This project demonstrates the potential of ground-based autonomous robots in dynamic environments, showcasing the capabilities of the ROSbot 2 Pro for object location, tracking, and interaction.
