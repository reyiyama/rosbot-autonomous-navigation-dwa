### Autonomous Navigation and Object Tracking Project on ROSbot 2 PRO

![image](https://github.com/user-attachments/assets/2b653b59-3105-4ddb-9a19-445ca08acec9)


**Project Name**: Autonomous Ground Robot Navigation: Object Detection and Tracking with ROSbot 2 Pro Using Dynamic Window Approach and Computer Vision  
**Course**: Programming Autonomous Robots (COSC2781)  
**Group Name**: 03-XAI  
**Contributors**:  
- Andria Nicholas (S3995645)  
- Maryam Musallam Al-Howaiti (S3856144)  
- Amay Viswanathan Iyer (S3970066)  
- Basavaraj Somashekhar Sanshi (S3975993)

---

### Project Overview

This project explores the development of an autonomous navigation system for the **ROSbot 2 Pro** to perform object location, tracking, and interaction using computer vision and SLAM techniques. Inspired by the **IMAV 2024 UAV competition**, this project adapts UAV-based tasks for ground robots. The system autonomously navigates unknown environments, detects **Aruco markers**, and performs specific actions based on marker detection. It employs the **Dynamic Window Approach (DWA)** for obstacle avoidance, demonstrating the potential for ground-based robots in diverse autonomous robotics applications such as **self-driving vehicles**, **wildlife conservation**, **infrastructure inspection**, and **search and rescue**.

---

### Importance of Ground Robotics and Computer Vision

**Computer Vision**, **Ground Robotics**, **SLAM**, and other robotics mechanisms are pivotal technologies that drive advancements in **autonomous robotics** and **self-driving vehicles**. These technologies are essential because they enable robots and vehicles to understand and interact with their environments autonomously.

1. **Computer Vision**: By employing computer vision techniques, robots can detect and track objects, landmarks, and obstacles in real time. This is especially important for tasks like **object tracking**, **obstacle detection**, and **navigation**, which are essential in applications ranging from autonomous driving to industrial robots. In this project, the use of **Aruco markers** showcases how fiducial markers help robots understand their surroundings and perform specific tasks based on marker detection.

2. **SLAM (Simultaneous Localization and Mapping)**: SLAM is crucial for robots to build a map of unknown environments while simultaneously localizing themselves within that map. In the context of self-driving cars and autonomous robots, SLAM allows the vehicle or robot to navigate dynamic and unstructured environments. This project incorporates SLAM to enable the ROSbot 2 Pro to explore and map its surroundings, allowing it to perform navigation and tracking tasks autonomously.

3. **Dynamic Window Approach (DWA)**: DWA is a local path-planning algorithm that enables robots to dynamically avoid obstacles and plan optimal trajectories in real-time. For autonomous ground vehicles, this is essential for ensuring safety, efficiency, and adaptability in ever-changing environments.

4. **Ground Robotics**: Ground robots, like the ROSbot 2 Pro, play a significant role in applications where longer operational times and heavier payload capacities are required compared to aerial systems. The project's focus on ground robotics is relevant for industries like agriculture, search and rescue, and infrastructure inspection, where such robots can operate for extended periods in harsh environments and navigate complex terrains.

### Relevance of Exploring These Mechanisms:
- **Self-Driving Vehicles**: The integration of computer vision, SLAM, and obstacle avoidance is fundamental in self-driving technologies, where vehicles must continuously interpret their environment and make decisions in real-time to ensure safe operation.
- **Search and Rescue Operations**: In disaster zones or hazardous environments, autonomous ground robots equipped with object detection and mapping technologies can assist in locating survivors or assessing damage without putting human lives at risk.
- **Industrial and Agricultural Applications**: Robots that can autonomously navigate large, complex environments—such as warehouses, factories, or fields—can enhance efficiency and reduce human intervention, particularly in tasks like inventory management or precision farming.
  
By studying and implementing these technologies in autonomous robots, we contribute to the future of robotics where machines are more intelligent, efficient, and capable of handling increasingly complex tasks.

---

### Key Objectives
- Develop a navigation system for ROSbot 2 Pro to autonomously locate and interact with objects using **Aruco markers**.
- Implement efficient **obstacle avoidance** using the **Dynamic Window Approach (DWA)**.
- Incorporate **SLAM** for mapping environments and localizing the robot.
- Perform marker-specific actions upon detection.
- Showcase modular software architecture to enhance scalability and future adaptability.

---

### Project Architecture

1. **NavigationPublisher Node**:
   - Basic navigation system for following walls using obstacle distance data.

2. **DWANavigator Node**:
   - Advanced navigation system implementing **Dynamic Window Approach** (DWA) for real-time obstacle avoidance and efficient trajectory planning. The system uses laser scan data for distance measurements and integrates with ROS2 topics (`/scan`, `/objects`).
   - The node controls the robot's movement and detects Aruco markers to perform specific actions.

3. **ArucoMarker Node**:
   - Implements **Aruco marker detection** using the **Find Object 2D** package.
   - Upon detecting markers, the robot performs pre-programmed actions like turning or moving forward.

---

### Methodology

#### Three-Tiered Software Architecture
This architecture enables clean separation of the robot's functionalities:
- **Deliberative Layer**: Responsible for high-level decision-making and planning using the **DWANavigator class**. This layer gathers sensor data from topics like `/odom` and `/scan` to plan the robot’s goals and determine actions based on marker detection.
  
- **Sequencing Layer**: Breaks down high-level commands into sequences of specific actions, such as turning, moving forward, and avoiding obstacles. This layer ensures robust navigation and recovery from stuck scenarios using methods like `avoid_obstacle()` and `recover_from_stuck()`.
  
- **Reactive Layer**: Direct interaction with the robot’s actuators, sending velocity commands to the robot based on sensor data. Real-time responses to the environment are handled here, ensuring smooth and reactive motion.

<img width="1077" alt="Screenshot 2024-10-09 at 11 27 22 AM" src="https://github.com/user-attachments/assets/c788ba69-25a9-4d91-97fd-01085ce408f7">

#### Sense-Plan-Act Architecture
Our system incorporates elements of the **Sense-Plan-Act (SPA)** architecture. It collects data from sensors (`/scan`, `/odom`), processes it in the planning layer, and executes behaviors using the actuation layer, providing a robust method for real-time decision-making in dynamic environments.

<img width="1417" alt="Screenshot 2024-10-09 at 11 28 41 AM" src="https://github.com/user-attachments/assets/9641491f-49b8-40b3-9864-004121fb216e">

#### Dynamic Window Approach (DWA)
DWA is the core navigation algorithm that optimizes velocity commands based on the robot’s sensor inputs, enabling smooth navigation and obstacle avoidance. This approach is ideal for environments with dynamic obstacles, as it continuously updates the robot's path.

<img width="976" alt="Screenshot 2024-10-09 at 11 29 38 AM" src="https://github.com/user-attachments/assets/16b6c89b-2380-44b4-9ea2-afb51b956297">

---

### Key Results

1. **Aruco Marker Detection and Interaction**:
   - The system successfully detects all 7 Aruco markers in a maze-like environment and executes corresponding actions.
   - The robot returns to the previous position upon marker re-detection, as demonstrated in our results.

2. **DWA-Based Navigation**:
   - The **DWANavigator** achieves a 90% success rate in both **obstacle avoidance** and **navigation**.
   - Compared to a wall-following algorithm, DWA provides smoother and more efficient navigation, leveraging dynamic obstacle avoidance.

<img width="1109" alt="Screenshot 2024-10-09 at 11 31 31 AM" src="https://github.com/user-attachments/assets/a0182819-e010-4d72-8de5-ffb175679290">

---

### Relevance to Autonomous Robotics and Applications
This project demonstrates the capabilities of autonomous ground robots, particularly in:
- **Self-driving vehicles**: Similar path-planning, sensor fusion, and decision-making principles can be applied to autonomous cars.
- **Computer vision**: The integration of **fiducial marker detection** aligns with the latest trends in vision-based navigation systems for drones, robots, and autonomous vehicles.
- **SLAM**: The project highlights the use of **SLAM** for mapping and localization, which is critical for autonomous exploration in dynamic environments.
- **Search and rescue**: Ground-based robots with object tracking and obstacle avoidance capabilities can play a significant role in real-time rescue operations in hazardous environments.
  
---

### Detailed Directory Structure Explanation

Here's an explanation of the main components of the `aruco_detector` node:

#### 1. `aruco_detector/` (Root Directory)
   This directory contains the core implementation of the Aruco detection and navigation system.

#### 2. **Files in `aruco_detector`**:
   - `__init__.py`: Marks this directory as a Python package.
   - `aruco.py`: Contains the main Aruco marker detection logic, responsible for identifying fiducial markers in the robot's environment.
   - `aruco_generator.py`: A utility script for generating custom Aruco markers.
   - `aruco_marker_action.py`: Defines specific actions for the robot to take when a marker is detected (e.g., stopping, turning, etc.).
   - `dwa_navigator.py`: Implements the **Dynamic Window Approach (DWA)** for real-time obstacle avoidance and navigation.
   - `navigate.py`: Handles the basic navigation logic for the robot, including movement and simple obstacle avoidance.

#### 3. **Launch Directory** (`launch/`):
   - `exploration.launch.py`: A launch file that initializes the ROS2 nodes required for the project. This file sets up the navigation, Aruco marker detection, and other essential nodes.

#### 4. **Resource Directory** (`resource/`):
   - `objects/`: A directory for storing any resources or objects related to the project, such as images or 3D models.
   - `aruco_detector/`: The main resource folder related to the Aruco marker detection system.

#### 5. **Test Directory** (`test/`):
   - `test_copyright.py`, `test_flake8.py`, `test_pep257.py`: These files are for testing the codebase and ensuring it adheres to copyright rules and coding standards (e.g., PEP257).

#### 6. **Other Project Files**:
   - `CMakesList.txt`: Contains the configuration for building the ROS2 packages using CMake.
   - `package.xml`: Describes the ROS package, listing its dependencies and other metadata.
   - `setup.py` and `setup.cfg`: Configuration files for setting up the Python package.

---

### Prerequisites Before Running

Before running the project, ensure the following prerequisites are in place:

1. **ROS2 Humble**: This project is designed for ROS2 Humble, which is run using Docker on the ROSBot 2 Pro. Make sure you have the necessary Docker setup to run ROS2 on the ROSBot.
   
2. **Docker and AIIL Setup**:
   - The ROSBot uses pre-configured Docker containers to handle the ROS2 stack. Ensure that the Docker containers are correctly set up.
   - Run the script to start the ROSBot stack:
     ```bash
     ~/ros_driver_start.sh all
     ```

3. **AIIL Docker Container**:
   - You will be using a custom Docker container provided by the AIIL repository. To start this container and access the ROS2 environment:
     ```bash
     ~/docker_aiil.sh
     ```

4. **Source ROS Setup Script**:
   - After the Docker container is running, navigate to the `/ros_ws` directory where the ROS workspace is located. Before building and running nodes, you must source the ROS setup file:
     ```bash
     source install/setup.bash
     ```

5. **Colcon Build**:
   - Build the ROS2 workspace inside the Docker container using `colcon`:
     ```bash
     colcon build
     ```

6. **Set ROS Domain**:
   - Set the ROS Domain ID for the specific ROSBot you are using. This step ensures that your ROS2 nodes can communicate with the robot:
     ```bash
     set_ros_domain <robot_name>
     ```
   - Example:
     ```bash
     set_ros_domain tardis
     ```

---

### Updated Steps to Run the Project

Once all the prerequisites are in place, follow these steps to run the project:

1. **Clone the Repository**:
   If you haven't already, clone the repository to your workspace:
   ```bash
   git clone https://github.com/reyiyama/rosbot-autonomous-navigation-dwa/
   ```

2. **Start the ROSBot Software Stack**:
   On the ROSBot 2 Pro, start the ROS software stack:
   ```bash
   ~/ros_driver_start.sh all
   ```

3. **Launch AIIL Docker Container**:
   To access the ROS2 environment, enter the Docker container prepared for this project:
   ```bash
   ~/docker_aiil.sh
   ```

4. **Navigate to Workspace**:
   Inside the Docker container, navigate to the workspace directory:
   ```bash
   cd /ros_ws
   ```

5. **Build the Project**:
   Build the ROS packages using `colcon`:
   ```bash
   colcon build
   ```

6. **Source Setup File**:
   After building, source the setup file to make your newly built nodes available:
   ```bash
   source install/setup.bash
   ```

7. **Set ROS Domain**:
   Set the ROS domain for the specific ROSBot you're working with (e.g., "tardis"):
   ```bash
   set_ros_domain tardis
   ```

8. **Launch the Exploration Nodes**:
   Launch the necessary ROS nodes for navigation and Aruco marker detection:
   ```bash
   ros2 launch aruco_detector exploration.launch.py
   ```

9. **Run Aruco Marker Detection**:
   To test the Aruco marker detection system and execute the associated actions:
   ```bash
   ros2 run aruco_detector aruco_marker_action
   ```

---

### Future Improvements

- **Enhanced SLAM Techniques**: Implement more advanced SLAM algorithms for better mapping and localization.
- **Optimized Marker Detection**: Explore better computer vision models or neural networks for more accurate marker detection and interaction.
- **Machine Learning for Obstacle Avoidance**: Incorporate reinforcement learning techniques to make the robot's navigation system smarter and more adaptable to dynamic environments.

---

### Acknowledgments
- **IMAV 2024 UAV competition** provided the inspiration for the challenges adapted in this project.
- Various research on object detection, path planning, and SLAM contributed to the development of our algorithms.

---

This project exemplifies how autonomous ground robots can achieve real-time navigation and object tracking, showcasing the potential of ROSbot 2 Pro in advanced robotics applications such as self-driving cars, search and rescue, and industrial robotics.
