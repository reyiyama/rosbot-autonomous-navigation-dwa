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

### Importance of Ground Robotics, Computer Vision, and IMAV-Inspired Navigation

<img width="744" alt="Screenshot 2024-10-15 at 2 18 45 AM" src="https://github.com/user-attachments/assets/2c97e085-dcd0-42bd-82cb-0b8211f943d4">


**Ground Robotics**, **Computer Vision**, **SLAM**, and other autonomous robotics technologies are at the forefront of modern advancements, particularly in autonomous vehicles and unstructured environment navigation. This project draws heavily from the **IMAV 2024 UAV competition** tasks, which focus on search, navigation, and visual localization in dynamic environments. Adapting these aerial challenges to a ground-based robot platform, like the **ROSbot 2 Pro**, highlights the versatility of ground robotics and extends the benefits of object detection, obstacle avoidance, and navigation to terrestrial robots.

1. **Computer Vision**: Leveraging **Aruco markers** as fiducial points, the robot interprets its environment visually and performs actions based on detection. This capability is crucial not only in aerial drones but also in ground-based platforms where markers provide reliable positioning and task coordination. The ROSbot 2 Pro, equipped with camera systems, can track these markers, which facilitates tasks such as obstacle navigation and localized interactions. By extending this to a ground context, it showcases how computer vision enhances real-time object recognition, localization, and operational tasks—central to industries like autonomous driving, logistics, and robotics.

2. **SLAM (Simultaneous Localization and Mapping)**: SLAM techniques allow robots to map unknown environments while simultaneously determining their location within them. In competitions like IMAV, SLAM is key for UAVs to navigate and localize in dynamic spaces. For ground robots like the ROSbot 2 Pro, SLAM becomes equally vital in building internal maps of environments, which aids in navigation and marker-based task execution. This is especially important in settings like indoor competition circuits, where environments are less structured.

3. **Dynamic Window Approach (DWA)**: The **Dynamic Window Approach** plays a central role in the project’s navigation system. In environments filled with obstacles (as in the IMAV competition), DWA enables the robot to efficiently compute collision-free trajectories in real time. Ground robots, especially wheeled platforms like the ROSbot 2 Pro, benefit from DWA due to its suitability for real-time navigation in dynamic, cluttered spaces. This method enhances the robot's adaptability, ensuring it can safely navigate both structured and unstructured environments while adhering to its non-holonomic constraints.

4. **Ground Robotics and Extended Applications**: While UAVs dominate aerial navigation tasks, ground robots like the ROSbot 2 Pro offer distinct advantages in operational time, payload handling, and ground-based interaction. By adapting aerial competition tasks to ground-based challenges, the ROSbot demonstrates how ground robotics can handle complex tasks such as sample collection, exploration, and interaction. This is particularly relevant to fields like agriculture, search and rescue, infrastructure monitoring, and wildlife conservation, where extended interaction with the environment is crucial.

---

### Relevance of Exploring These Mechanisms:
- **Self-Driving Vehicles**: The integration of SLAM, computer vision, and obstacle avoidance, as demonstrated in this project, is fundamental to the advancement of self-driving vehicles. Such systems must continuously interpret their environments and make real-time decisions to ensure safe and efficient navigation.
  
- **Search and Rescue Operations**: Autonomous ground robots equipped with the ability to navigate unstructured environments and identify fiducial markers can play a significant role in search and rescue missions, where rapid localization of individuals or objects is crucial.

- **Industrial and Agricultural Applications**: Autonomous robots capable of navigating large, complex environments can enhance operational efficiency in sectors like agriculture and manufacturing. These robots can be used for inventory management, precision farming, and infrastructure inspection, reducing human intervention and improving productivity.

By adapting the challenges posed by the **IMAV 2024 competition** to a ground-based robot like the **ROSbot 2 Pro**, this project contributes to the development of more intelligent and efficient ground robots capable of handling increasingly complex tasks in diverse environments. Below is a diagram of the original IMAV 2024 indoor circuit rules:

<img width="808" alt="Screenshot 2024-10-15 at 2 19 50 AM" src="https://github.com/user-attachments/assets/0df87831-2849-425d-b895-25fc9a716bec">


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

### Methodology, Implementation, and Software Architecture

This project focuses on developing an autonomous navigation system for the **ROSbot 2 Pro**, leveraging ROS 2 for navigating maze-like environments, avoiding obstacles, detecting **Aruco markers**, and performing specific marker-based actions. The project integrates two primary components: the **DWANavigator** node and the **ArucoMarker** node. These components control the ROSbot's movement using the **Dynamic Window Approach (DWA)** for navigation and obstacle avoidance while incorporating marker-based interaction.

#### Key Components:
- **DWANavigator Node**: Employs the **DWA** algorithm to navigate dynamic environments and avoid obstacles. It subscribes to topics such as `/scan` for laser scan data and `/objects` for marker information.
- **ArucoMarker Node**: Detects **Aruco markers** and executes actions based on the marker IDs, such as turning or moving to specific positions.
<img width="656" alt="Screenshot 2024-10-15 at 2 11 13 AM" src="https://github.com/user-attachments/assets/884987ab-b89f-4f70-85de-2f41f4273919">


#### Software Architecture:
The software follows a **three-tiered architecture** that separates high-level decision-making, sequencing, and hardware interaction:

<img width="1077" alt="Screenshot 2024-10-09 at 11 27 22 AM" src="https://github.com/user-attachments/assets/c788ba69-25a9-4d91-97fd-01085ce408f7">

1. **Deliberative Layer**:
   - **High-Level Decision Making**: This layer, represented by the **DWANavigator class**, gathers sensor data (e.g., odometry, laser scans) and makes decisions on the robot's navigation goals and actions.
   - **Marker Handling**: The `handle_markers()` method processes detected markers and determines the robot's response based on the marker ID, such as spinning or returning to a prior position.
   - **Navigation**: The **navigate()** method plans navigation and returns the robot to its starting position if necessary, while the **return_to_position()** method uses the ROS2 Navigation stack to navigate to specific locations.

2. **Sequencing Layer**:
   - **Action Breakdown**: The sequencing layer translates high-level decisions into specific robot actions. Methods like `spin_360()`, `turn_right()`, and `avoid_obstacle()` execute sequences of movements, ensuring the robot follows the planned path.
   - **Obstacle Avoidance and Recovery**: Functions such as `avoid_obstacle()` handle real-time obstacle avoidance, and `recover_from_stuck()` assists the robot in escaping tight or stuck situations.

3. **Reactive Layer**:
   - **Direct Hardware Interaction**: This layer interacts directly with the robot's actuators and sensors. The `/cmd_vel` publisher sends velocity commands to the robot, while subscribers for `/odom`, `/scan`, and `/objects` process sensor data in real time.
   - **Real-Time Sensor Processing**: The robot continuously adjusts its actions based on the latest sensor readings, ensuring responsive and adaptive behavior in dynamic environments.

<img width="1559" alt="Screenshot 2024-10-15 at 2 08 56 AM" src="https://github.com/user-attachments/assets/6797096d-1cb9-4e37-a343-71fbc0bda73a">


#### Sense-Plan-Act Cycle:
The project loosely follows a **Sense-Plan-Act** architecture:


<img width="976" alt="Screenshot 2024-10-09 at 11 29 38 AM" src="https://github.com/user-attachments/assets/16b6c89b-2380-44b4-9ea2-afb51b956297">

- **Sense**: Sensors (e.g., laser scanner, odometry, marker detection) gather environmental data, which is processed by subscribers like `odom_callback()` and `scan_callback()`.
- **Plan**: The **Deliberative Layer** plans actions based on the sensed data, using methods like `handle_markers()` to decide on marker-specific behaviors or obstacle avoidance strategies.
- **Act**: The robot executes the planned actions, publishing velocity commands to move through the environment or interact with detected markers.

<img width="1417" alt="Screenshot 2024-10-09 at 11 28 41 AM" src="https://github.com/user-attachments/assets/9641491f-49b8-40b3-9864-004121fb216e">

#### Dynamic Window Approach (DWA):
DWA, chosen for its suitability in fast-paced, dynamic environments, generates safe and efficient velocity commands based on the robot's sensor inputs and dynamics. This allows the robot to navigate complex environments with real-time obstacle avoidance.

<img width="1638" alt="Screenshot 2024-10-15 at 2 09 51 AM" src="https://github.com/user-attachments/assets/bb81bda6-b73e-40c1-83a0-33ebb59a9a96">

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
