# Sanitizer Robot

## Autonomous and Mobile Robotics - Final Project
**University of Bologna**  
Department of Electrical, Electronic, and Information Engineering (DEI - LAR)  
Viale del Risorgimento 2, 40136 Bologna

---

## **Project Overview**
This project involves the implementation of an autonomous disinfection robot using **TurtleBot3** and **ROS** in a simulated or real environment. The goal is to make the robot navigate unknown environments, create maps, and optimize disinfection paths using **UV light**.

## **Tasks Overview**

### **Task 1: Environment Setup**
Students must set up the **TurtleBot3 Burger** robot in one of the following scenarios:
- **Simulation**: Using the Gazebo **TurtleBot3 Big House** scenario.
- **Real Environment**: The robot navigates an unknown, confined space with obstacles.

During the evaluation, the robot's starting position can be random, and additional obstacles may be present.

### **Task 2: Autonomous Mapping**
The robot must **autonomously explore** and **map** the environment:
- No prior knowledge of the environment is provided.
- Existing ROS packages can be utilized.
- Implementing an optimal mapping strategy is encouraged.

### **Task 3: Navigation to Target Goals**
After mapping, the robot must reach a set of goal points specified in a **text file**:
- Align the **map** with the **navigation system**.
- Implementing an optimal map alignment process is encouraged.

### **Task 4: UV-Based Disinfection**
The robot is equipped with **UV lamps** to sanitize the area. The UV energy at a point **(x, y)** at time **t** is given by:
\[
E(x, y, t) = \int_0^t \frac{P_l}{(x - p_x(\tau))^2 + (y - p_y(\tau))^2} d\tau
\]

#### **Key Considerations**:
- **UV Power**: \( P_l = 100 \mu W/m^2 \)
- **Obstacle Blocking**: UV propagation is blocked by obstacles.
- **Minimum Disinfection Requirement**: **10 mJ** over the entire room.
- **Grid Resolution**: The room is discretized into a **0.2m** resolution grid.
- **Path Planning**: The robot must follow an optimized path to ensure disinfection coverage.

---

## **Implementation Details**

### **Simulation Setup (Gazebo + ROS)**
- Simulate the TurtleBot3 Burger in **Gazebo Big House**.
- Define **rooms/areas** using tables.
- Implement ROS services/actions for:
  - **Lamp Control**
  - **Energy Computation**
  - **Task Updates**
  - **Sanitization Execution**

### **Path Generation Strategy**
Path planning consists of:
1. **Initial Path Planning**: Based on the shape of the room.
2. **Obstacle Avoidance**: Adjust the path to navigate around obstacles.

### **Grid Map Visualization**
- Grid-based maps are used to visualize the UV disinfection coverage.
- Heatmaps show energy distribution over the environment.

---

## **General Rules & Deliverables**
- The project can be developed in **simulation** or with a **real robot**.
- A **report (presentation/document)** must be presented.
- The **implemented code** must be provided separately.
- During the evaluation, students must **demonstrate** their results through:
  - **Simulations**
  - **Real-world experiments**
  - **Videos**
- A **clear project organization** and well-structured codebase are required.
- Available **ROS packages** can be used.

---

## **Installation & Requirements**
### **Prerequisites**
- **ROS (Robot Operating System)**
- **Gazebo Simulation**
- **TurtleBot3 Packages**
- **Python 3.x**
- **Matplotlib, NumPy**

### **Installation**
To install the required dependencies, run:
```sh
sudo apt update && sudo apt upgrade -y
sudo apt install ros-noetic-turtlebot3*
```

---

## **Usage Instructions**
1. **Setup the environment:**
   - Simulation: Launch the **Gazebo Big House** scenario.
   - Real Robot: Ensure TurtleBot3 is operational in a confined space.
2. **Run Mapping:** Execute autonomous exploration to create a map.
3. **Set Goal Points:** Provide a text file with target locations.
4. **Run Navigation:** Align the map and navigate to goals.
5. **Execute Sanitization:** Optimize the path for UV disinfection.

---

## **Results & Evaluation**
- Grid maps showing **sanitization coverage**.
- **Path planning analysis** and **optimization insights**.
- **Simulation/real-world performance comparison**.

---
---

### **Acknowledgments**
This project is part of the **Autonomous and Mobile Robotics** course at the **University of Bologna**.

