5-DOF Robotic Manipulator — ROS 2 & Gazebo Pick-and-Place Simulation
A CAD-designed five-degree-of-freedom (5-DOF) robotic manipulator modelled and simulated using ROS 2 and Gazebo. The current simulation demonstrates a pick-and-place task: the arm grasps an object at one location, transports it through a planned or commanded motion, and places it at a second location.
Project status: CAD design and ROS 2–Gazebo pick-and-place simulation have been developed. Features listed under Planned Improvements are future work and should not be interpreted as completed functionality.

<p align="center">
  <!-- Replace the paths below with screenshots/GIFs committed to this repository. -->
  <img src="docs/images/robot_overview.png" alt="5-DOF robotic manipulator in Gazebo" width="80%" />
</p>Project Overview
The aim of this project is to take a mechanical robot-arm design beyond a static CAD assembly and bring it into a robotics simulation environment. The manipulator is represented in Gazebo and controlled through ROS 2 to perform a basic object-transfer operation.

The project combines mechanical design, robot modelling, simulation and robotic control. It provides a foundation for studying the relationship between the arm's physical geometry, joint motion and end-effector movement.
Current Features
- 5-DOF manipulator design: CAD model of a five-axis robotic arm.
- ROS 2 integration: ROS 2 is used as the robotics software framework.
- Gazebo simulation: The manipulator and pick-and-place scenario are simulated in Gazebo.
- Object transfer demonstration: The arm picks up an object from one location and places it at another.
- Simulation-based development: Robot behaviour can be inspected and iterated without requiring a physical prototype.
Only retain a feature above if it is present and working in the version of the project you publish.
Technology Stack
Area	Technology
Mechanical design	CAD software — add the software and version used
Robotics middleware	ROS 2 — add distribution (e.g. Humble, Jazzy)
Robot simulation	Gazebo — add version / release
Robot description	Add the format actually used (URDF, Xacro, SDF, etc.)
Control	Add the controller or control interface actually used
Programming	Add languages actually used (e.g. Python, C++)


Demonstration
The current demonstration follows this sequence:
1. The simulation is launched and the robotic manipulator is loaded into Gazebo.
2. The object and target placement location are available in the simulated workspace.
3. The manipulator moves its end effector towards the object.
4. The gripper picks up the object.
5. The arm transfers the object to the second location.
6. The gripper releases the object.
Add a short GIF or video link here so visitors can see the complete sequence without having to build the project.
[Add demonstration video link]
Repository Structure
The exact layout depends on how the ROS 2 workspace is organised. A suggested structure is:
robotic-arm-ros2-gazebo/
├── README.md
├── docs/
│   ├── images/
│   │   ├── robot_overview.png
│   │   ├── gazebo_scene.png
│   │   └── pick_and_place_sequence.png
│   └── videos/
├── cad/                    # Native CAD files and exported geometry
├── ros2_ws/
│   └── src/                # ROS 2 packages
│       ├── <robot_description_package>/
│       ├── <simulation_package>/
│       └── <control_package>/
└── LICENSE
Update this tree to match the actual folders and package names in the repository. Do not leave placeholder package names in the final published README.
Getting Started
Prerequisites
Install and configure the versions used by this project:
- Ubuntu: [add tested version]
- ROS 2: [add distribution]
- Gazebo: [add version / release]
- ROS–Gazebo integration packages required by the selected versions
- Any additional ROS 2 packages used by the robot description and controllers
The ROS 2 and Gazebo versions matter because package names and integration instructions differ between Gazebo releases. Use the official installation instructions for the versions selected by this project.
Clone the repository
git clone <repository-url>
cd <repository-folder>
Build the ROS 2 workspace
If the repository contains a ROS 2 colcon workspace at ros2_ws/, build it with:
cd ros2_ws
source /opt/ros/<ros2-distro>/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
Replace <ros2-distro> with the tested ROS 2 distribution. If your repository uses a different workspace path or has no ros2_ws/ directory, adjust these commands accordingly.
Launch the simulation
Add the actual launch command used by your package. For example, the command will generally follow this pattern:
ros2 launch <simulation-package> <launch-file>.launch.py
Do not publish the example command unchanged: replace both placeholders with the real package and launch-file names. If the simulation is started directly through Gazebo or another script, document that exact procedure instead.
Mechanical Design and Robot Modelling
The manipulator is based on a five-joint mechanical design. In the final project documentation, include:
- An isometric CAD view of the complete assembly.
- An exploded view or joint-by-joint breakdown.
- Link dimensions and joint-axis locations.
- Joint type, movement range and mechanical limits for each axis.
- End-effector / gripper design.
- A description of how the CAD geometry was prepared for simulation.
Where available, include the relevant engineering calculations, such as link mass properties, payload assumptions, joint torque estimates and motor/gearbox selection. Clearly identify calculated, assumed and measured values.
Simulation and Control
Document the implementation that is actually present in the repository:
- How the robot model is loaded into Gazebo.
- How each joint is actuated.
- How the pick and place positions are specified.
- How the gripper is opened and closed.
- Which ROS 2 nodes, topics, services or actions are involved.
Useful evidence to include is a screenshot of the simulated robot, a ROS 2 node/topic graph, joint-state plots and a short recording of a complete pick-and-place cycle.
Results and Evaluation
The current project demonstrates a simulated pick-and-place operation. To make the project more technically complete, measure and report results rather than relying only on screenshots.
Metric	Result
Number of robot DOF	5
Pick-and-place operation	Demonstrated in simulation
Payload	To be measured / documented
End-effector positioning error	To be measured
Pick success rate	To be measured over repeated trials
Cycle time	To be measured
Collision test scenarios	To be documented


Do not fill the result cells with estimates unless they are clearly labelled as estimates. A useful evaluation should state the test conditions, number of trials and how each metric was measured.
Planned Improvements
The following are possible next steps. Mark an item as complete only after it has been implemented and tested.
- Derive and document the Denavit–Hartenberg (DH) parameters.
- Implement and verify forward kinematics (FK) and inverse kinematics (IK).
- Integrate MoveIt 2 for motion planning and collision checking, if not already used.
- Add obstacle scenarios and compare collision-free trajectories.
- Add a vision-based object localisation pipeline.
- Add object sorting by colour, shape or another measurable property.
- Benchmark positioning accuracy, repeatability, cycle time and pick success rate.
- If hardware is built, compare simulation results with physical measurements.
Engineering Considerations
A five-DOF arm cannot independently command every possible six-dimensional end-effector pose. The reachable positions and orientations depend on the joint arrangement, link geometry and joint limits. Define a feasible task workspace and choose grasp orientations that the manipulator can actually reach.

Simulation results also depend on the robot's mass and inertia properties, joint limits, friction, actuator limits and controller settings. Document these assumptions so that the simulation can be interpreted and reproduced.
