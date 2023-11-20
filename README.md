# Realistic Human Driving Motion Platform Connected with Carla Driving Simulation and Autonomous Driving for Human-Computer Interaction

## Description
This project modifies the Carla driving simulation software in an Unreal Engine build to allow for realistic human driving within a simulated environment. It is designed to connect the Eleetus Blue Tiger and Simcraft Apex motion simulator platforms to allow for human driving within simulation. Notably, under the "PythonAPI\Chitsein-SmartCitiesREU-Scripts" folder are all the Python scripts and executables used to create these platforms. Additionally, a multiagent physics-based motion prediction algorithm was constructed to estimate trajectories of human drivers in real-time for later use in an intersection manager system. The Real-Time Intelligent Systems lab at UNLV will use this platform to help study interactions between human and autonomous drivers, and plans to develop infrastructure-based Vehicle to Everything (V2X) systems to aid autonomous vehicles in identifying and predicting motion of agents occluded from the vehicle's vantage point. 

This project was conducted during the NSF-funded Smart Cities Research Experience for Undergrads (REU) program at the University of Nevada, Las Vegas during Summer 2023.

Note that due to the specific hardware systems used within this project, replication of this project is only possible with the same motion platforms and their most up-to-date drivers. However, to replicate the system within another motion simulator platform, within the manual control python scripts (e.g. "manual_control_Simcraft.py") the Pygame.joystick inputs must be replaced such that the ID numbers correspond to their respective steering wheel, pedal, or button variables. Motion simulation will require more work with the specific motion simulator platform's API, and the ego vehicle's translational and rotational acceleration, velocity, and position can be accessed through the carla package within the python scripts. Before running any python scripts, be sure to install the correct packages using "pip install -r requirements.txt" from the base folder of whichever set of scripts you are using.

Information about the Carla platform:
CARLA is an open-source simulator for autonomous driving research. CARLA has been developed from the ground up to support development, training, and
validation of autonomous driving systems. In addition to open-source code and protocols, CARLA provides open digital assets (urban layouts, buildings,
vehicles) that were created for this purpose and can be used freely. The simulation platform supports flexible specification of sensor suites and
environmental conditions.

For more information about Carla or the Unreal Engine build, please check out their websites. For more information about the steps to set up the build, check outthe link below.
Carla and Unreal Engine for Windows build: https://carla.readthedocs.io/en/latest/build_windows/
