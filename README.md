This project modifies the Carla driving simulation software in an Unreal Engine build to allow for realistic human driving within a simulated environment. It is designed to connect the Eleetus Blue Tiger and Simcraft Apex motion simulator platforms to allow for human driving within simulation. Notably, under the "PythonAPI\Chitsein-SmartCitiesREU-Scripts" folder are all the Python scripts and executables used to create these platforms. Additionally, a multiagent LSTM motion prediction algorithm was constructed to estimate trajectories of human drivers in real-time for later use in an intersection manager system.

This project was conducted during the NSF-funded Smart Cities Research Experience for Undergrads (REU) program at the University of Nevada, Las Vegas during Summer 2023.

Information about the Carla platform:
CARLA is an open-source simulator for autonomous driving research. CARLA has been developed from the ground up to support development, training, and
validation of autonomous driving systems. In addition to open-source code and protocols, CARLA provides open digital assets (urban layouts, buildings,
vehicles) that were created for this purpose and can be used freely. The simulation platform supports flexible specification of sensor suites and
environmental conditions.

For more information about Carla or the Unreal Engine build, please check out their websites. For more information about the steps to set up the build, check outthe link below.
Carla and Unreal Engine for Windows build: https://carla.readthedocs.io/en/latest/build_windows/
