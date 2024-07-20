# V2X REU 2024 Repository

![image](https://github.com/user-attachments/assets/0280b4e2-2846-422b-a410-7883fd8d00ef)

Research Team:

- Marcial Machado
- Michael Evans
- Rickey Johnson
- Beñat Froemming-Aldanondo
- Tatiana Rastoskueva
- Luis Escamilla
- Anna Vadella

Graduate Assistants:

- Devson Butani
- Milan Jostes
- Ryan Kaddis

Principal Investigators:

- Dr. Chan-Jin Chung
- Dr. Joshua Siegel

## Testing and Launching

Ubuntu 20.04 "Focal Fossa" is recommended. ROS Noetic should be installed.

The following Python packages are required:

- Rospy
- Scikit learn
- OpenCV

### Launch configurations

There are two primary launch files used for testing and running the ACTor vehicles: `v2x.launch` and `follow_lane_one_car.launch`. These are the arguments available to change (those with \*stars *must* be set manually):

`follow_lane_one_car.launch`:
- `vehicle_namespace` (default: "robot1"): "robot1", "actor1", "actor2": This is the vehicle being run (the first option is to run Gazelle Sim).
- `preprocessor` (default: "easy_birdseye"): "easy_birdseye", "full", "passthrough", "crop_only": The initial image preprocessor.
- `lane_detector` (default: "dbscan"): "dbscan", "kmeans", "birdsdbs", "birdseye", "deeplsd", "kmeans", "largest_contour": The lane detection algorithm. **Note:** the "birds-" family of lane detectors *requires* the "passthrough" preprocessor.
- \*`lane_name` (default: "northbound"): "northbound", "eastbound": the H lot lane the car is driving in. The inner lane is northbound.

`v2x.launch`:
- `vehicle_namespace` (default: "sim"): "sim", "actor1", "actor2": This is the vehicle being run (the first option is to run Gazelle Sim).
- `preprocessor` (default: "easy_birdseye"): "easy_birdseye", "full", "passthrough", "crop_only": The initial image preprocessor.
- `lane_detector` (default: "kmeans"): "dbscan", "kmeans", "birdsdbs", "birdseye", "deeplsd", "kmeans", "largest_contour": The lane detection algorithm. **Note:** the "birds-" family of lane detectors *require* the "passthrough" preprocessor.
- \*`lane_name` (default: "northbound"): "northbound", "eastbound": the H lot lane the car is driving in. The inner lane is northbound.
- `redlight_behavior` (default: "adaptive"): "adaptive", "stop": What the car should do with regards to intersection states and redlights.
- `measure_comfort` (default: "false"): "false", "true": whether to collect comfort data. **Note:**: this only works in the actual ACTor vehicles.

### Launching the RSU

If launching the ACTor vehicles using the `v2x.launch` file, you will, at minimum, want to run the traffic-light node. Here is how to accomplish this:

1. Run the following command: `roslaunch traffic_light_pkg start_nonadaptive_ns_with_runner.launch`
2. When launching `v2x.launch` from the ACTor vehicles, ensure that the launch environment is connected to the same `roscore` as that on which the traffic light node is now running.
3. Optionally, if you wish to be able to dynamically reconfigure the traffic light (such as changing its mode or timing configurations), you may launch `roslaunch traffic_light_pkg start_nonadaptive_ns_control.launch` on a desired Linux system with a functioning desktop environment.
4. Optionally, if using Dr. Siegel's traffic light Arduino, simply turn on the light nearby the WiFi-enabled traffic light computer. The Arduino will automatically establish a connection to the light and begin echoing the state changes.

Because of ROS' WiFi connection capabilities, these launch files can be run on the same machine, or on a different machine, such as having the traffic light run on a WiFi-enabled Raspberry Pi.
