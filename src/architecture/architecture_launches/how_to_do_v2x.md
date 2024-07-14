# 1. Connect properly to pi
You need to make sure that the vehicle is connect to the Pi over ROS. After you've connected
over WiFi, run `hostname -I` to find out what your Pi-connected IP address is. If you run
`rosswitch` to set your Pi configuration, and your local IP doesn't match what `hostname`
gave you, run `export ROS_IP=<whatever hostname gave you>` to get those to match. Run
`rostopic list` to make sure this worked; if the Pi is on, you should at least see the topics
`/rosout` and `/rosagg`.

# 2.a. If you're doing lane following

# 2.b. If you're doing V2X
After sourcing your `setup.bash` file, you'll need to launch the following:

```roslaunch architecture_launches v2x.launch```

There are a couple of launch arguments you need to keep in mind. Remember that setting these
arguments at launch is as simple as `<argument name>:=<value>`:

* `vehicle_namespace` (default is `sim`): The name of the vehicle you are running. These
are valid argument values:
    * `sim`: Run it in gazelle.
    * `actor1`: Run it in the ACTor 1 vehicle.
    * `actor2`: Run it in the ACTor 2 vehicle.
- `lane_detector` (default is `kmeans`): The type of lane detector to use. These are valid
argument values:
    - `kmeans`: Uses kmeans clustering to find lanes
    - `dbscan`: A lane detector that uses DBSCAN clustering.
    - `largest_contour`: uses the largest white contour to the right and offsets to follow
    the lane.
    - `birdseye`: a LSRL lane detector (**NOTE:** This lane detector *requires* that you use the `passthrough` preprocessor.)
- `preprocessor` (default is `easy_birdseye`): The type of image preprocessor to use; the
image is sent to a lane detector. These are valid argument values:
    - `easy_birdseye`: A full-featured preprocessor with two sliders for adjusting birdseye.
    - `full`: A full-featured preprocessor with total warp perspective control (kind of a lot)
    - `passthrough`: A dummy node; simply passes the unedited image through to the lane detector.
- `redlight_behavior` (default is `adaptive`): The behavior of the vehicle when it encounters
a redlight. These are valid argument values:
    - `adaptive`: Vehicle adjusts its speed in advance of the intersection.
    - `stop`: Vehicle simply stops at the intersection if its red.
- `lane_name` (default is `northbound`): The name of the lane you are driving in. These are
valid argument values:
    - `northbound`: Inner lane.
    - `eastbound`: Outer lane.
- `measure_comfort` (default is `false`): Whether to measure comfort in a rosbag (acceleration,
distance, and velocity). These are valid argument values:
    - `false`: Do not measure comfort.
    - `true`: Measure comfort.
