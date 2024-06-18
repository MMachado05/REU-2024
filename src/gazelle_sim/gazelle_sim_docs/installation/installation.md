Installation
------
This section will review the process of installing GazelleSim and optional GazelleSim model example for Ubuntu 20.04 running ROS Noetic.

### Step 1: Create ROS workspace (skip this step if you already have a workspace to install GazelleSim)

From your home directory,
```
mkdir catkin_ws
cd catkin_ws
mkdir src
catkin build
```

### Step 2: Clone the GazelleSim repository
From your catkin_ws directory,
```
cd src
git clone git@github.com:gderose2/gazelle_sim.git
cd ..
```

### Step 3: Build GazelleSim and example models
From your catkin_ws directory,
```
catkin build
source devel/setup.bash
```

Next: [Getting Started](../getting_started/getting-started.md)

<!--  LocalWords:  GazelleSim ROS
 -->
