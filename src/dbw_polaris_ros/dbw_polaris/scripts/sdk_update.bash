#! /bin/bash

# Perform an SDK Install
#./sdk_install.bash
bash <(wget -q -O - https://bitbucket.org/DataspeedInc/dbw_polaris_ros/raw/master/dbw_polaris/scripts/sdk_install.bash)

# Update with apt-get
#sudo apt-get update && sudo apt-get upgrade && rosdep update

# Detect and update legacy source installation
MY_WORKSPACE=$HOME/dbw_ws
if [ -e "$MY_WORKSPACE/devel/setup.bash" ]; then
  if [ -e "$MY_WORKSPACE/src/.rosinstall" ]; then
    if [ -e "$MY_WORKSPACE/src/dbw_polaris_ros/LICENSE" ]; then
      echo "Detected source installation. Updating..."
      source `find /opt/ros -name setup.bash | sort -r | head -1`
      wstool update -t $MY_WORKSPACE/src
      rosdep update && rosdep install -y -r --from-paths $MY_WORKSPACE/src --ignore-src
      cd $MY_WORKSPACE
      catkin_make -DCMAKE_BUILD_TYPE=Release
    fi
  fi
fi

echo "SDK update: Done"

