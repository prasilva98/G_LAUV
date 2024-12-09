# G_LAUV

This repository shall provide all the the tools needed to integrate a LAUV-like vehicle into the [MBARI's Gazebo simulator](https://github.com/osrf/lrauv). 

# Replay Logs
At this point it only includes the ```replay_logs``` package which is used to replay the actuators given to a LAUV log during a real-life operation, into a Gazebo maritime simulation. To achieve this, you need to install a Gazebo version that includes the necessary plugins (Buoyancy, Hydrodynamics, etc), the ROS_GZ package and the necessary actuator data. 

## Requirements

### Gazebo
Obviously you will need to install Gazebo to run this. MBARI's packages are irregularly upstreamed into Gazebo releases, meaning the most recent version of Gazebo will include MBARI's work integrated into it. Nonetheless, I reccomend you install the base Gazebo that this package was installed on and then the actual packages on top of that. You can find an installation guide on how to do this [here](https://github.com/osrf/lrauv/wiki/Installation#build-from-source). 

### ROS Parameters bridge

Gazebo communicates through [Gazebo Transport](https://gazebosim.org/api/transport/14/introduction.html), using a pub/sub methodology, which uses Protobuf as its "language" and a combination of custom code and ZeroMQ for serialization/deserialization processes, etc. For now, to communicate with Gazebo, we go through ROS by using the [ROS_GZ](https://github.com/gazebosim/ros_gz/tree/humble) package which, essentially, creates a bridge between Gazebo topics and ROS topics. This allows you to read Gazebo topics as if they were ROS topics and publish to said ROS topics to communicate with Gazebo. 

if you followed the previous installation process, you will probably have Gazebo's fortress version installed and ROS2 humble, which means you should also install the Ros Gazebo version compatible with it. To do so, you should follow these specific [installation](https://github.com/gazebosim/ros_gz/tree/humble) steps. 

### Actuators 

You will also need to provide a csv with the given actuator info. You can generate one using [this](https://github.com/LSTS/pyimclsts/tree/feature/extractAct) and a LAUV log file. 

### Water Density

If you want to be really exact in your simulation you can also provide a water density value. If you wish to generate a density value from the CTD data in the lauv log, you can do so by using [this](https://github.com/LSTS/pyimclsts).

## Usage

Do the standard ROS2 installation procedure.

Install package dependencies with (if you're using a mint distribution add the --os ubuntu:your_distro: 

```
rosdep install -r --from-paths src -i -y --rosdistro humble
```

Build said packages with ``colcon build``. Do the usual sourcing of ROS (you probably have ROS2 humble) and the install:
```
source /opt/ros/humble/setup.bash
source install/setup.bash
```


