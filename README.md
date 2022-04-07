# Autoware.Auto Shuttle Bus
A project that implements full software stack for RoboBus based on ROS 2 and Autoware.Auto.

## Use Cases
Supported regions:
* Pennovation Center at the University of Pennsylvania

## Development Requirements
1. Each folder in this repository represents a valid ROS 2 package.
2. All dependencies should be properly resolved in `package.xml` so that the entire system can be installed using `rosdep update`.

## Usage
1. Set up Autoware.Auto code base following [this official documentation](https://autowarefoundation.gitlab.io/autoware.auto/AutowareAuto/installation-no-ade.html).
2. Go into `src/external` and clone this repo:
```bash
cd src/external
git clone https://github.com/mlab-upenn/autoware-auto-shuttlebus.git
```

3. Download the required data by following instructions in [this README](shuttlebus_launch/data/README.md).
4. Build the require packages:
```bash
colcon build --packages-up-to shuttlebus_launch
```

5. Run the entire system:
```bash
ros2 launch shuttlebus_launch localization.launch.py
ros2 launch shuttlebus_launch planning.launch.py
```

## Designs and READMEs
For specific algorithm/software designs, please check out the `design` folder in each ROS package. 
