# Autoware.Auto Shuttle Bus
A project that implements full software stack for RoboBus based on ROS 2 and Autoware.Auto.

## Use Cases
Supported regions:
* Pennovation Center at the University of Pennsylvania

## Development Requirements
1. Each folder in this repository represents a valid ROS 2 package.
2. All dependencies should be properly resolved in `package.xml` so that the entire system can be installed using `rosdep update`.

***Developers:* please checkout [this note](https://docs.google.com/document/d/1tw0OC-AYvR9rxGwK4fLNkouOU2YLm5H-BTmGHltnF4w/edit?usp=sharing) for the latest documented known issues.**

## Usage
1. Set up Autoware.Auto code base following [this official documentation](https://autowarefoundation.gitlab.io/autoware.auto/AutowareAuto/installation-no-ade.html).
2. Go into `src/external` and clone this repo:
```bash
cd src/external
git clone https://github.com/mlab-upenn/autoware-auto-shuttlebus.git pennovation-shuttlebus
```

3. Import the missing dependencies:
```bash
cd ${AUTOWARE_AUTO_ROOT}
vcs import < src/external/pennovation-shuttlebus/penn.shuttlebus.repos --recursive 
```

4. Download the required data by following instructions in [this README](shuttlebus_launch/data/README.md).
5. Build the require packages:
```bash
colcon build --packages-up-to shuttlebus_launch
```

6. Run the entire system:
```bash
ros2 launch shuttlebus_launch localization.launch.py
ros2 launch shuttlebus_launch planning.launch.py
```

## Designs and READMEs
For specific algorithm/software designs, please check out the `design` folder in each ROS package. 
