# Net F/T Sensor Driver

This is meta-package that contains ROS2 software for reading the data from F/T sensors
with RDT interface such as: ATI F/T sensors, OnRobot F/T sensors.
* `net_ft_driver`: A ros2_control hardware interface for F/T sensor.
* `net_ft_diagnostic_broadcaster`: A ros2_controller for broadcasting a diagnostic data from the F/T sensor.
* `net_ft_description`: A package containing F/T sensors' description files.

Currently this package supports only following F/T sensors:
* OnRobot HEX series
* ATI AXIA series
* ATI Net F/T series

Software was tested with `ATI AXIA80` and `OnRobot HEX-E V2`. Tests on ATI Net F / T series sensor are needed.

## Installation

Installing dependencies:
```
sudo apt update
sudo apt dist-upgrade
rosdep update
git -C src clone --branch galactic https://github.com/gbartyzel/ros2_net_ft_driver.git
sudo apt install -y libasio-dev libcurlpp-dev
rosdep install --ignore-src --from-paths src -y -r --rosdistro $ROS_DISTRO
```

Build the package:
```
colcon build --symlink-install
ource install/local_setup.sh
```


## Running

Launch the controller:
```
ros2 launch net_ft_driver net_ft_broadcaster.launch.py ip_address:=192.168.1.1 sensor_type:=ati_axia rdt_sampling_rate:=500
```
where:
* `ip_address`: the IP address of the F/T sensor.
* `sensor_type`: the sensor type, select one of `ati`, `ati_axia`, `onrobot`.
* `rdt_sampling_rate`: the sampling rate of the RDT communication, please refer to the sensor manuals for the frequency range.
