# sentech_gige_driver

ROS1/ROS2 driver for OMRON Sentech GigE Vision Model M Series.

## Installation

Download and install Sentech SDK: https://sentech.co.jp/en/products/GigE/CMOS-M.html

```bash
cd ros2_src/src
git clone https://github.com/koide3/sentech_gige_driver

cd ros2_src
colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

## Run

```bash
ros2 run sentech_gige_driver sentech_gige_driver
```
