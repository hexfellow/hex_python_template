# hex_python_template

**hex_python_template** is a ROS package template designed to seamlessly integrate ROS 1 and ROS 2 Python code.

## Maintainer

[Dong Zhaorui](mailto:847235539@qq.com)

## Public APIs

### Published Topics

| Topic      | Msg Type                | Description           |
| ---------- | ----------------------- | --------------------- |
| `/out_str` | `std_msgs/(msg/)String` | Example of a string publication. |
| `/out_int` | `std_msgs/(msg/)Int32`  | Example of an int32 publication.  |

### Subscribed Topics

| Topic     | Msg Type                | Description           |
| --------- | ----------------------- | --------------------- |
| `/in_str` | `std_msgs/(msg/)String` | Example of a string subscription. |
| `/in_int` | `std_msgs/(msg/)Int32`  | Example of an int32 subscription.  |

### Parameters

| Name        | Data Type     | Description                  |
| ----------- | ------------- | ---------------------------- |
| `str_name`  | `string`      | Prefix for the output string. |
| `int_range` | `vector<int>` | Range for the output int.     |

## Getting Started

Follow these steps to set up the project for development and testing on your local machine:

1. Create a workspace `catkin_ws` and navigate to the `src` directory:

   ```shell
   mkdir -p catkin_ws/src
   cd catkin_ws/src
   ```

2. Clone the repository:

   ```shell
   git clone https://github.com/IBNBlank/hex_python_template.git
   ```

3. Navigate back to the `catkin_ws` directory and build the workspace:

   For ROS 1:
   ```shell
   cd ../
   catkin_make
   ```

   For ROS 2:
   ```shell
   cd ../
   colcon build
   ```

4. Source the `setup.bash` file and run the tests:

   For ROS 1:
   ```shell
   source devel/setup.bash --extend
   ```

   For ROS 2:
   ```shell
   source install/setup.bash --extend
   ```

### Verified Platforms

* [ ] **Jetson Orin Nano**
* [x] **Jetson Orin NX**
* [ ] **Jetson AGX Orin**
* [ ] **Horizon RDK X5**
* [ ] **Rockchip RK3588**

### Prerequisites

Ensure the following software and hardware are installed:

* **ROS**:  
   Refer to the [ROS Installation guide](http://wiki.ros.org/ROS/Installation)

### Usage

1. Launch the `python_template` node:

   For ROS 1:
   ```shell
   roslaunch hex_python_template python_template.launch
   ```

   For ROS 2:
   ```shell
   ros2 launch hex_python_template python_template.launch.py
   ```

2. Publish RGB and depth images to the `/in_str` and `/in_int` topics, respectively.
3. View the output on the `/out_str` and `/out_int` topics.