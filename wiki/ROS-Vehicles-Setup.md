## Table of Contents
- [ROS Setup](#ros-setup)
  - [ROS UAV Models](#ros-uavs)
    - [Hector Quadrotor](#hector-quadrotor)
      - [Hector Quadrotor Sensors](#quadrotor-sensors)
  - [ROS UGV Models](#ros-ugvs)
    - [Turtlebot](#turtlebot)
    - [Husky](#husky)
    - [Grizzly](#grizzly)
    - [Jackal](#jackal)

# <a name="ros-setup"></a> ROS Setup
Currently, the only tested ROS distribution with MACE is Indigo. Follow the instructions on the ROS wiki to install:
[Install ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu) or [Install ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu)

Follow the default install and use default install paths, as MACE currently checks the default locations for library linkings.

## <a name="ros-uavs"></a> ROS UAV Models
### <a name="hector-quadrotor"></a> Hector Quadrotor Installation
Hector Quadrotor is a quadrotor package for ROS with laser-scanning and mapping capabilities. The stack provides packages related to modeling, control and simulation of quadrotor UAV systems.

#### ROS Indigo Setup:
To install on ROS Indigo, run the following:
```
sudo apt-get install ros-indigo-hector-quadrotor ros-indigo-hector-localization ros-indigo-hector-gazebo ros-indigo-hector-models ros-indigo-hector-quadrotor-demo
```

#### ROS Kinetic Setup:
To install on ROS Kinetic, run the following:
```
sudo apt-get install -y linux-headers-generic
sudo sh -c 'echo "deb-src http://us.archive.ubuntu.com/ubuntu/ xenial main restricted
deb-src http://us.archive.ubuntu.com/ubuntu/ xenial-updates main restricted
deb-src http://us.archive.ubuntu.com/ubuntu/ xenial-backports main restricted universe multiverse
deb-src http://security.ubuntu.com/ubuntu xenial-security main restricted" > \
  /etc/apt/sources.list.d/official-source-repositories.list'
sudo apt-get update
sudo apt-get install -y ros-kinetic-librealsense
sudo apt-get install -y ros-kinetic-librealsense-camera
sudo apt-get install -y ros-kinetic-turtlebot
```

**NOTE: If running ROS Kinetic on a Virtual Machine, there are additional steps required.**

If on a VM, older versions of Gazebo will fail when spawning a camera. Updating to the latest version of Gazebo should fix this. To update, run the following command:
```
curl -ssL http://get.gazebosim.org | sh
```

In addition to upgrading Gazebo, the Kinect sensor in the hector quadrotor package will fail to render. To fix this, replace the `xacro` file located in your `hector_sensors_description` directory. Wherever Hector was installed, navigate to `<hector_home>/hector_models/hector_sensors_description/urdf/` and open the `kinect_camera.urdf.xacro`. Replace the contents of that file with the following:

```xml
<?xml version="1.0"?>

<robot xmlns:xacro="http://ros.org/wiki/xacro">
    <xacro:property name="M_PI" value="3.1415926535897931" />

    <xacro:macro name="kinect_camera_model" params="name parent *origin">
      <joint name="${name}_joint" type="fixed">
        <xacro:insert_block name="origin" />
        <parent link="${parent}"/>
        <child link="${name}_link"/>
      </joint>

      <link name="${name}_link">
<!--
        <xacro:inertial_sphere mass="0.01" diameter="0.07" />
        <visual>
          <origin xyz="0 0 0" rpy="0 0 0" />
          <geometry>
            <mesh filename="package://hector_sensors_description/meshes/kinect_camera/kinect_camera_simple.dae"/>
          </geometry>
        </visual>
        <collision>
          <origin xyz="0 0 0" rpy="0 0 0" />
          <geometry>
            <mesh filename="package://hector_sensors_description/meshes/kinect_camera/kinect_camera_simple.stl"/>
          </geometry>
        </collision>
-->
      </link>

      <joint name="${name}_depth_joint" type="fixed">
        <origin xyz="0.0 -0.02 0.0" rpy="0 0 0" />
        <parent link="${name}_link" />
        <child link="${name}_depth_frame"/>
      </joint>

      <link name="${name}_depth_frame"/>

      <joint name="${name}_depth_optical_joint" type="fixed">
        <origin xyz="0 0 0" rpy="${-M_PI/2} 0.0 ${-M_PI/2}" />
        <parent link="${name}_depth_frame" />
        <child link="${name}_depth_optical_frame"/>
      </joint>

      <link name="${name}_depth_optical_frame"/>

      <joint name="${name}_rgb_joint" type="fixed">
        <origin xyz="0.0 -0.0125 0.0" rpy="0 0 0" />
        <parent link="${name}_link" />
        <child link="${name}_rgb_frame"/>
      </joint>

      <link name="${name}_rgb_frame"/>

      <joint name="${name}_rgb_optical_joint" type="fixed">
        <origin xyz="0 0 0" rpy="${-M_PI/2} 0.0 ${-M_PI/2}" />
        <parent link="${name}_rgb_frame" />
        <child link="${name}_rgb_optical_frame"/>
      </joint>

      <link name="${name}_rgb_optical_frame"/>

    </xacro:macro>

    <xacro:macro name="kinect_camera" params="name parent *origin">
      <xacro:kinect_camera_model name="${name}" parent="${parent}">
        <xacro:insert_block name="origin" />
      </xacro:kinect_camera_model>

      <gazebo reference="${name}_depth_frame">
        <sensor type="depth" name="${name}">
          <update_rate>20</update_rate>
          <camera>
            <horizontal_fov>${60 * M_PI/180.0}</horizontal_fov>
            <image>
              <format>R8G8B8</format>
              <width>640</width>
              <height>480</height>
            </image>
            <clip>
              <near>0.05</near>
              <far>3</far>
            </clip>
          </camera>
          <plugin name="${name}_camera_controller" filename="libgazebo_ros_openni_kinect.so">
            <imageTopicName>${name}/rgb/image_raw</imageTopicName>
            <cameraInfoTopicName>${name}/rgb/camera_info</cameraInfoTopicName>
            <depthImageTopicName>${name}/depth/image_raw</depthImageTopicName>
            <depthImageCameraInfoTopicName>${name}/depth/camera_info</depthImageCameraInfoTopicName>
            <pointCloudTopicName>${name}/depth/points</pointCloudTopicName>
            <frameName>${name}_depth_optical_frame</frameName>
          </plugin>
        </sensor>
      </gazebo>
    </xacro:macro>
  </robot>
```

#### <a name="quadrotor-sensors"></a> Hector Quadrotor Sensors
A rundown of the available sensors for Hector Quadrotor is located [HERE](https://github.com/heronsystems/OpenMACE/wiki/Hector-Quadrotor-Sensors)

## <a name="ros-ugvs"></a> ROS UGV Models
### <a name="turtlebot"></a> Turtlebot Installation
A simple to use, common robot platform with ROS integrations is the Turtlebot. It contains an RGB camera and a laser scanner. To install, follow the steps on the ROS wiki:
[Install Turtlebot](http://wiki.ros.org/turtlebot_simulator)

If the installation fails or other issues present themselves, try installing the packages outlined in the Debs installation section [HERE](http://wiki.ros.org/turtlebot/Tutorials/indigo/Turtlebot%20Installation)

### <a name="husky"></a> Husky UGV
Husky is a rugged, outdoor-ready unmanned ground vehicle (UGV), suitable for research and rapid prototyping applications. Husky fully supports ROS. To simulate the Husky UGV, follow the steps [HERE](http://wiki.ros.org/husky_gazebo/Tutorials/Simulating%20Husky)

### <a name="grizzly"></a> Husky UGV
Grizzly Robotic Utility Vehicle (RUV) is designed for rough, rugged, outdoor environments. With four 26" all-terrain tires, a solid steel chassis, and 48V at 400Ah of power, it's ideal for military, mining an agricultural applications. Grizzly RUV offers incredible strength, an unbeatable control system and front axle articulation that keeps the vehicle grounded and stable on even the most challenging terrain. Combined power and precision is the result of onboard current and voltage sensors, high precision wheel encoders, IMU, GPS, and a maximum drawbar of almost 1700lbf.

The Grizzly simulator is implemented using simulator_gazebo stack. It is a three-dimensional, rigid-body model of the Grizzly with most of the hardware-ROS interfaces found on the actual robot. You can install the simulator following the steps [HERE](http://wiki.ros.org/grizzly_simulator)

### <a name="jackal"></a> Jackal UGV
Jackal's fully integrated, weatherproof design offers unique capability in a compact package. To install, follow the steps [HERE](http://wiki.ros.org/jackal_simulator).

# Setup Environment Variables
add the following to the ~/.bashrc and source it

export ROS_MASTER_URI=http://localhost:11311

This will enable running from command line, however, if executing from the QT IDE, you must add it to the environment variables list within the Projects/Build Environment window as at this time it does not appear QT pulls from bash.