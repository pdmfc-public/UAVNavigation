<img src="bebop.png"  width="450px" align="right" />

# Bebop
> Dev branch from  [bebop_autonomy](https://bebop-autonomy.readthedocs.io/en/latest/)

This repo contains a bebop_autonomy workspace which  is a ROS driver for Parrot Bebop 1.0 and 2.0 drones (quadrocopters), based on Parrot’s official ARDroneSDK3. 

This driver has been developed in Autonomy Lab of Simon Fraser University by Mani Monajjemi and other contributers (List of Contributers). 



# Pre-requirements

- ROS Melodic (Only tested on Ubuntu)
- Ubuntu packages: `build-esstential`,` python-rosdep`, `python-catkin-tools`
- Basic familiarity with building ROS packages


```
sudo apt-get install build-essential python-rosdep python-catkin-tools
```

To compile from source, you need to clone the source code in a new or existing catkin workspace, use rosdep to install dependencies and finally compile the workspace using catkin. 
It's highy recommended to use `catking build` rather then `catkin_make`.
The following commands demonstrate this procedure in a newly created catkin workspace.

> ##### Note
> 
> Since version 0.6, [Parrot ARSDK](https://developer.parrot.com/docs/SDK3/), the main underlying dependency of bebop_autonomy is not build inline anymore.
>
> Instead, the slightly patched and catkinized version of it, called [parrot_arsdk](https://github.com/AutonomyLab/parrot_arsdk)), is fetched as a dependency during the `rosdep install` step below.
>
> This dramatically decreases the compile time of the package compared to previous versions (e.g. from ~15 minutes to less than a minute on a modern computer). If you are re-compiling from source, you need to clean your workspace first:  `catkin clean [-y]`.
>

```md
# Create and initialize the workspace
mkdir -p ~/bebop_ws/src && cd ~/bebop_ws
catkin init
git clone https://github.com/AutonomyLab/bebop_autonomy.git src/bebop_autonomy
# Update rosdep database and install dependencies (including parrot_arsdk)
rosdep update
rosdep install --from-paths src -i
# Build the workspace
catkin build
```


# Running

The executable node is called `bebop_driver_node` and exists in `bebop_driver` package. 

It’s recommended to run the Node in its own namespace and with default configuration. 

The driver package comes with a sample launch file `bebop_driver/launch/bebop_node.launch` which demonstrates the procedure.

```
roslaunch bebop_driver bebop_node.launch
```

##### bebop_node.launch

```xml
<launch>
    <arg name="namespace" default="bebop" />
    <arg name="ip" default="192.168.42.1" />
    <arg name="drone_type" default="bebop2" /> <!-- available drone types: bebop1, bebop2 -->
    <arg name="config_file" default="$(find bebop_driver)/config/defaults.yaml" />
    <arg name="camera_info_url" default="package://bebop_driver/data/$(arg drone_type)_camera_calib.yaml" />
    <group ns="$(arg namespace)">
        <node pkg="bebop_driver" name="bebop_driver" type="bebop_driver_node" output="screen">
            <param name="camera_info_url" value="$(arg camera_info_url)" />
            <param name="bebop_ip" value="$(arg ip)" />
            <rosparam command="load" file="$(arg config_file)" />
        </node>
        <include file="$(find bebop_description)/launch/description.launch" />
    </group>
</launch>
```

# Sending Commands to Bebop

`bebop_tools` package comes with a launch file for tele-operating Bebop with a joystick using ROS joy_teleop package. 

The configuration file (key-action map) is written for Logitech F710 controller and is located in `bebop_tools/config` folder. 

Adapting the file to your own controller is straightforward. To teleop Bebop while the driver is running execute `roslaunch bebop_tools joy_teleop.launch`.

##### Takeoff

Publish a message of type `std_msgs/Empty` to `takeoff` topic.

```
rostopic pub --once [namespace]/takeoff std_msgs/Empty
```

##### Land

Publish a message of type `std_msgs/Empty` to `land` topic.
```
rostopic pub --once [namespace]/land std_msgs/Empty
```

##### Emergency

Publish a message of type `std_msgs/Empty` to `reset` topic.

```
rostopic pub --once [namespace]/reset std_msgs/Empty
```

##### Piloting

To move Bebop around, publish messages of type [geometry_msgs/Twist to cmd_vel](http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html) topic while Bebop is flying. 

The effect of each field of the message on Bebop’s movement is listed below:

```
linear.x  (+)      Translate forward
          (-)      Translate backward
linear.y  (+)      Translate to left
          (-)      Translate to right
linear.z  (+)      Ascend
          (-)      Descend
angular.z (+)      Rotate counter clockwise
          (-)      Rotate clockwise
```


Acceptable range for all fields are `[-1..1]`. The drone executes the last received command as long as the driver is running. This command is reset to zero when Takeoff, Land or Emergency command is received. To make Bebop hover and maintain its current position, you need to publish a message with all fields set to zero to `cmd_vel`.

The `linear.x` and `linear.y` parts of this message set the pitch and roll angles of the Bebop, respectively, hence control its forward and lateral accelerations. The resulting pitch/roll angles depend on the value of `~PilotingSettingsMaxTiltCurrent` parameter, which is specified in degrees and is dynamically reconfigurable (Dynamically Reconfigurable Parameters for Bebop).

The `linear.z` part of this message controls the vertical velocity of the Bebop. The resulting velocity in m/s depends on the value of `~SpeedSettingsMaxVerticalSpeedCurrent` parameter, which is specified in meters per second and is also dynamically reconfigurable (Dynamically Reconfigurable Parameters for Bebop). Similarly, `the angular.z` component of this message controls the rotational velocity of the Bebop (around the z-axis). The corresponding scaling parameter is `SpeedSettingsMaxRotationSpeedCurrent` (in degrees per sec).


```
roll_degree       = linear.y  * max_tilt_angle
pitch_degree      = linear.x  * max_tilt_angle
ver_vel_m_per_s   = linear.z  * max_vert_speed
rot_vel_deg_per_s = angular.z * max_rot_speed
```

##### Moving the Virtual Camera


To move Bebop’s virtual camera, publish a message of type [geometry_msgs/Twist to cmd_vel](http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html) to camera_control topic. `angular.y` and `angular.z` fields of this message set absolute tilt and pan of the camera in degrees respectively. The field of view of this virtual camera (based on our measurements) is ~80 (horizontal) and ~50 (vertical) degrees.

```
angular.y (+)      tilt down
          (-)      tilt up
angular.z (+)      pan left
          (-)      pan right
```

#### GPS Navigation


##### Start Flight Plan

An autonomous flight plan consists of a series of GPS waypoints along with Bebop velocities and camera angles encoded in an XML file.

Requirements that must be met before an autonomous flight can start:

- Bebop is calibrated
- Bebop is in outdoor mode
- Bebop has fixed its GPS

To start an autonomous flight plan, publish a message of type [std_msgs/String](http://docs.ros.org/api/std_msgs/html/msg/String.html) to autoflight/start topic. The `data` field should contain the name of the flight plan to execute, which is already stored on-board Bebop.

> ##### Note
> 
> If an empty string is published, then the default ‘flightplan.mavlink’ is used.
>
> If not already flying, Bebop will attempt to take off upon starting a flight plan.

An FTP client can also be used to view and copy flight plans on-board Bebop. FileZilla is recommended:


```
sudo apt-get install filezilla
filezilla
```

Then open Site Manager (top left), click New Site:

- Host: 192.168.42.1
- Protocol: FTP
- Encrpytion: Use plain FTP
- Logon Type: Anonymous
- Connect.
 

##### Pause Flight Plan

To pause the execution of an autonomous flight plan, publish a message of type [std_msgs/Empty](http://docs.ros.org/api/std_msgs/html/msg/Empty.html) to autoflight/pause topic. Bebop will then hover and await further commands. To resume a paused flight plan, publish the same message that was used to start the autonomous flight (ie. to the topic autoflight/start). Bebop will fly to the lastest waypoint reached before continuing the flight plan.

> ##### Note
> 
> Any velocity commands sent to Bebop during an autonomous flight plan will pause the plan.

##### Stop Flight Plan

To stop the execution of an autonomous flight plan, publish a message of type [std_msgs/Empty](http://docs.ros.org/api/std_msgs/html/msg/Empty.html) to autoflight/stop topic. Bebop will hover and await further commands.


##### Navigate Home


To ask Bebop to autonomously fly to it’s home position, publish a message of type [std_msgs/Bool](http://docs.ros.org/api/std_msgs/html/msg/Bool.html) to autoflight/navigate_home topic with the `data` field set to `true`. To stop Bebop from navigating home, publish another message with `data` set to `false`.














