# Functionality
This is a Gazebo plugin that simulate an open-circuit battery model. This is a fairly extensible and reusable battery plugin for any kind of Gazebo compatible robots. We developed this plugin primarily for A DARPA sponsored project [BRASS MARS](http://www.cs.cmu.edu/~brassmars/) at CMU. 

This power model simulates the power consumption of a robot. The amount of power consumed by each component of a robot depends on its usage. The battery its current state of the charge after each simulation iteration determined by `dt` in the code. The battery plugin takes the power loads for each components in the robot that consume energy and current voltage value of the battery (which updates according to the open circuit voltage model) as inputs and returns a new voltage value.


# Support
This plugin is tested for ROS kinetic and Gazebo 7.8.1.

# Build
Create the build directory:
```bash
mkdir ~/catkin_ws/src/brass_gazebo_battery/build
cd ~/catkin_ws/src/brass_gazebo_battery/build
```

Make sure you have sourced ROS before compiling the code:
```bash
source /opt/ros/<DISTRO>/setup.bash
```

Compile the code:
```bash
cmake ../
make    
```

Compiling will result in a shared library, `~/catkin_ws/src/brass_gazebo_battery/build/devel/lib/libbattery_discharge.so`, that can be inserted in a Gazebo simulation.

Lastly, add your library path to the `GAZEBO_PLUGIN_PATH`:
```bash
export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:~/catkin_ws/src/brass_gazebo_battery/build/devel/lib
```

# Build by catkin
Build the plugin by going to the base of your work space and running catkin:
```bash
cd ~/catkin_ws
catkin_make
```

# Installing the Plugin
```bash
cd ~/catkin_ws/src/brass_gazebo_battery/build
cmake ../
make
sudo make install
```

# Usage

A xacro file has been created to include this plugin easily. 

In your *.urdf.xacro file include this line at the top

```
  <xacro:include filename="$(find brass_gazebo_battery)/urdf/battery.xacro"/>
```

Add then an instance of the battery 
```
  <xacro:battery_plugin name="battery" /> 
```

# Exposed ROS services and topics

This Gazebo plugin expose several services that can be accessed via ROS:

```
/battery_monitor_client/robot/battery_discharged
/battery_monitor_client/robot/charge_level 
/battery_monitor_client/robot/charge_level_mwh               
/battery_monitor_client/robot/reset_battery
```
- You can check the charge level of the battery by subscribing to `charge_level` and `charge_level_mwh`.
- A `std_msgs/bool => true` will be published in `battery_discharged` when the battery is completely discharged.
- If you wish to restore the battery to its initial state, you can publish `std_msgs/empty` in the `reset_battery` topic.

Also, it is possible to change the behavior of the topic by publishing in the following services:
```
/battery_monitor_client/robot/set_charge
/battery_monitor_client/robot/set_charge_rate
/battery_monitor_client/robot/set_charging
/battery_monitor_client/robot/set_model_coefficients

```
- By calling `set_charging` with `true`, you can recharge the battery.

# Extending ROS Services

First create the service description file `.srv` and put it in the `srv` folder. Then declare it in the `CMakeList.txt` in the
`add_service_files()` section. Also, add the following to the `CMakeList.txt`:
```cmake
generate_messages(
DEPENDENCIES
std_msgs  # Or other packages containing msgs
)
```

For updating the parameters of the battery model we use ROS services,
so here we explain how to add new services to the code if needed:

```bash
cd ~/catkin_ws
catkin_make
```
The header files associated to the service can be found here:

```bash
cd ~/catkin_ws/devel/include/brass_gazebo_battery
```
The add the following header into the code that want to use the services:

```cpp
#include "brass_gazebo_battery/SetLoad.h"
```
And then add the following declaration:

```cpp
public: bool ServiceName(brass_gazebo_battery::SetLoad::Request& req, brass_gazebo_battery::SetLoad::Response& res);
```
The service can then be advertised as follows:

```cpp
this->rosNode->advertiseService(this->model->GetName() + "/api", &Plugin::ServiceName, this);
```

# Notes about conversions
For converting capacity and charge rate (in `Ah`) to power (`mwh`) which is consumed by planner the formula is `(Ah)*(V) = (Wh)`. For example, if you have a `3Ah` battery rated at `5V`, the power is `3Ah * 5V = 15wh` or `15000mwh`.
For converting `Watts` to `watt-hour`, we do `watt * hour`, e.g., `6 watts / 3600 (wh)` per seconds. 

# Acknowledgements
We used/inspired by existing theory of open circuit battery model. This battery discharge/charge plugin uses the Gazebo `Battery` class which is shipped by the default simulator.


Further references: [r1](http://security.livewatch.com/forum-ref/ohms-law-calculator), [r2](http://batteriesbyfisher.com/determining-charge-time), [r3](https://electronics.stackexchange.com/questions/24160/how-to-calculate-the-time-of-charging-and-discharging-of-battery).

# Maintainer

If you need a new feature to be added, please contact [Pooyan Jamshidi](https://pooyanjamshidi.github.io).
