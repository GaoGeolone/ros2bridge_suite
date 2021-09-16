rosbridge_suite
===============

#### Server Implementations of the rosbridge v2 Protocol

rosbridge provides a JSON interface to ROS, allowing any client to send JSON to publish or subscribe to ROS topics, call ROS services, and more. rosbridge supports a variety of transport layers, including WebSockets and TCP. For information on the protocol itself, see the [rosbridge protocol specification](ROSBRIDGE_PROTOCOL.md).

For full documentation, see [the ROS wiki](http://ros.org/wiki/rosbridge_suite).

This project is derived from the origion project from [Robot Web Tools/rosbridge_suite](https://github.com/RobotWebTools/rosbridge_suite) as part of the [Robot Web Tools](http://robotwebtools.org/) effort. To make a difference, this project mainly maintain the TCP part. 

### Packages

 * [rosbridge_suite](rosbridge_suite) is a [ROS meta-package](http://www.ros.org/wiki/catkin/conceptual_overview#Metapackages_and_the_Elimination_of_Stacks) including all the rosbridge packages.

 * [rosbridge_library](rosbridge_library) contains the Python API that receives JSON-formatted strings as input and controls ROS publishers/subscribers/service calls according to the content of the JSON strings.

 * [rosbridge_server](rosbridge_server) contains a WebSocket server, a TCP server and UDP server implementation that exposes the rosbridge_library. 

 * [rosapi](rosapi) provides service calls for getting meta-information related to ROS like topic lists as well as interacting with the Parameter Server.

### Clients

A rosbridge client is a program that communicates with rosbridge using its JSON API. rosbridge clients include:

 * [roslibjs](https://github.com/RobotWebTools/roslibjs) - A JavaScript API, which communicates with rosbridge over WebSockets.
 * [jrosbridge](https://github.com/WPI-RAIL/jrosbridge) - A Java API, which communicates with rosbridge over WebSockets.
 * [roslibpy](https://github.com/gramaziokohler/roslibpy) - A Python API, which communicates with rosbridge over WebSockets.

 * [ROS-Integration](https://github.com/GaoGeolone/ROSIntegration) - An UnrealEngine 5 plugin, which communicates with rosbridge over TCP server.

### License
ros2bridge is inherited from [rosbridge_suit](https://github.com/RobotWebTools/rosbridge_suite) so it will also released with a BSD license. For full terms and conditions, see the [LICENSE](LICENSE) file.

### Authors
See the [AUTHORS](AUTHORS.md) file for a full list of contributors.

### Troubleshooting

See the [TROUBLESHOOTING](TROUBLESHOOTING.md) doc for common problems and solutions.

### Guidance
add this package suite to your ROS2 projects workspace's src document
```bash
colcon build
```
if On Windows
```bash
call install\setup.bat  
```
if on Ubuntu
```bash
source install\setup.sh
```
to launch the isolate package nodes
```bash
ros2 launch rosbridge_server rosbridge_tcp_launch.xml
```



