# actuators-control

This repository holds the ROS package of the execution system.
The package, for now, only contains a toy node that continously publishes a constant over a topic.

## Installation
Make sure you have *docker* installed. Then to build and run the docker container: </br>
`chmod +x ./run_ros.sh && ./run_ros.sh` </br>

## Execution
Once the container is running, to launch an interactive shell inside it run: </br>
`docker exec -it ros /bin/bash` </br>
You can then launch a toy ROS node by typing in the terminal: `rosrun execution dispatcher.py`.

## Implementing new Nodes
To implement new nodes as python script, touch a file under the **src/execution/scripts** directory.
Then add the node to the CMakeLists.txt under the `catkin_install_python` section.

## Architectural Sketch
In this section we'll describe the ROS node we'll implement and how they comminucate with each other.
Let's start from the "top" (the furthest from the actuators).
### Planner Node
This node will actually be how we interface with the Planning Module.
It will publish on the *Planned States Topic* a certain message with TBD types (either the trajectory we want the car to follow or the next kinematic state we aim for).
### Kinematics Broker Node
This node acts both as a listener and as a talker, subscribing to the *Planned States Topic* data from the Planner, storing it into a fixed length buffer and publishing it in the *Buffered Kinematics Topic*. 
When the buffer is full, new states get stored by replacing the oldest one and by rearranging the others to maintain the correct ordering.
This buffer is critical to smooth the decisions the Planner outputs. Since it has a (user defined) fixed size, for each kinetic state at any given time it holds many different values; when they are combined together the resulting kinetic value will act as a weighted rolling average of the past and actual planned behaviour.

### Controller Node
This node acts both as a listener and as a talker,subscribing to the *Buffered Kinematics Topic* and to the *Data Feed Topic* and publishing the tranformed data in the *Signal Topic*.
This set is transformed into a *n*-dimensional signal, where *n* is the number of available actuators.  The signal traveling in the *i*-th channel is assumed to be compatible with the input interface of the *i*-th actuator.
Each signal will be stored in a list, easing up the future implementation of a feedback loop.
### Dispatcher Node
This node acts both as a listener and as a talker, subscribing to the *Signal Topic* and sending requests through difference *services* (one feed for each actuator).
### Actuator(s) Node(s)
There will be four actuators: Brake, Gear, Steering Wheel and Throttle.
Each of these nodes is a high level representation of the corresponding mechanical/electrical device and itsdriver.  We assume these nodes to be given and not subject to changes. 
Building on what we described earlier, we further conjecture that each Driver node shall offer an asyn-chronously callable service, under a permanent connection for performance reasons, through which they canreceive operating signals from the Dispatcher.  A mechanism of response (either as an acknowledgement ofreception, or as a confirmation of execution) is expected in order to allow the Dispatcher more control overthe actuators.
