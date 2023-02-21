# Exercise 3: Computer Vision for Robotics

This repository contains implementation solutions for exercise 3. For information about the project, please read the report at: [insert report here]

## Structure

### Augmented Reality

Most of the source code is in the `packages/augmented_reality/src` directory. Here is a brief description of each file:

- `packages/augmented_reality/src/led_server.py`: Implements a node that starts up an LED server. This service changes the color of the LED lights according to a string message containing the color.

## Execution:

To execute this project, comment/uncomment the nodes you would like to launch in the `packages/mypackage/launch/multiple_nodes.py` launch file. Currently, it is set to start an LED service, as well as a node that implements the Multi-State task from part 2. To run the program, ensure that the variable `$BOT` store your robot's host name, and run the following commands:

```
dts devel build -f -H $BOT
dts devel run -H $BOT
```

## Credit:

This code is built from the 412 exercise 3 template that provides a boilerplate repository for developing ROS-based software in Duckietown (https://github.com/wagonhelm/cmput412_exercise3).

Build on top of by Nadeen Mohamed and Celina Sheng.

Code was also borrowed (and cited in-code) from the following sources.

- https://docs.duckietown.org/daffy/duckietown-robotics-development/out/dt_infrastructure.html
- https://github.com/Coral79/exA-3/blob/44adf94bad728507608086b91fbf5645fc22555f/packages/augmented_reality_basics/include/augmented_reality_basics/augmented_reality_basics.py
