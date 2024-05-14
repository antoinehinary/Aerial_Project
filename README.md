# cflib: Crazyflie python library [![CI](https://github.com/bitcraze/crazyflie-lib-python/workflows/CI/badge.svg)](https://github.com/bitcraze/crazyflie-lib-python/actions)

cflib is an API written in Python that is used to communicate with the Crazyflie
and Crazyflie 2.0 quadcopters. It is intended to be used by client software to
communicate with and control a Crazyflie quadcopter. For instance the [Crazyflie PC client](https://www.github.com/bitcraze/crazyflie-clients-python)  uses the cflib.

See [below](#platform-notes) for platform specific instruction.
For more info see our [documentation](https://www.bitcraze.io/documentation/repository/crazyflie-lib-python/master/).

## Installation
See the [installation instructions](docs/installation/install.md) in the github docs folder.

## Official Documentation

Check out the [Bitcraze crazyflie-lib-python documentation](https://www.bitcraze.io/documentation/repository/crazyflie-lib-python/master/) on our website.

## Contribute
Go to the [contribute page](https://www.bitcraze.io/contribute/) on our website to learn more.

### Test code for contribution
Run the automated build locally to test your code

	python3 tools/build/build



## CODE EXPLANATION

"with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf" -> for the communication with the Crazyfy
"with MotionCommander(scf) as mc" -> to command the drone's movement

### Examples

- Step-by-step : contains the file "sbs_motion_commander.py" which shows how to take off, move forward, turn functions

- multiranger : contains the file "multiranger_push.py" example scripts that allows a user to "push" the Crazyflie 2.0 around
using your hands while it's hovering.

- positionning : contains the file "bezier_trajectory.py" example of how to generate trajectories for the High Level commander using Bezier curves.

- positionning : contains the file "flowsequenceSync.py" simple example that connects to the crazyflie at `URI` and runs a figure 8 sequence.

- positionning : contains the file "initial_position.py" The initial pose (x, y, z, yaw) is configured in a number of variables and the trajectory is flown relative to this position, using the initial yaw.

- parameters : contains the file "basicparam.py" Simple example that connects to the first Crazyflie found, triggers reading of all the parameters and displays their values. It then modifies one parameter and reads back it's value. Finally it disconnects.

- autonomy : contain the file "autonomous_sequence_high_level.py" Simple example that connects to one crazyflie (check the address at the top and update it to your crazyflie address) and uses the high level commander to send setpoints and trajectory to fly a figure 8.

- autonomy : contain the file "autonomousSequence.py" Simple example that connects to one crazyflie (check the address at the top and update it to your crazyflie address) and send a sequence of setpoints, one every 5 seconds.

- autonomy : contain the file "motion_commander_demo.py" This script shows the basic use of the MotionCommander class. Contains most of the necessary functoins for the drone's motion. The MotionCommander uses velocity setpoints.

- autonomy : contain the file "position_commander_demo.py" This script shows the basic use of the PositionHlCommander class. The PositionHlCommander uses position setpoints.

### Positioning

- motion_commander.py
- position_hl_commander.py

### Crazyfly 

- high_level_commander.py : Used for sending high level setpoints to the Crazyflie
