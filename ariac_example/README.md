# Getting Started/Running the Script

## How to open the environment:
1. Open up the launch file (cd ariac_ws/src/ARIAC/nist_gear/launch/sample_environment.launch) and make sure that lines 34-43 (the two group tags near the end of the file) are NOT commented out. These are needed to enable MoveIt.
2. Open up a terminal and run
```
source ~/ariac_ws/devel/setup.bash
```
3. To launch Gazebo and the whole environment, run
```
roslaunch nist_gear sample_environment.launch load_moveit:=true
```

## How to control the kitting robot using the GUI
1. Once you can see the environment in Gazebo, open up a new tab in terminal and run the command in step 2 again (sourcing the setup file).
2. To open up the GUI controller, run
```
rosrun rqt_gui rqt_gui robot_description:=/ariac/kitting/robot_description
```
3. Once the GUI pops up, open a joint controller plugin with Plugins > Robot Tools > Joint trajectory controller (this might already be open, in that case you can skip this step)
4. Select /ariac/kitting/controller_manager and kitting_arm/controller
5. A bunch of sliders should pop up, you can use this to control the joints manually.

## How to run a full simulation
1. Open up a new terminal and resource the setup file (command in step 2)
2. Open up the test script (cd ariac_ws/src/ARIAC/ariac_example/script/ariac_test.py)
3. Back in terminal, you can run the script with
```
rosrun ariac_example auto_control.py default_world.json
```
## How to control the ktting robot using the test script
1. Open up a new terminal and resource the setup file (command in step 2)
2. Open up the test script (cd ariac_ws/src/ARIAC/ariac_example/script/ariac_test.py)
3. Modify the first argument of the function 'goto_preset_location' to either 'start', 'home', 'bin8', etc. To see all preset locations, scroll down one function.
4. Back in terminal, you can run the script with
```
rosrun ariac_example ariac_test.py
```
## How to have the kitting robot pickup batteries from the conveyor belt and place it on an agv
1. Start the competition to get the batteries to appear on the conveyor belt (might take a minute). Make sure you have the setup file sourced. Run 
```
rosservice call /ariac/start_competition
```
2. After the green batteries begin to show up on the belt, manually stop the belt with the following command:
```
rosservice call /ariac/conveyor/control "power: 0"
```
At this point, we haven't written functionality for the robot to pick batteries off of a moving conveyor belt. That functionality may be added in the next few weeks or so.
3. Click on the green battery you would like to pick up, and in Gazebo check its pose. Take the y-value and subtract 0.16. Set this as the first value in kitting_arm array under 'conveyor' in the set_preset_location function.
4. If you would like to modify which agv and which station it will be delivered to, modify these arguments in the main function call in the last goto_preset_location function call and the move_agvs function call.

## Also be sure to have the rqt_gui shut off before attempting to run any of this script.

