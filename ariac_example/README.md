# Getting Started/Running the Script

## How to open the environment (ariac_test.py):
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

## How to control the ktting robot using the test script
1. Open up a new terminal and resource the setup file (command in step 2)
2. Open up the test script (cd ariac_ws/src/ARIAC/ariac_example/script/ariac_test.py)
3. Modify the first argument of line 112 (self.goto_preset_location('start', 'kitting_robot')) to either 'start', 'home', 'bin8', etc. To see all preset locations, scroll down one function.
4. Back in terminal, you can run the script with
```
rosrun ariac_example ariac_test.py
```
**Keep in mind a lot of this stuff is still kinda buggy.
