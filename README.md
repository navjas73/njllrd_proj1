# njllrd_proj1
MAE 5750 Project 1
Richard Dunphey, Naveen Jasty, Lisa Li

SYMBOLIC SIMULATOR



REAL ROBOT

How to run:
- Clone the project with https://github.com/navjas73/njllrd_proj1.git into the ros_ws/src folder
- Make sure Baxter is on and powered up if using the real robot. Next, in a new terminal, connect to Baxter with the command : ". baxter.sh" or ". baxter.sh sim" if using the simulator from within the ros_ws directory
- From the ros_ws directory, enable the robot with the command: "rosrun baxter_tools enable_robot.py -e"
- Navigate to the ros_ws/src/njllrd_proj1/launch directory
- Run the launch file with the command: "roslaunch proj1.launch"

How to send commands:
- Open a new terminal and connect to Baxter
- From the ros_ws directory, publish messages according to the desired action. Enter: "rostopic pub command std_msgs/String COMMAND_HERE". Acceptable strings for COMMAND_HERE are:
	- "stack_ascending"   - will stack the blocks in ascending order
	- "stack_descending"  - will stack the blocks in descending order
	- "scatter"           - will scatter the blocks 

Navigating the launch file, How to change the initial state or single/dual arm mode:
- From, ros_ws/src/njllrd_proj1/launch, open the file "proj1.launch" in a text editor.
- Change the values of the parameters according to the current setup. 
- The parameters and possible values are:
	- "num_blocks"        - Set this to the initial number of physical blocks in the stack.
	- "configuration"     - Set this to the initial configuration of the blocks: 	'stacked_ascending', 'stacked_descending', or 'scattered'
	- "dual_arm_mode"     - Set this to 'True' to use dual arm mode, or 'False' for right arm only mode
	- "current_mode"      - Do not need to change this
	- "simulator_mode"    - Set this to 'True' to use the Symbolic simulator or 'False' to use the real robot

If using dual arm mode:
- The left arm will manipulate odd blocks, and the right arm will manipulate right blocks. Make sure to initialize the arms as such.
- If the uppermost block is even, set the right arm over this block and the left arm below.
- If the uppermost block is odd, set the left arm over this block and the right arm below. 
- For dual arm mode, initialize the grippers such that they grip the blocks at a downward 45 degree angle from opposite sides. ie, left arm from the left and right arm from the right

Notes:
- Commands can be sent after an action has been completed by publishing a new message using "rostopic pub command std_msgs/String" as before.  The first mode should finish before commanding a new mode. The controller will not allow a command to the existing configuration. If a command is sent while an action is being performed, that new command will be ignored. 
- The initial configuration cannot be scattered. 
- In single arm mode, initialize the right arm vertically above the uppermost block. 
