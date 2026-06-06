# Operating state of the R0192 arm, owned by the robot_state_manager node.
# The five states are mutually exclusive (see SetRobotState.srv for transitions).

uint8 DISABLED=0   # Idle, torque-free (start state): motors off, arm_controller inactive
uint8 HOLD=1       # Servos on, holding last setpoint; arm_controller inactive
uint8 JOG=2        # Teach-pendant via MoveIt Servo; arm_controller active, servo running
uint8 MOVEIT=3     # move_group may execute trajectories; arm_controller active, servo paused
uint8 HOMING=4     # Homing sequence running

uint8 state
string status      # human-readable detail (last transition result / error)
