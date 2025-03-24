# Skills Messages 

EventNme="skills"


## EE Pose

EventType = ""ee_pose""

EE Pose data, for tracking the robot gripper

### DTO - EndEffectorPoseMessage

"position": float[] - The position relative to the robot base frame
"orientation": deprecated
"yaw_pitch_roll": YawPitchRoll : The Yaw, Pitch and Roll of the gripper, relative to the robot base frame


## Robot Pose message

EventType = "robot_pose"

Robot Pose data, for tracking the robot 

### DTO - RobotPoseMessage

"position": float[] - the position of the robot relative to the robot home
"yaw": float - the yaw of the robot relative to the robot home

## Navigation Highlight

EventType = "navigation_highlight"

Sent when the robot has determined the next location to navigate to. This triggers a navigation highlight

### DTO - NavigationHighlightMessage
"target" - The name of the navigation target. This is displayed in the highlight label
"position" - the position of the target, relative to the robot home 
"action_id" - the action id that the message relates to


## Detecting Objects

EventType = "detecting_objects"

Send when the robot has received a pick command and is trying to determine which object to pick up

### DTO - TargetObjectDetectionMessage
"label" - The name of the object to be picked up.
"targets" - a list of 1- many targets. The threshold value indicates the likelihood of it being chosen
"action_id"- the id of the action this message relates to


## Found Object

EventType = "found_object"

Send when the robot has received a pick command and has determined which object to pick up

### DTO - TargetObjectDetectionMessage

"label" - The name of the object to be picked up.
"targets" - a list of 1 item, containing the selected object
"action_id"- the id of the action this message relates to


## Gripper State 

EventType = "gripper_state"

The state of the gripper : open | closed | holding

### DTO - GripperStateMessage

"state": GripperState


## Location Detection 

EventType = "location_detection"

Sent when the robot has determined the location to place an object. Will show the semantic place highlight and place location highlight (if enabled in Feature Configuration)

### DTO - PlaceLocationMessage
"action_id"- the id of the action this message relates to
"position" - the position of the target, relative to the robot home 
"target_euler" - the target gripper orientation. used to show the Semantic Place highlight
