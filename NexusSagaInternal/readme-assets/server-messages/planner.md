# Planner Messages 

EventNme="planner"


## NextAction

EventType = "next_action"

### DTO

"action": The type of action. Please see supported types below
"is_replanned" : deprecated. Do not use
"display_message": the message that is displayed in the Robot Display panel
"child_actions": used for compound actions. This is a list of NextActionDTOs

### Supported next_action types

We lowercase the action id and see if it starts with a certain action prefix. These are the supported prefixes:
- nav
- pick
- place
- open
- close
- compound
    
Please use the action_id supplied in the next_action message for any skill feedback messages related to that action. 


## Replanning

EventType = "replanning"

The system could not implement the current plan and needs to formulate a new plan


## Action Feedback

EventType = "action_feedback"

This is used to provide a status on the current action, typically to determine if it was completed
The action_id MUST match the "action" field in the NextAction message

### DTO

"action": str — the action id that the feedback relates to
"feedback": str - the feedback message
"status": int - the status of the action 

## Instruction Complete

EventType = "instruction_complete"

The current instruction is complete. Allow a new instruction to be sent

## Planner Busy

EventType = "planner_is_busy"

The planner is currently processing an instruction. 

### DTO

"current_instruction": string — the current instruction being run


## Instruction Not Understood

EventType = "instruction_not_understood"

The instruction received from the Unity application is not a valid instruction

### DTO

"instruction": str — the instruction that was sent


## Instruction Failed

EventType = "instruction_failed"

The instruction could not be completed. Replanning did not succeed.

## Instruction Received

EventType = "instruction_received"

The instruction has been received by the planner. This is used to sync the users' hand UI.

### DTO

"user": str - the id of the user that sent the instruction
"data": str — the encoded audio or text instruction  
"type": str — audio/text. audio if using STT, text if sending a manual instruction



## Status message

EventType = "status"

General status message for planner

### DTO 

"status": str 

Status can be:
- "cancelling": A user has sent a stop message to the service, and the planner is cancelling the current instruction
- "cancelled": the instruction has been successfully cancelled