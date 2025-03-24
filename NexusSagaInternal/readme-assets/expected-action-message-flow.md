# Expected message flow for Actions

### Navigation action
1. {"data":{"action":"Navigate[donut_on_table]","is_replanned":false,"child_actions":null},"event":"planner","type":"next_action"}
** navigation_highlight message triggers navigation highlight:
2. {"data":{"target":"coffee_table[1]","position":[0.9,-2.48,0.0],"action_id":"Navigate[donut_on_table]"},"event":"skills","type":"navigation_highlight"} 
3. {"data":{"action":"Navigate[donut_on_table]","feedback":"feedback","status":"Complete"},"event":"planner","type":"action_feedback"}


### Pick action:
1. {"data":{"action":"Pick[donut_on_table]","is_replanned":false,"child_actions":null},"event":"planner","type":"next_action"}
** detecting_objects message triggers object highlights:
2. {"data":{"label":"donut[1]","targets":[{"threshold":1.0,"position":[0.849518538,-4.24712133,0.635576665],"normal":[0.0,0.0,1.0]}],"action_id":"Pick[donut_on_table]"},"event":"skills","type":"detecting_objects"}
** found_object message triggers target highlight:
3. {"data":{"label":"donut[1]","targets":[{"threshold":1.0,"position":[0.849518538,-4.24712133,0.635576665],"normal":[0.0,0.0,1.0]}],"action_id":"Pick[donut_on_table]"},"event":"skills","type":"found_object"}
4. {"data":{"state":"Open"},"event":"skills","type":"gripper_state"}
** gripper state 'holding' message in a pick action will track the target highlight with the EE:
5. {"data":{"state":"Holding"},"event":"skills","type":"gripper_state"}
** world graph edit message (not mandatory for place action highlights, but assume it should happen around here)
{"data":{"object":{"receptacle":"robot[1]","operation":"add/receptacle","object_id":"donut[1]","object_name":"donut","category_tag":"donut[1]","room":"kitchen","bbox_extent":[0.127667189,-0.105458736,0.105576709],"bbox_center":[0.849518538,-4.24712133,0.635576665],"yaw":0.0}},"event":"world_graph","type":"edit"}
6. {"data":{"action":"Pick[donut_on_table]","feedback":"feedback","status":"Complete"},"event":"planner","type":"action_feedback"}


### Place action

1. {"data":{"action":"Place[donut_in_bin]","is_replanned":false,"child_actions":null},"event":"planner","type":"next_action"}
** location_detection message triggers place object highlight and semantic place highlight:
2. {"data":{"action_id":"Place[donut_in_bin]","position":[3.86718273,0.5236677,0.20927304],"target_orientation":null},"event":"skills","type":"location_detection"}
** gripper state 'open' will stop the target object highlight tracking with the EE:
3. {"data":{"state":"Open"},"event":"skills","type":"gripper_state"}
4. {"data":{"state":"Closed"},"event":"skills","type":"gripper_state"}
** world graph edit message (not mandatory for place action highlights, but assume it should happen around here)
{"data":{"object":{"receptacle":"trash_can[1]","operation":"add/receptacle","object_id":"donut[1]","object_name":"donut","category_tag":"donut[1]","room":"kitchen","bbox_extent":[0.127667189,-0.105458736,0.105576709],"bbox_center":[3.86718273,0.5236677,0.20927304],"yaw":0.0}},"event":"world_graph","type":"edit"}
5. {"data":{"action":"Place[donut_in_bin]","feedback":"feedback","status":"Complete"},"event":"planner","type":"action_feedback"}

### Replanning
{"data":{"event":null,"type":null,"data":null},"event":"planner","type":"replanning"}

### Open
1. {"data":{"action":"Open[drawer]","is_replanned":false,"child_actions":null},"event":"planner","type":"next_action"}
** found_object message triggers target highlight:
2. {"data":{"label":"drawer","targets":[{"threshold":1.0,"position":[8.057953,-5.83281946,0.64],"normal":[0.0,0.0,1.0]}],"action_id":"Open[drawer]"},"event":"skills","type":"found_object"}
** gripper state 'holding' message in a pick action will track the target highlight with the EE:
3. {"data":{"state":"Holding"},"event":"skills","type":"gripper_state"}
** gripper state 'open' will stop the target object highlight tracking with the EE:
4. {"data":{"state":"Open"},"event":"skills","type":"gripper_state"}
5. {"data":{"action":"Open[drawer]","feedback":"feedback","status":"Complete"},"event":"planner","type":"action_feedback"}


### Close
1. {"data":{"action":"Close[drawer]","is_replanned":false,"child_actions":null},"event":"planner","type":"next_action"}
** found_object message triggers target highlight:
2. {"data":{"label":"drawer","targets":[{"threshold":1.0,"position":[6.22,-5.49,0.78],"normal":[0.0,0.0,1.0]}],"action_id":"Close[drawer]"},"event":"skills","type":"found_object"}
** gripper state 'holding' message in a pick action will track the target highlight with the EE:
3. {"data":{"state":"Holding"},"event":"skills","type":"gripper_state"}
** gripper state 'open' will stop the target object highlight tracking with the EE:
4. {"data":{"state":"Open"},"event":"skills","type":"gripper_state"}

### Instruction Complete will show complete state and re-enable the recording UI
{"data":{"event":null,"type":null,"data":null},"event":"planner","type":"instruction_complete"}

### Instruction Failure will show failed state and re-enable the recording UI
{"data":{"event":null,"type":null,"data":null},"event":"planner","type":"instruction_failed"}

