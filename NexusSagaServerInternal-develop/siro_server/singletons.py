import multiprocessing

# TODO why a dict and make each key a separate variable?
STATE = {
    "CONNECTED_CLIENTS": set(),
    "ROS_NODES_STATUS": {
        "PLANNER": "unavailable",
        "HUMAN_ACTIVITY": "unavailable",
        "WORLD_GRAPH": "unavailable",
        "SKILLS": "unavailable",
        "SPEECH": "unavailable",
    },
}
MESSAGES_TO_BROADCAST_QUEUE = multiprocessing.Queue()
CURRENT_INSTRUCTION = None

# only for enable_ros_nodes_subscriptions flag
# values are sid
ROS_NODES_SUBSCRIPTIONS = {
    'planner': set(),
    'human_activity': set(),
    'world_graph': set(),
    'skills': set(),
    'speech': set(),
}
