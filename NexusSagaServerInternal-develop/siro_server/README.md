# SIRO_SERVER

Additional information is available in root Readme.

## script arguments

`main.py` has the following arguments

* `--host localhost` default is `0.0.0.0`
* `--port 7000` default is `7500`
* `--enable-ros-nodes-subscriptions` default is disabled, which means
    the Unity client will receives all messages from all ROS nodes
    immediately. Enabling this flag requires the Unity client to send
    a `subscribe` event to register to specific ROS node(s)
* `--disable-service-discovery` default is enabled, which means `siro_server` 
    will try to broadcast its IP address to the local network, 
    only works in native Python, doesn't work in Docker and WSL
* `--service-discovery-port 8000` default is `7400`
* `--disable-ros-nodes-reconnection` default is enabled, only useful in
    development/debugging when we only care about `siro_server` e.g.
    we only want to test local multiplayer state.

If you wish to change the arguments for the Docker, you can adjust
the `base.dockerfile` in the root of the project.

## Running tests

There are two styles of tests in here.
One is "integration" that actually requires you to spin up all the services
(using Docker is easiest) first before running the test.

The other one is fully automated tests.

To run all tests (you need to run all the services first).
```
# create/activate your virtualenvironment

python -m unittest
```

To run specific file, you can run

```
# create/activate your virtualenvironment

python -m unittest test/test_local_multiplayer_state.py 
```

## Configuration
`siro_server/config` contains configuration files for the server.

### Default Instructions
`default_instructions.json` contains the default instructions which will be shown to the user if STT is not enabled / fails.
This is simply a dictionary with a value `instructions` which is a list of strings.

e.g.
```json
{
    "instructions": ["Help me tidy up."]
}
```

### Suppressed Logs
'suppressed_logs.json' defines the logs that should be suppressed.
This is a dictionary with values with the name of the service with a list of event names that should be suppressed.

e.g.
```json
{
    "planner": ["next_action", "skill_feedback"]
}
```

### Multiplayer State
`multiplayer_state.pkl` is a pickle file into which the siro_server saves the multiplayer state.
This contains room information including shared anchors, so that the server can be restarted without having to reconfigure the room.
This file can be safely deleted if required, it will regenerate the next time a room has an anchor added.

### Feature Configuration

  "har" : 0 = always active, 1 = only show with world visualisation, 2 = disabled
  "is_notifications_active": (true/false) global enable or disable notifications (also applies to user join/leave/ plan completed messages) 
  "is_show_place_object_highlight": (true/false) enable or disable place object highlight
  "is_show_semantic_place": (true/false) enable or disable semantic place

## Notes

### Service discovery
This can send the wrong IP Address if you have multiple NICs.
Workarounds:
* Manually return the correct IP Address in service_discovery/server.py get_my_ip()
* Disable other NICs on your machine.

### Multiplayer State
Though it should not happen, it's possible that some bad data can be saved in the multiplayer state. In the event of an 
unknown error, try deleting the `multiplayer_state.pkl` file.


