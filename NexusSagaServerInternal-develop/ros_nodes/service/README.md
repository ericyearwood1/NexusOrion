# ROS Nodes
This folder/package is meant as a boilerplate for each ROS node team.

We should copy the whole folder and only implements the respective ROS node 
service per-team, as each team most likely will have different requirements
and Python versions.

There are also examples of stub services. They can be safely deleted by each
ROS node team.

Additional information is also available in root Readme.

## Base Data Provider
- `data_provider.py` is the base class for anything which provides data to be sent on the socket and handles the data received from the socket.

### Abstract Methods
- `tick()` - called by the `service` object. Use if you need to do something periodically
- `register_listeners()` - called by the service on startup to register the listeners for the socket
- `on_hearbeat()` - called by the service when a heartbeat is received'

`tick()` and listeners calls are non-blocking since they are running in completely different process. In order to communicate with
the tick thread put messages into the _queue and consume them in the tick thread.


### Service Data Providers
Each service has a module containing the data provider class for that service. 
The data provider class should inherit from the base data provider class and implement the abstract methods.
Data Provider class defines abstract methods to retrieve data from the service and to handle data received from the socket.
It also defines methods to correctly return the data in the format expected by the service and handle incoming commands.

eg. `planner/data_provider.py` defines `get_next_action()` and `get_skill_feedback()`

The stub data provider inherits from the service data provider and returns example data on those methods.
The production data provider will override those methods to provide real data, however that is done for each service.

### DTOs
Each service has a module containing the DTOs for that service. `dto.py` defines the data transfer objects for the service.
These are used to define the structure of the data being sent and received on the socket.
These structures are mirrored in the Unity client to ensure that the data is correctly parsed on both ends.

eg. `planner/dto.py` defines `NextActionData` and `ActionFeedbackData`

## Service
- `service.py` is the base class for interacting with the socket provided by the ws_server `main.py` script

### Abstract Properties
- `host` - the host of the ws_server
- `port` - the port of the ws_server, ensure that each service has a unique port if running multiple services from the same machine
- `data_provider_class` - the class of the data provider to be used, this should inherit from the base data provider class for the service
- `logger` - the logger to be used


### Setup

#### Speech Service (with Meta's Seamless via transformers)
Create a new virtual environment for running speech to text (with Seamless)
```bash
mamba env create -f speech/environment.yaml
```
#### Other services
For other services you can either create your new conda environment, or use it with speech conda environment from above or install it with your existing conda envionrment (if there are no conda installation errors)
```bash
pip install -r requirements.txt
```
## Running
After Data Provider and Service are implemented (examples are the stub services),
you can run it with native Python.

If you wish to do integration tests for all services, please refer
to the root Readme to run them in a Docker.

You would need to adjust the base.dockerfile to point to the new ROS node
Python files, and most likely create each requirements file for each ROS node
service (but this only applies if you wish to do integration test using Docker).

## Running tests
There are two styles of tests in here.
One is "integration" that actually requires you to spin up all the services
(using Docker is easiest) first before running the test. These tests do not have
asserts, so you need to manually assert the responses.

The other one is fully automated tests.

To run the manual tests
```
# create/activate your virtualenvironment
cd test/integration
python test_human_activity.py
```

To run specific automated test file, you can run

```
# create/activate your virtualenvironment
python -m unittest test/test_base_service.py 
```


