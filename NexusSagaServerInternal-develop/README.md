# SIRo #

## Services

This repo consists of 3 main Services:

* `ros_nodes`
* `siro_server`
* `speech`

They are all running on different ports. `ros_nodes` package is meant to be
extended/implemented, which will have numerous services running on different ports.

## WebSockets

`ros_nodes` and `siro_server` are running an older version of socket.io for
compatibility reasons. 

You must use Socket.io client version 2 to be able
to connect and interact with those services.

## ros_nodes

`ros_nodes` package is more of like a template/boilerplate. Each ROS team
should copy almost all of the files inside `ros_nodes` and implement their
own service.

For integration testing and examples, we provided implemented
stub services in the root of `ros_nodes/service`.

Each stub services are
running on different ports. Stub service files can be safely deleted by
the ROS team.

## How do I get set up?

Each packages have their own requirements (with `ros_nodes` also having different 
requirements for each of its services), so it's recommended to set up 
virtual environment (using your preferred method) for each packages.

Each package should have their base `requirements.txt` filled in. We may
adjust each one according to the needs (especially for each ROS node).

### ros_nodes

`ros_nodes` package is meant to be implemented first before you can run it,
however we provided examples and stub services to be run.

The stub services exists in the root `ros_nodes/service`, they are all share
the same requirements for now, but definitely will be different for each ROS 
node when implemented.

1. create and activate your virtual environment for specific ROS node
2. `pip install -r requirements.txt`
3. `python human_activity_service_stub.py`

And do those for each of the ROS node service.

### siro_server

1. create and activate your virtual environment
2. `pip install -r  requirements.txt`
3. `python main.py`

This will run `siro_server` on port 7500. More details for the script args
can be found in the `parse_args` function inside `main.py`.

The easiest way to run the whole services is by using Docker (explained in later
section).

### speech

**WIP**

### Running with Docker

1. install and set up [Docker](https://docs.docker.com/engine/install/), which should includes [Docker compose](https://docs.docker.com/compose/install/)
    ,it's heavily recommended to use latest Docker version
2. then run `docker-compose up --build` from the root folder, and run this everytime
    there are code changes, alternatively, the Docker `watch` is still WIP

If you wish to point to the implemented ROS node service, you just need
to adjust the Python files in `base.dockerfile`.

There's also another compose file `siro-only-compose.yaml` which the name
suggests it will only run the `siro_server`, it's useful if we only
want to test Unity client code.

### Running with native Python

Running with native Python means you need to open multiple terminals and run
all the `ros_nodes`services and `siro_server` e.g.

```
# in siro_server dir
python main.py

# in ros_nodes/service dir
python human_activity_service_stub.py
python planner_service_stub.py
... other services
```

That can be a bit tedious, alternatively we prepared a script but this assumes
you are using `venv` that is set up in the root and the script has only been
tested in MacOS.

```
venv/bin/python ./scripts/run_all_services.py
```

### Automated integration tests

Automated integration tests are available but is very barebones, but will be
updated.

You can run it from the `siro_server` package, but you need to have all 
the services running first (using Docker is the easiest). Then run:

```
python -m unittest test/test_integration.py
```

### Manual tests, connect and send WS message

`ros_nodes` and `siro_server` are running an older version of socket.io for
compatibility reasons. You must use Socket.io client version 2 to be able
to connect and interact with those services.

There are some automated tests located in `siro_server/test/` you can
take a look there for request/response examples.

Or, you can use Postman to do manual tests/take a look at docs folder for the images.

![postman.png](docs/images/postman.png)

**Important** since we must use Socket.io version 2, in Postman, you need to
configure it here

![postman_v2.png](docs/images/postman_v2.png)

**Aso important**, Postman only registers the event listeneres after you
managed to connect. So you will be missing e.g. messages that are emitted
at on_connect!

### World Graph Messages

Please only support on or inside relationships. `Under` and `next to` relationships are not handled in the Unity application.
Please make sure that the object_name is correct for the type of object. We display the top plane only for the following object_names: table; console; counter; island; cabinet
Otherwise we use the full extents value for the object scale. As specified by Meta, the extents equate to the full size of the item.

Supported object_names:

- party_hat
- donut
- cup
- can
- bottle
- dining_table
- kitchen_island
- kitchen_counter
- sofa
- wooden_console
- wooden_dresser
- coffee_table
- office_chair
- cabinet
- drawer
- trash_can
- shelf
- chair
- hamper
- pillow
- refrigerator
- bed
- toy
- ball

"add/item"; // item added to world graph
"remove/item"; // item removed from world graph
"add/receptacle"; // item is contained in receptacle
"remove/receptacle"; // item is no longer contained in receptacle
"add/update-room"; // item has been taken to another room

### Expected Messages for Highlights

Please see docs/expected-action-message-flow.md

### Videos demo

#### Integration test using Docker

Prerequisite: please read Running with Docker section on to install Docker first

[docker.mp4](docs/videos/docker.mp4)

### Integration test using native Python

[native_python.mp4](docs/videos/native_python.mp4)

Alternatively, you can use `run_all_services.py` (explained in previous sections).

## Tests

### ros_nodes

Please refer to the Readme in there.

### siro_server

Please refer to the Readme in there.
