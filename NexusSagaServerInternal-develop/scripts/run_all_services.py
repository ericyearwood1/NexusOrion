# IMPORTANT
# only tested in macos and assuming venv is present in root and all reqs installed
# this script is mainly to get the service discovery working, because
# it can't work in Docker (only works in Linux)
import logging
import os
import signal
import subprocess


def main():
    commands = [
        ['../../venv/bin/python', 'human_activity_service_stub.py'],
        ['../../venv/bin/python', 'planner_service_stub.py'],
        ['../../venv/bin/python', 'skills_service_stub.py'],
        ['../../venv/bin/python', 'world_graph_service_stub.py'],
    ]
    os.chdir('ros_nodes/service')
    for command in commands:
        subprocess.Popen(command)

    os.chdir('../../siro_server/')
    subprocess.call([
        '../venv/bin/python', 'main.py'
    ])


if __name__ == '__main__':
    # ensure child processes are killed on exit
    os.setpgrp()  # create new process group, become its leader
    try:
        main()
    except Exception as e:
        logging.exception(e)
    finally:
        os.killpg(0, signal.SIGKILL)  # kill all processes in my group
