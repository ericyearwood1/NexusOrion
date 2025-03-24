FROM python:3.8-alpine AS base

ENV PYTHONUNBUFFERED=1

FROM base AS siro-server-base

# Requirements are installed here to ensure they will be cached.
COPY siro_server/requirements.txt /requirements.txt
RUN pip install -r /requirements.txt

ADD siro_server/ /app
WORKDIR /app

FROM siro-server-base AS siro-server
CMD ["python", "main.py", "--port", "7500"]

FROM siro-server-base AS siro-server-without-ros-nodes-reconnection
CMD ["python", "main.py", "--port", "7500", "--disable-ros-nodes-reconnection"]

FROM base AS ros-nodes-service

# Requirements are installed here to ensure they will be cached.
COPY ros_nodes/service/requirements.txt /requirements.txt
RUN pip install -r /requirements.txt
ADD ros_nodes/service/ /app
WORKDIR /app

FROM ros-nodes-service AS planner
CMD ["python", "planner_service_stub.py"]

FROM ros-nodes-service AS human-activity
CMD ["python", "human_activity_service_stub.py"]

FROM ros-nodes-service AS skills
CMD ["python", "skills_service_stub.py"]

FROM ros-nodes-service AS world-graph
CMD ["python", "world_graph_service_stub.py"]