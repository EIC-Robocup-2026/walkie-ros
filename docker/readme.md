
# ROS2 Simulation Docker

This directory contains the necessary files to build a Docker image for a ROS 2 simulation environment. 

## Prerequisites

Before you begin, ensure you have **Docker** and **Docker Compose** installed.

## X11 Forwarding

To allow the Docker container to display graphical applications, you need to grant access to the X server. Run the following command in your terminal before starting the simulation:
Bash
``` bash
xhost local:root
````
## Getting Started

Recommend using **just** as a task runner to simplify common commands.

### Install `just` on Ubuntu

```bash
sudo apt update
sudo apt install just
````

## `just` Commands

Below is a list of the available commands in your `justfile`:

  - `list-topics`: Lists the ROS 2 topics inside the running simulation container.
  - `shell CONTAINER_NAME`: Opens a shell inside the specified container. You must provide the name of the container as an argument (e.g., `just shell robot-simulation`).
  - `run-sim`: Starts the simulation using the `compose.simulation.yml` file.
  - `stop-sim`: Stops the running simulation containers.
  - `build-sim`: Builds the Docker image and tags it as `robocup2026-developmentls:latest`.

## Manual Docker Commands

If you prefer not to use `just`, you can use the following standard Docker commands.

### Build the image

```bash
docker build -t "eic-robocup2026-development:latest" -f Dockerfile.simulation ..
```

### Run the simulation

```bash
docker compose -f compose.simulation.yml up
```

### Stop the simulation

```bash
docker compose -f compose.simulation.yml down
```

### Enter a container's shell

```bash
sudo docker exec -it <CONTAINER_NAME> /bin/bash -c "source /ros_entrypoint.sh && /bin/bash"
```

