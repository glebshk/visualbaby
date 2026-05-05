# IMAV-videostream
IMAV 2026 Base Repo 

# Preparation
## Docker Images
### Login to ghcr.io
> [!NOTE]
> To get the Token you have to create an personal access token(classic).
> You find it here [Token](https://github.com/settings/tokens)
> You have to give minimum write/read:packages right
```
echo TOKEN_HERE | docker login ghcr.io -u YOUR_GITHUB_USERNAME --password-stdin
```

### IPv4 Setup for RPi
Setup IPv4 for better download of the image
```
sudo sysctl -p /etc/sysctl.d/99-disable-ipv6.conf
```
### Pull Docker image
For the drone 
```
docker pull ghcr.io/aachen-drone-development-initiative/imav-videostream/imav-drone:latest
```
For developement
```
docker pull ghcr.io/aachen-drone-development-initiative/imav-videostream/imav-dev:latest
```

# Drone Operation
## Start of docker
Start docker via bash file
> [!NOTE]
> -- test for the ghcr.io/aachen-drone-development-initiative/imav-videostream/imav-drone:test images
> -- main for the ghcr.io/aachen-drone-development-initiative/imav-videostream/imav-drone:latest images
```
chmod +x start_docker.sh
./start_docker.sh --main
```
## Start Fly
Start of fly bash file
```
./scripts/fly_x500.sh
```

# Simulation Docker
## Setup docker container
### Download dev docker
First Download the Docker Image
```
docker pull ghcr.io/aachen-drone-development-initiative/imav-videostream/imav-dev:latest
```

## Start Docker Container
Open the Docker image with the Container Extension with the setup file "devcontainer.json".

There are different Cofigurations for ubuntu24.04, windows and the Imav PC


## Start Simulation
### Setup Simulation
If you changed something in the Code then run the setup script
```
./scripts/setup-sim.sh
```

### Run Simulation
Run the startup Script of the simulation
```
./scripts/lauch-sim-x500-indoor.sh
```

### Kill Simulation
> [!NOTE]
> Please Close the Gazebo GUI first, because if not gazebo crash and you have to restart the container.

You have to close the Gazebo GUI first then kill the tmux session
```
ros2 pkg create --build-type ament_python my_python_pkg
```

# Simulation Docker
## Setup docker container
### Download dev docker
First Download the Docker Image
```
docker pull ghcr.io/aachen-drone-development-initiative/imav-videostream/imav-dev:latest
```
### Enable X11 Access for Docker
Enable the X11 Access that the Docker container can open Windows
```
xhost +local:root
```

## Start Docker Container
Open the Docker image with the Container Extension with the setup file "devcontainer.json"
***
At the first start of the Container you have to make the scripts executable
```
chmod +x scripts/
```

## Start Simulation
### Setup Simulation
If you changed something in the Code then run the setup script
```
./scripts/setup-sim.sh
```

### Run Simulation
Run the startup Script of the simulation
```
./scripts/lauch-sim-x500-indoor.sh
```

### Kill Simulation
To stop the Simulation use 
```
Strg. B + :kill-session
```