# IMAV
IMAV Repo

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


### Pull Docker image
For the drone 
```
docker pull ghcr.io/aachen-drone-development-initiative/imav/drone:latest
```
For developement
```
docker pull ghcr.io/aachen-drone-development-initiative/imav/dev:latest
```

# Simulation Docker

> [!WARNING]
> If you run Fedora Linux or another Distro with SELINUX pre-installed, you may have to disable it for docker to work properly.
## Setup docker container
### Download dev docker
First Download the Docker Image
```
docker pull ghcr.io/aachen-drone-development-initiative/imav/dev:latest
```

## Start Docker Container
Open the Docker image with the Container Extension with the setup file "devcontainer.json".

There are different Cofigurations for ubuntu, windows and the Imav PC


## Start Simulation
### Setup Simulation
> [!NOTE]
> If you start your container the first time it is usefull to run ./scripts/launch_sitl-x500-indoor.sh for building px4 right. 
> If not px4 is building at the first start of the simulation. After running the script Gazebo and the script with Ctrl +C

If you changed something in the Code then run the setup script
```
./scripts/setup-sim.sh
```

### Run Simulation
Run the startup Script of the simulation
```
./scripts/launch-sim-x500-indoor.sh
```
> [!NOTE]
> If the the drone is not start px4 has to be build. If the build is finished restart the simulation with the command then it will work


### Kill Simulation
> [!NOTE]
> Please Close the Gazebo GUI first, because if not gazebo crash and you have to restart the container.

You have to close the Gazebo GUI first then kill the tmux session

```
Ctrl. B + :kill-session
```

### Start QGroundControl
If you want to look onto the telemetry data directly you can start QGroundControl via the following script
```
./scripts/launch-qgc.sh
```
# Drone Operation
## Preperation
### Setup RPi
> [!NOTE]
> Only nessesary if raspian is new !!!!
Run the following Script to install pigpio deamon
```
./scripts/setup-rpi.sh
```

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
### Pull Docker
For the drone 
```
docker pull ghcr.io/aachen-drone-development-initiative/imav/drone:latest
```
## Start of docker
### IMAV Docker Setup

This project provides Docker-based environments for running the IMAV video streaming and drone systems. It includes two configurations:

- **Main container** (`imav`)
- **Test container** (`imav-test`) (use `*/imav-drone:test` docker image)

---


### Starting Containers
> [!NOTE]
> For using the test container replace `imav` with `imav-test`
#### Start Main Container
```
docker compose run -it imav bash
```

###  Accessing a Running Container

#### Open a Bash Shell in Main Container
```
docker compose exec imav bash
```
If the container is not running yet:
```
docker compose up -d imav
docker compose exec imav bash
```

### Restart Containers

#### Restart Main
```
docker compose restart imav
```

### Stopping Containers

#### Stop All Services
```
docker compose down
```
#### Stop a Single Service
```
docker compose stop imav
```


### Remove Containers
```
docker compose rm imav
```


## Start Fly
Start of fly bash file inside of the docker container
```
./scripts/fly_x500.sh
```
