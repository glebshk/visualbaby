# Setup Simulation
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