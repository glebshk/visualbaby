# IMAV-videostream
ROS Nodes zur Aufnahme von Bildern (camera_node) (Realsense und USB Cameras) die Erkennung von Arucomarkern (aruco_detect) sowie das Streamen der Bilder zum debuggen (camera_stream).

## Nutzung der Nodes
### Docker
Herunterladen des docker Containers
1. anmelden bei ghcr.io
```
echo TOKEN_HERE | docker login ghcr.io -u GITHUB_USERNAME --password-stdin
```
Als Token den Classic Access Token

2. Herunterladen des docker Container
Vor dem Herunterladen auf ipv4 setzen
```
sudo sysctl -p /etc/sysctl.d/99-disable-ipv6.conf
```
Dann docker herunterladen
```
docker pull ghcr.io/aachen-drone-development-initiative/imav-videostream/imav_video:latest
```

### Clonen des Git Repositorys
```
git clone git@github.com:Aachen-Drone-Development-Initiative/imav-videostream.git
```
In Repo gehen
```
cd imav-videostream
```
### Docker starten
Bash datei start_docker.sh ausführen
```
chmod +x start_docker.sh
./start_docker.sh
```
### Starten der Nodes
Ausführung des Scripts start_ROS.sh
```
./start_ROS.sh
```


## ROS Node create
# Python
```
ros2 pkg create --build-type ament_python my_python_pkg
```