#!/bin/bash
set -e

# Check if a Docker container named "imav" exists
if docker ps -a --format '{{.Names}}' | grep -wq "imav"; then
    echo "Container 'imav' exists. start into container."
    docker start imav
    docker exec -it imav /bin/bash 

else
    echo "Container 'imav' does not exist."
    echo "Starting new container 'imav'."
    {
        echo "Start new container 'imav' with video device access."
        docker run -it --name imav --device=/dev/video0:/dev/video0 --device=/dev/video1:/dev/video1 --device=/dev/bus/usb:/dev/bus/usb ghcr.io/aachen-drone-development-initiative/imav-videostream/imav_video:latest /bin/bash 
    } || {
        echo "Failed to start Docker container 'imav'. Please check if Docker is running and the image exists."
        echo "Attempting to pull the latest image..."
        docker pull ghcr.io/aachen-drone-development-initiative/imav-videostream/imav_video:latest
        }
    }
fi