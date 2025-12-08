#!/bin/bash
set -e

usage() {
    echo "Usage: $0 [--test] [--main] [-h] [--help] [--attach]"
    exit 1
}

if [ $# -eq 0 ]; then
    usage
fi
while [[ "$#" -gt 0 ]]; do
    case $1 in
        --main)
            echo "Start new container 'imav' with latest main image and video device access."
            {
                echo "Remove existing container 'imav'"
                docker rm imav
            } || {
                docker stop imav
                docker rm imav
            }
        {
            echo "Run container with video devices."
            docker run -it --name imav --device=/dev/video0:/dev/video0 --device=/dev/video1:/dev/video1 --device=/dev/bus/usb:/dev/bus/usb -p 8555:8555 ghcr.io/aachen-drone-development-initiative/imav-vid>
            } || {
                echo "Run container without video devices."
                docker rm imav
                docker run -it --name imav --device=/dev/bus/usb:/dev/bus/usb -p 8555:8555 ghcr.io/aachen-drone-development-initiative/imav-videostream/imav_video:latest /bin/bash
                }
                 ;;
        --test)
            echo "Start new container 'imav' with test image and video device access."
            {
                echo "Remove existing container 'imav-test'"
                docker rm imav-test
            } || {
                docker stop imav-test
                docker rm imav-test
            }
    {
            echo "Start container 'imav-test' with video access"
            docker run -it --name imav-test --device=/dev/video0:/dev/video0 --device=/dev/video1:/dev/video1 --device=/dev/bus/usb:/dev/bus/usb -p 8555:8555 ghcr.io/aachen-drone-development-initiative/ima>
            } || {
            echo "Start container without video access"
                docker rm imav-test
                docker run -it --name imav-test --device=/dev/bus/usb:/dev/bus/usb -p 8555:8555 ghcr.io/aachen-drone-development-initiative/imav-videostream/imav_video:test /bin/bash
             } ;;
        -h|--help) 
            echo "--main : opens the latest main image container \n
            --test : opens the latest test image container \n
            --restart : restarts the container\n
            -h, --help : displays this help message" ;;
        --attach)
            echo "Attaching to running container 'imav'."
            docker start imav
            docker exec -it imav /bin/bash ;;
        *) usage ;;
    esac
    shift
done

