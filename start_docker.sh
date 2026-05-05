#!/bin/bash
set -e

usage() {
    echo "Usage: $0 [--test] [--main] [-h] [--help] [--attach {main}] [--restart {main}]"
    exit 1
}
TAGET = ""
if [ $# -eq 0 ]; then
    usage
fi
while [[ "$#" -gt 0 ]]; do
    case $1 in
        --main)
            echo "Start new container 'imav' with latest main image and video device access."
            if docker ps -a --format '{{.Names}}' | grep -q "^imav$"; then
                echo "Container imav available"
                if docker ps --format '{{.Names}}' | grep -q "^imav$"; then
                    echo "Stop Container imav"
                    docker stop imav
                    docker rm imav
                else
                    docker rm imav
                fi
            else
                :
            fi
            echo "Start of main imav Container"
            {
                    echo "Run container with video devices."
                    docker run -it --name imav --device=/dev/video0:/dev/video0 --device=/dev/video1:/dev/video1 --device=/dev/bus/usb:/dev/bus/usb -p 8555:8555 ghcr.io/aachen-drone-development-initiative/imav-videostream/imav_video:latest /bin/bash
            } || {
                echo "Run container without video devices."
                docker rm imav
                docker run -it --name imav --device=/dev/bus/usb:/dev/bus/usb --device=/dev/ttyAMA0:/dev/ttyAMA0  -p 8555:8555 ghcr.io/aachen-drone-development-initiative/imav-videostream/imav-drone:latest /bin/bash
                }
                 ;;
        --test)
            echo "Start new container 'imav' with test image and video device access."
            if docker ps -a --format '{{.Names}}' | grep -q "^imav-test$"; then
                echo "Container imav available"
                if docker ps --format '{{.Names}}' | grep -q "^imav-test$"; then
                    echo "Stop Container imav-test"
                    docker stop imav-test
                    docker rm imav-test
                else
                    docker rm imav-test
                fi
            else
                :
            fi
    {
            echo "Start container 'imav-test' with video access"
            docker run -it --name imav-test --device=/dev/video0:/dev/video0 --device=/dev/video1:/dev/video1 --device=/dev/bus/usb:/dev/bus/usb  --device=/dev/ttyAMA0:/dev/ttyAMA0 -p 8555:8555 ghcr.io/aachen-drone-development-initiative/imav-videostream/imav-drone:test /bin/bash
            } || {
            echo "Start container without video access"
                docker rm imav-test
                docker run -it --name imav-test --device=/dev/bus/usb:/dev/bus/usb --device=/dev/ttyAMA0:/dev/ttyAMA0 -p 8555:8555 ghcr.io/aachen-drone-development-initiative/imav-videostream/imav-drone:test /bin/bash
             } ;;

        -h|--help) 
            echo "--main : opens the latest main image container \n
            --test : opens the latest test image container \n
            --restart {main}: restarts the container\n
            --attach {main}: attach to main container
            -h, --help : displays this help message" ;;
        --attach)
            echo "Attaching to running container."
            TAGET = "$2"
            if TAGET == "main"; then
                docker start imav
                docker exec -it imav /bin/bash 
            elif TAGET == "test";then
                docker start imav-test
                docker exec -it imav-test /bin/bash
            else
                echo "No valid taget"
                exit 1
            ;;
        
        --restart)
            echo "Restart container"
            TAGET = "$2"
            if TAGET == "main"; then
                docker restart -t 10 imav
                docker exec -it imav /bin/bash 
            elif TAGET == "test";then
                docker restart -t 10 imav-test
                docker exec -it imav-test /bin/bash
            else
                echo "No valid taget"
                exit 1
            ;;
        *) usage ;;
    esac
    shift
done

