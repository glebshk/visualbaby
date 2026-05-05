FROM ghcr.io/aachen-drone-development-initiative/imav-videostream/imav:latest


# COPY Code
WORKDIR /workspaces/imav-videostream/
COPY /scripts/ /workspaces/imav-videostream/scripts/
COPY /src/ /workspaces/imav-videostream/src/
COPY /tmux/ /workspaces/imav-videostream/tmux/

#Run start script
RUN chmod +x scripts/

RUN bash ./scripts/setup-sim.sh

CMD ["/bin/bash"]