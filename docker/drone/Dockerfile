FROM ghcr.io/aachen-drone-development-initiative/imav/base:latest


# COPY Code
WORKDIR /workspaces/IMAV/
COPY /scripts/ /workspaces/IMAV/scripts/
COPY /src/ /workspaces/IMAV/src/
COPY /tmux/ /workspaces/IMAV/tmux/

#Run start script
RUN chmod +x scripts/

RUN bash ./scripts/setup-sim.sh

CMD ["/bin/bash"]