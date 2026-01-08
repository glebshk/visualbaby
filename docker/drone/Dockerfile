FROM ghcr.io/aachen-drone-development-initiative/imav-videostream/imav_video:latest

# ------------------------------------------------------------
# System Tools & Dependencies
# ------------------------------------------------------------
RUN apt-get update && apt-get install -y \
    git \
    wget \
    curl \
    cmake \
    build-essential \
    python3-pip \
    tmux \
    tmuxinator \
    lsb-release \
    gnupg2

    # ------------------------------------------------------------
# Install Mavsdk
# ------------------------------------------------------------
RUN pip install mavsdk

# COPY Code
WORKDIR /workspaces/imav-videostream/
COPY /scripts/ /workspaces/imav-videostream/scripts/
COPY /src/ /workspaces/imav-videostream/src/
COPY /tmux/ /workspaces/imav-videostream/tmux/

#Run start script
RUN chmod +x scripts/

RUN bash ./scripts/setup-sim.sh

CMD ["/bin/bash"]