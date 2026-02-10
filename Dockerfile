FROM ros:humble

RUN apt update && apt install -y \
    python3-pip \
    nano

WORKDIR /root

