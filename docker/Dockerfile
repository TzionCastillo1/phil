FROM ubuntu:22.04

ENV DEBIAN_FRONTEND noninteractive

RUN apt-get -qq update && \
        apt-get -qq upgrade && \
        apt-get install -y apt-utils build-essential && \
        apt-get install -y python3-pip swig

RUN apt-get install -y python3-dev

#install Gazebo Harmonic
RUN apt-get update && \
        apt-get install -y lsb-release wget gnupg && \
        wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
        echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null && \
        apt-get update && \
        apt-get install -y gz-harmonic

