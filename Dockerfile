FROM --platform=linux/amd64 cartographer:latest AS google-potato

RUN apt-get update && apt-get install -y openjdk-17-jdk mc

WORKDIR /mnt