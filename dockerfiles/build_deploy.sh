#! /bin/bash
docker build -t ttb:final -f dockerfiles/Dockerfile.deploy . && \
echo -e "\e[92mRunning docker container now ...\e[0m" && \
docker run --device=/dev/dri \
			-it \
			--privileged \
			-e "DISPLAY=unix:0.0" \
			-v="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
			--network host \
			--rm \
			--name ttb_deploy \
			ttb:final
