docker run -it --rm \
--env="DISPLAY" \
--env="QT_X11_NO_MITSHM=1" \
--env="LIBGL_ALWAYS_SOFTWARE=1" \
--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
--volume="$XAUTHORITY:/dot.Xauthority" \
--mount type=bind,source="$(pwd)"/home,target=/home \
--privileged \
--net="host" \
--name="ros" \
--workdir="/home" \
ros1_local

# Add this line before the "ros1_docker" line to mount another directory
# The directory needs to exist in your host OS
# --mount type=bind,source=/your/dir/path,target=/home/path/in/container \
