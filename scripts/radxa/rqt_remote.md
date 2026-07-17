# rqt / RViz from your Arch PC (not on the Radxa)

rqt and RViz are GUI tools — run them on your desktop, pointed at the
robot's topics. The Radxa stays headless.

## Arch PC, one-time setup

```bash
sudo pacman -S docker
sudo systemctl enable --now docker
xhost +local:docker
```

## RViz

```bash
docker run -it --rm --net=host \
  -e DISPLAY=$DISPLAY -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  osrf/ros:humble-desktop rviz2
```

## rqt (graph, plots, everything)

```bash
docker run -it --rm --net=host \
  -e DISPLAY=$DISPLAY -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  osrf/ros:humble-desktop rqt
```

- Node/topic graph:  Plugins -> Introspection -> Node Graph
- Live plots:        Plugins -> Visualization -> Plot  (add /pybot/pitch/data)
- Topic monitor:     Plugins -> Topics -> Topic Monitor

`--net=host` + same ROS_DOMAIN_ID (default 0) = automatic discovery of
the Radxa's topics on your LAN. Verify with:

```bash
docker run -it --rm --net=host osrf/ros:humble-desktop ros2 topic list
```

If topics don't appear, fall back to Foxglove: run `foxglove.sh` on the
Radxa and connect Foxglove Studio to ws://<radxa-ip>:8765.
