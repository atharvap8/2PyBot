#!/bin/bash
# get_models.sh -- run ONCE on the Radxa, in the same folder as radxa_brain.py
# Downloads the pretrained YOLOv4-tiny model (80 COCO classes, ~24 MB).
set -e
wget -nc https://raw.githubusercontent.com/AlexeyAB/darknet/master/cfg/yolov4-tiny.cfg
wget -nc https://github.com/AlexeyAB/darknet/releases/download/darknet_yolo_v4_pre/yolov4-tiny.weights
wget -nc https://raw.githubusercontent.com/AlexeyAB/darknet/master/data/coco.names
echo "Models ready: $(ls -sh yolov4-tiny.weights | cut -d' ' -f1)"
