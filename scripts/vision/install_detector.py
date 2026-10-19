"""Install the fixed, checksum-verified ONNX Model Zoo detector outside Git."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import urllib.request

URL = "https://media.githubusercontent.com/media/onnx/models/main/validated/vision/object_detection_segmentation/ssd-mobilenetv1/model/ssd_mobilenet_v1_12-int8.onnx"
SHA256 = "2b79e6a7fb1ec6a33f332b9b10d82d9de4b7b49dcd26b5946921bb356895c954"


def install(destination):
    destination = Path(destination)
    destination.parent.mkdir(parents=True, exist_ok=True)
    if destination.is_file() and hashlib.sha256(destination.read_bytes()).hexdigest() == SHA256:
        print("Verified detector already installed:", destination)
        return
    temporary = destination.with_suffix(".download")
    digest = hashlib.sha256()
    try:
        with urllib.request.urlopen(URL, timeout=60) as source, temporary.open("wb") as output:
            while True:
                chunk = source.read(1024*1024)
                if not chunk:break
                output.write(chunk);digest.update(chunk)
        if digest.hexdigest() != SHA256:
            raise RuntimeError("Detector checksum mismatch; downloaded model was not installed")
        os.replace(temporary, destination)
    finally:
        if temporary.exists():temporary.unlink()
    metadata = {"name": "SSD-MobileNetV1 INT8", "source": URL, "sha256": SHA256,
                "license": "MIT", "classes": "COCO category IDs", "input": "RGB NHWC uint8",
                "role": "2D perception only; no motor commands or uncalibrated distance claims"}
    destination.with_suffix(".json").write_text(json.dumps(metadata, indent=2)+"\n")
    print("Detector installed and verified:", destination)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--destination", default="/var/lib/2pybot/models/ssd-mobilenet-v1-int8.onnx")
    install(parser.parse_args().destination)
