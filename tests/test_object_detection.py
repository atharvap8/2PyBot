import importlib.util
from pathlib import Path
import numpy as np

source=Path(__file__).resolve().parents[1]/"software/radxa/console/console/object_detection.py"
spec=importlib.util.spec_from_file_location("object_detection",source)
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)


def test_coco_outputs_use_correct_box_order_and_unknown_distance():
    outputs={"num_detections:0":np.array([2]),"detection_boxes:0":np.array([[[.1,.2,.7,.8],[0,0,1,1]]]),
             "detection_scores:0":np.array([[.9,.1]]),"detection_classes:0":np.array([[1,73]])}
    result=module.decode_outputs(outputs,.5)
    assert len(result)==1 and result[0]["label"]=="person"
    assert result[0]["box"]==[.2,.1,.8,.7]
    assert result[0]["distance_m"] is None
    assert module.decode_outputs({name.split(':')[0]:value for name,value in outputs.items()},.5)==result


def test_tracker_reuses_ids_without_assigning_identity_across_classes():
    tracker=module.Tracks()
    first=tracker.update([{"class_id":1,"box":[.1,.1,.4,.6],"score":.9}],10)[0]
    moved=tracker.update([{"class_id":1,"box":[.12,.1,.42,.6],"score":.8}],11)[0]
    assert first["track_id"]==moved["track_id"]
    other=tracker.update([{"class_id":73,"box":[.12,.1,.42,.6],"score":.8}],12)[0]
    assert other["track_id"]!=first["track_id"]
    expired=tracker.update([{"class_id":1,"box":[.12,.1,.42,.6],"score":.8}],20)[0]
    assert expired["track_id"]!=first["track_id"]
