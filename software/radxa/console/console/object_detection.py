"""Bounded, snapshot-based ONNX perception. No serial or motor access."""
import asyncio
import copy
import importlib.util
import os
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

COCO = {
    1:"person",2:"bicycle",3:"car",4:"motorcycle",5:"airplane",6:"bus",7:"train",8:"truck",9:"boat",10:"traffic light",11:"fire hydrant",
    13:"stop sign",14:"parking meter",15:"bench",16:"bird",17:"cat",18:"dog",19:"horse",20:"sheep",21:"cow",22:"elephant",23:"bear",24:"zebra",25:"giraffe",
    27:"backpack",28:"umbrella",31:"handbag",32:"tie",33:"suitcase",34:"frisbee",35:"skis",36:"snowboard",37:"sports ball",38:"kite",39:"baseball bat",40:"baseball glove",
    41:"skateboard",42:"surfboard",43:"tennis racket",44:"bottle",46:"wine glass",47:"cup",48:"fork",49:"knife",50:"spoon",51:"bowl",52:"banana",53:"apple",54:"sandwich",
    55:"orange",56:"broccoli",57:"carrot",58:"hot dog",59:"pizza",60:"donut",61:"cake",62:"chair",63:"couch",64:"potted plant",65:"bed",67:"dining table",70:"toilet",
    72:"tv",73:"laptop",74:"mouse",75:"remote",76:"keyboard",77:"cell phone",78:"microwave",79:"oven",80:"toaster",81:"sink",82:"refrigerator",84:"book",85:"clock",
    86:"vase",87:"scissors",88:"teddy bear",89:"hair drier",90:"toothbrush",
}


def overlap(a,b):
    width=max(0,min(a[2],b[2])-max(a[0],b[0]));height=max(0,min(a[3],b[3])-max(a[1],b[1]))
    intersection=width*height
    union=(a[2]-a[0])*(a[3]-a[1])+(b[2]-b[0])*(b[3]-b[1])-intersection
    return intersection/union if union>0 else 0


class Tracks:
    def __init__(self):self.items={};self.next_id=1
    def update(self,detections,now):
        self.items={key:value for key,value in self.items.items() if now-value["seen"]<3.5}
        used=set()
        for detection in sorted(detections,key=lambda item:item["score"],reverse=True):
            candidates=[(overlap(detection["box"],track["box"]),key) for key,track in self.items.items()
                        if key not in used and track["class_id"]==detection["class_id"]]
            best=max(candidates,default=(0,None))
            if best[0]>=.30:identifier=best[1]
            else:identifier=self.next_id;self.next_id+=1
            used.add(identifier);detection["track_id"]=identifier
            self.items[identifier]={"box":detection["box"],"class_id":detection["class_id"],"seen":now}
        return detections


def decode_outputs(outputs,threshold=.50,people_only=False):
    tensors={name.split(':')[0]:value for name,value in outputs.items()}
    count=int(tensors["num_detections"].reshape(-1)[0])
    boxes=tensors["detection_boxes"][0];scores=tensors["detection_scores"][0];classes=tensors["detection_classes"][0]
    detections=[]
    for i in range(min(count,len(boxes),len(scores),len(classes),100)):
        category=int(classes[i]);score=float(scores[i])
        if score<threshold or (people_only and category!=1):continue
        top,left,bottom,right=map(float,boxes[i])
        box=[max(0,min(1,left)),max(0,min(1,top)),max(0,min(1,right)),max(0,min(1,bottom))]
        if box[2]<=box[0] or box[3]<=box[1]:continue
        detections.append({"class_id":category,"label":COCO.get(category,"object "+str(category)),
                           "score":round(score,4),"box":box,"distance_m":None})
    return detections


class ObjectDetector:
    def __init__(self,frame_source,intrinsics_source,model_path=None):
        self.frame_source=frame_source;self.intrinsics_source=intrinsics_source
        self.model_path=Path(model_path or os.environ.get("BOT_ML_MODEL","/var/lib/2pybot/models/ssd-mobilenet-v1-int8.onnx"))
        self.lock=threading.RLock();self.executor=ThreadPoolExecutor(max_workers=1,thread_name_prefix="object-detector")
        self.enabled=False;self.threshold=.50;self.people_only=False;self.corrected=True
        self.session=None;self.preview=None;self.last_frame=None;self.tracks=Tracks();self.last_inference=0.0
        self.result={"state":"off","detections":[],"message":"Enable object detection to start"}

    def configure(self,enabled,threshold=.50,people_only=False,corrected=True):
        if not .15<=threshold<=.95:raise ValueError("Confidence must be 0.15–0.95")
        if enabled and not self.model_path.is_file():raise ValueError("Detector model is not installed")
        with self.lock:
            self.enabled=bool(enabled);self.threshold=threshold;self.people_only=bool(people_only);self.corrected=bool(corrected)
            self.last_frame=None
            if not enabled:self.result=dict(self.result,state="off",detections=[],message="Detection disabled")
        return self.status()

    def status(self):
        with self.lock:
            result=copy.deepcopy(self.result)
            result.update(enabled=self.enabled,threshold=self.threshold,people_only=self.people_only,corrected=self.corrected,
                          model_installed=self.model_path.is_file(),runtime_available=importlib.util.find_spec("onnxruntime") is not None,
                          model="SSD-MobileNetV1 INT8",provider="CPUExecutionProvider",
                          age_ms=round((time.monotonic()-self.last_inference)*1000) if self.last_inference else None,
                          measured_distance_available=False)
            return result

    def tick(self):
        with self.lock:
            if not self.enabled:return
            threshold=self.threshold;people_only=self.people_only;corrected=self.corrected
        import cv2
        import numpy as np
        if self.session is None:
            import onnxruntime as ort
            settings=ort.SessionOptions();settings.intra_op_num_threads=1;settings.inter_op_num_threads=1
            self.session=ort.InferenceSession(str(self.model_path),sess_options=settings,providers=["CPUExecutionProvider"])
        jpeg,timing=self.frame_source()
        if not jpeg:
            self.result={"state":"waiting","detections":[],"message":"Waiting for fresh camera frame"};return
        frame_id=timing.get("frame_id")
        if frame_id==self.last_frame:return
        self.last_frame=frame_id
        image=cv2.imdecode(np.frombuffer(jpeg,np.uint8),cv2.IMREAD_COLOR)
        if image is None:raise ValueError("Camera frame could not be decoded")
        intrinsics=self.intrinsics_source() if corrected else None
        corrected_applied=False
        if intrinsics and intrinsics.get("quality",{}).get("passed"):
            from .calibration_math import rectify
            image=rectify(image,intrinsics,960);corrected_applied=True
        else:
            image=cv2.resize(image,(960,round(960*image.shape[0]/image.shape[1])),interpolation=cv2.INTER_AREA)
        tensor=cv2.cvtColor(cv2.resize(image,(300,300),interpolation=cv2.INTER_AREA),cv2.COLOR_BGR2RGB)[None].astype(np.uint8)
        started=time.monotonic()
        names=[value.name for value in self.session.get_outputs()]
        values=self.session.run(names,{self.session.get_inputs()[0].name:tensor})
        inference_ms=(time.monotonic()-started)*1000
        detections=decode_outputs(dict(zip(names,values)),threshold,people_only)
        now=time.monotonic();detections=self.tracks.update(detections,now)
        height,width=image.shape[:2]
        for detection in detections:
            x1,y1,x2,y2=detection["box"];p=(round(x1*width),round(y1*height));q=(round(x2*width),round(y2*height))
            color=(210,230,40) if detection["class_id"]==1 else (90,220,110)
            cv2.rectangle(image,p,q,color,2)
            label=f'{detection["label"]} #{detection["track_id"]} {detection["score"]:.0%}'
            cv2.putText(image,label,(p[0],max(16,p[1]-7)),cv2.FONT_HERSHEY_SIMPLEX,.55,color,2,cv2.LINE_AA)
        image=cv2.putText(image,f'ONNX CPU {inference_ms:.0f} ms | depth distance uncalibrated',(10,height-12),cv2.FONT_HERSHEY_SIMPLEX,.45,(220,220,220),1,cv2.LINE_AA)
        preview=cv2.imencode(".jpg",image,[cv2.IMWRITE_JPEG_QUALITY,88])[1].tobytes()
        with self.lock:
            if not self.enabled:return
            self.preview=preview;self.last_inference=now
            self.result={"state":"running","detections":detections,"inference_ms":round(inference_ms,2),
                         "frame_id":frame_id,"timing":timing,"image_size":[width,height],
                         "corrected_applied":corrected_applied,"message":"2D object detection; distances require camera–ToF alignment"}

    async def run(self):
        try:
            while True:
                try:await asyncio.get_running_loop().run_in_executor(self.executor,self.tick)
                except Exception as exc:
                    self.result={"state":"error","detections":[],"message":str(exc)[-500:]}
                await asyncio.sleep(.9)
        finally:self.executor.shutdown(wait=False)
