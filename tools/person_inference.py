"""Small CPU YOLOv8 runner shared by ROS and reproducible offline comparisons."""
from pathlib import Path
from collections import deque
import time

import cv2
import numpy as np


class PersonInference:
    def __init__(self, model, backend='opencv', width=320, height=None, threads=2,
                 confidence=.35, iou=.45):
        self.backend = backend
        self.width, self.height = width, height or width
        self.confidence, self.iou = confidence, iou
        self.last_timing = {}
        if backend == 'opencv':
            self.net = cv2.dnn.readNetFromONNX(str(model))
            self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
        elif backend == 'ncnn':
            import ncnn
            self.ncnn = ncnn
            self.net = ncnn.Net()
            self.net.opt.use_vulkan_compute = False
            self.net.opt.num_threads = threads
            self.net.opt.use_fp16_storage = True
            self.net.opt.use_fp16_arithmetic = True
            self.net.opt.use_packing_layout = True
            root = Path(model)
            if self.net.load_param(str(root/'model.ncnn.param')) != 0:
                raise ValueError('NCNN parameter file could not be loaded')
            if self.net.load_model(str(root/'model.ncnn.bin')) != 0:
                raise ValueError('NCNN weight file could not be loaded')
            self.input_name = self.net.input_names()[0]
            self.output_name = self.net.output_names()[0]
        elif backend == 'onnxruntime':
            import onnxruntime as ort
            options = ort.SessionOptions()
            options.intra_op_num_threads = threads
            options.inter_op_num_threads = 1
            options.execution_mode = ort.ExecutionMode.ORT_SEQUENTIAL
            options.add_session_config_entry('session.intra_op.allow_spinning', '0')
            options.add_session_config_entry('session.inter_op.allow_spinning', '0')
            self.net = ort.InferenceSession(str(model), sess_options=options,
                                            providers=['CPUExecutionProvider'])
            self.input_name = self.net.get_inputs()[0].name
            shape = self.net.get_inputs()[0].shape
            if shape[-2:] != [self.height, self.width]:
                raise ValueError(f'model input {shape} differs from requested {self.height}x{self.width}')
        else:
            raise ValueError('unknown inference backend: '+backend)

    def predict(self, image):
        begin = time.perf_counter()
        height, width = image.shape[:2]
        ratio = min(self.width/width, self.height/height)
        resized_width, resized_height = round(width*ratio), round(height*ratio)
        left, top = (self.width-resized_width)//2, (self.height-resized_height)//2
        canvas = np.full((self.height, self.width, 3), 114, dtype=np.uint8)
        canvas[top:top+resized_height, left:left+resized_width] = cv2.resize(
            image, (resized_width, resized_height), interpolation=cv2.INTER_LINEAR)
        if self.backend == 'ncnn':
            tensor = self.ncnn.Mat.from_pixels(
                canvas, self.ncnn.Mat.PixelType.PIXEL_BGR2RGB, self.width, self.height)
            tensor.substract_mean_normalize([], [1/255.0]*3)
        else:
            tensor = cv2.dnn.blobFromImage(canvas, 1/255.0,
                (self.width, self.height), swapRB=True, crop=False)
        ready = time.perf_counter()
        if self.backend == 'opencv':
            self.net.setInput(tensor)
            output = self.net.forward()
        elif self.backend == 'onnxruntime':
            output = self.net.run(None, {self.input_name:tensor})[0]
        else:
            with self.net.create_extractor() as extractor:
                if extractor.input(self.input_name, tensor) != 0:
                    raise RuntimeError('NCNN input failed')
                status, result = extractor.extract(self.output_name)
                if status != 0:
                    raise RuntimeError('NCNN output extraction failed')
                output = np.array(result, copy=True)
        inferred = time.perf_counter()
        persons = decode_persons(output, width, height, ratio, left, top,
                                  self.confidence, self.iou)
        finished = time.perf_counter()
        self.last_timing = {'preprocess_ms':(ready-begin)*1000,
                            'forward_ms':(inferred-ready)*1000,
                            'postprocess_ms':(finished-inferred)*1000}
        return persons


def decode_persons(output, width, height, ratio, left, top, confidence=.35, iou=.45):
    prediction = np.asarray(output)
    if prediction.ndim == 3 and prediction.shape[0] == 1:
        prediction = prediction[0]
    if prediction.ndim != 2:
        raise ValueError(f'unsupported YOLO output shape: {prediction.shape}')
    # YOLOv8 COCO exports contain 4 coordinates + 80 class scores.
    if prediction.shape[0] == 84:
        prediction = prediction.T
    if prediction.shape[1] != 84:
        raise ValueError(f'expected YOLOv8 COCO output, received {prediction.shape}')
    scores = prediction[:,4:]
    class_ids = scores.argmax(axis=1)
    confidences = scores[:,0]
    mask = (class_ids == 0) & (confidences > confidence)
    boxes = prediction[mask,:4]
    confidences = confidences[mask]
    if not len(boxes):
        return []
    low = (boxes[:,:2]-boxes[:,2:]/2-np.array([left,top]))/ratio
    high = (boxes[:,:2]+boxes[:,2:]/2-np.array([left,top]))/ratio
    # Clip both corners; retaining the old width after clipping x/y displaces
    # partially visible people and their depth sampling region.
    low = np.clip(low, [0,0], [width,height])
    high = np.clip(high, [0,0], [width,height])
    sizes = high-low
    valid = np.all(sizes>1,axis=1) & np.all(np.isfinite(low),axis=1) & np.all(np.isfinite(high),axis=1)
    rects = np.column_stack((low[valid],sizes[valid])).tolist()
    values = confidences[valid].astype(float).tolist()
    if not rects:
        return []
    indices = np.asarray(cv2.dnn.NMSBoxes(rects,values,confidence,iou)).reshape(-1)
    persons = []
    for index in indices:
        x,y,w,h = rects[int(index)]
        persons.append({'conf':round(values[int(index)],3),
                        'bbox':[round(float(v),1) for v in (x,y,w,h)],
                        'center':[round(x+w/2,1),round(y+h/2,1)]})
    persons.sort(key=lambda person:person['conf'],reverse=True)
    return persons


class ConfidenceGate:
    """Support short weak detections only near a recent confident observation.

    Never carries an old box forward: every output needs a current model box.
    Weak observations do not extend the last confident observation's lifetime.
    """
    def __init__(self, threshold=.35, support_threshold=.2, ttl=.6, match_iou=.3):
        self.threshold, self.support_threshold = threshold, support_threshold
        self.ttl, self.match_iou = ttl, match_iou
        self.history = deque(maxlen=64)

    @staticmethod
    def overlap(a, b):
        ax,ay,aw,ah = a
        bx,by,bw,bh = b
        area = max(0,min(ax+aw,bx+bw)-max(ax,bx))*max(0,min(ay+ah,by+bh)-max(ay,by))
        union = aw*ah+bw*bh-area
        return area/union if union>0 else 0.0

    def filter(self, persons, now=None):
        now = time.monotonic() if now is None else now
        while self.history and now-self.history[0][0] > self.ttl:
            self.history.popleft()
        result=[]
        for person in persons:
            if person['conf'] >= self.threshold:
                result.append(person)
                continue
            if person['conf'] < self.support_threshold:
                continue
            matched = [stamp for stamp,box in self.history
                       if self.overlap(person['bbox'],box) >= self.match_iou]
            if matched:
                supported = dict(person)
                supported['confidence_supported'] = True
                supported['confidence_support_age_ms'] = round((now-max(matched))*1000,1)
                result.append(supported)
        for person in persons:
            if person['conf'] >= self.threshold:
                self.history.append((now,list(person['bbox'])))
        return result
