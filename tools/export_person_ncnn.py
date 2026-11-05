#!/usr/bin/env python3
"""Export existing YOLOv8n weights for the Pi's CPU, recording provenance."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import sys

def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--weights',type=Path,required=True)
    parser.add_argument('--output',type=Path,default=Path.home()/'models/vision_v2')
    parser.add_argument('--runtime-dir',type=Path,default=Path.home()/'vision_runtime')
    args=parser.parse_args()
    if not args.weights.is_file():
        parser.error('weights file does not exist')
    sys.path.insert(0,str(args.runtime_dir))
    import ncnn
    import torch
    from ultralytics import YOLO
    import ultralytics
    import pnnx
    torch.set_num_threads(2)
    torch.set_num_interop_threads(1)
    args.output.mkdir(parents=True,exist_ok=True)
    destination=args.output/'yolov8n_320.pt'
    if destination.resolve()!=args.weights.resolve():
        shutil.copyfile(args.weights,destination)
    exported=Path(YOLO(str(destination)).export(
        format='ncnn',imgsz=320,batch=1,device='cpu',quantize=16))
    record={'source_weights_sha256':sha256(args.weights),
            'input_shape':[1,3,320,320],'ultralytics':ultralytics.__version__,
            'ncnn':ncnn.__version__,'pnnx':pnnx.__version__,
            'precision':'FP16 storage and arithmetic, no INT8 quantization',
            'artifacts':{name:sha256(exported/name) for name in ('model.ncnn.param','model.ncnn.bin')}}
    (exported/'provenance.json').write_text(json.dumps(record,indent=2)+'\n')
    print(json.dumps({'model_directory':str(exported),'provenance':record},indent=2))

if __name__=='__main__':
    main()
