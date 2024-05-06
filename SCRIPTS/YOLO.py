from ultralytics import YOLO
import json
from PIL import Image
import requests
from flask import Flask, request, jsonify
import time
import numpy as np
import cv2
import socket
def get_local_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    # We use the Google DNS here, but we won't actually make a request to Google
    # This is just to allow the socket library to find the most appropriate network interface to use.
    s.connect(("8.8.8.8", 80))
    local_ip = s.getsockname()[0]
    s.close()
    return local_ip
def process_results(results):
    # Process results list
    boxes = results[0].boxes
    classifs = boxes.cls.cpu().numpy()
    confs = boxes.conf.cpu().numpy()
    centers = boxes.xywh.cpu().numpy()
    object_list = []
    cnt = 0
    for cls in classifs:
        object = {}
        obj_name = label_dict[cls]
        object['obj_name'] = obj_name
        conf = confs[cnt]
        object['conf'] = conf
        center = (centers[cnt, 0], centers[cnt, 1])
        object['center'] = center
        object_list.append(object)
        cnt+=1
    return object_list
train = True
if train:
    model = YOLO("yolov8m.pt")
    n_epochs = 20
    batch = 6
    exp_name='PC_Detection'
    optimizer = 'auto'
    seed = 0
    overwrite = True
    model.train(data="datasets/data.yaml", epochs=n_epochs, workers=0, batch=batch, name=exp_name, seed=seed,
                exist_ok=overwrite)
else:
    model = YOLO('C:/Users/nicos/VS Code/ObjDet/runs/detect/PC_Detection/weights/best.pt')
label_dict = model.names
app = Flask(__name__)
@app.route('/analyze', methods=['POST'])
def analyze():
    start = time.time()
    image_data = request.data
    image_nparr = np.frombuffer(image_data, np.uint8)
    image = cv2.imdecode(image_nparr, cv2.IMREAD_COLOR)
    results = model(image)
    # results = model('datasets/valid/images/pc11_jpg.rf.e9b5d8f92e7f66979629d69967b3abbb.jpg')
    object_list = process_results(results)
    end = time.time()
    total_t = end - start
    print(f'Total time of computation: {total_t}')
    return jsonify({'results': object_list})
if __name__ == "__main__":
    app.run(host=get_local_ip(), port=5000)  # will accept incoming connections no matter the network adapter or the ip address (possible production issue!)
