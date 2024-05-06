from ultralytics import YOLO
from PIL import Image
import cv2

# Create a new YOLO model from scratch
model = YOLO('yolov8n.yaml')

# Load a pretrained YOLO model (recommended for training)
model = YOLO('yolov8n.pt')

# Train the model using the 'coco8.yaml' dataset for 3 epochs
results = model.train(data='coco8.yaml', epochs=3)

# Evaluate the model's performance on the validation set
results = model.val()

# Perform object detection on an image using the model
results = model('https://ultralytics.com/images/bus.jpg')

# Process results generator
for result in results:
    boxes = result.boxes  # Boxes object for bounding box outputs
    masks = result.masks  # Masks object for segmentation masks outputs
    keypoints = result.keypoints  # Keypoints object for pose outputs
    probs = result.probs  # Probs object for classification outputs
    result.show()  # display to screen
    result.save(filename='result.jpg')  # save to disk

# Export the model to ONNX format
success = model.export(format='onnx')
 

model = YOLO("model.pt")
# accepts all formats - image/dir/Path/URL/video/PIL/ndarray. 0 for webcam
results = model.predict(source="0")
results = model.predict(source="folder", show=True) # Display preds. Accepts all YOLO predict arguments

# from PIL
im1 = Image.open("bus.jpg")
results = model.predict(source=im1, save=True)  # save plotted images

# from ndarray
im2 = cv2.imread("bus.jpg")
results = model.predict(source=im2, save=True, save_txt=True)  # save predictions as labels

# from list of PIL/ndarray
results = model.predict(source=[im1, im2])


# Load a model
model = YOLO('yolov8n.pt')  # load an official detection model
model = YOLO('yolov8n-seg.pt')  # load an official segmentation model
model = YOLO('path/to/best.pt')  # load a custom model

# Track with the model
results = model.track(source="https://youtu.be/LNwODJXcvt4", show=True)
results = model.track(source="https://youtu.be/LNwODJXcvt4", show=True, tracker="bytetrack.yaml")


 
# Load the YOLOv8 model
model = YOLO('yolov8n.pt')
 
# Open the video file
video_path = "12.mp4"
cap = cv2.VideoCapture(video_path)
 
# Loop through the video frames
while cap.isOpened():
    # Read a frame from the video
    success, frame = cap.read()
 
    if success:
        # Run YOLOv8 inference on the frame
        results = model(frame)
 
        # Visualize the results on the frame
        annotated_frame = results[0].plot()
 
        # Display the annotated frame
        cv2.imshow("YOLOv8 Inference", annotated_frame)
 
        # Break the loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
    else:
        # Break the loop if the end of the video is reached
        break
 
# Release the video capture object and close the display window
cap.release()
cv2.destroyAllWindows()

import cv2
from ultralytics import YOLO

 
# Load the YOLOv8 model
model = YOLO('runs/yolov8n.pt')
 
# Open the video file
cap = cv2.VideoCapture(0)  
 
# Loop through the video frames
while True:
    # Read a frame from the video
    success, frame = cap.read()
 
    if success:
        # Run YOLOv8 inference on the frame
        results = model(frame)
 
        # Visualize the results on the frame
        annotated_frame = results[0].plot()
 
        # Display the annotated frame
        cv2.imshow("YOLOv8目标检测", annotated_frame)
 
        # test
        key = cv2.waitKey(1) & 0xFF
 
        # Break the loop if 'q' is pressed
        if key == ord("q"):
            break
 
    else:
        break
 
# Release the video capture object and close the display window
cap.release()
cv2.destroyAllWindows()
