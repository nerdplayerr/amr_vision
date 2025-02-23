# OBSTACLE AVOIDANCE SYSTEM WITH YOLOV8 FOR AMR 
Note: This repository only contains packages for object detection and tracking with YOLOv8 exported to TensorRT

  This Autonomous Mobile Robot is designed to move a shelf from one point to another automatically. There are times when unwanted things might happen such as boxes or people blocking the robot's moving path. Therefore, this system is designed to overcome this problem.

  <img src="https://github.com/user-attachments/assets/fccbc03b-9c66-489d-b84b-0d184714a879"/>
  
## Hardware components:
  1. NVIDIA Jetson Orin Nano Dev Kit
  2. SSD 128 GB NVMe
  3. Webcam 1080p

## Requirement:
  1. Ultralytics YOLOv8
  2. TensorRT
  3. CUDA
  4. MQTT (https://github.com/Greatreyhan/AMR_Publisher.git) -b fix

## Watch The Demonstration Here:
<a href="http://www.youtube.com/watch?feature=player_embedded&v=M5sQa9pAAg0
" target="_blank"><img src="http://img.youtube.com/vi/M5sQa9pAAg0/0.jpg" 
alt="IMAGE ALT TEXT HERE" width="240" height="180" border="10" /></a>

YouTube: https://youtu.be/M5sQa9pAAg0?si=Cwq4DyM9dMtC3YIy

## Object Detection with Yolov8 (TensorRT)
In the object detection system, I used yolov8 with custom datasets of 6000 images consisting of 2 classes, namely box and person. This dataset was trained for 300 epochs and then converted to the TensorRT engine.

### Yolov8 to TensorRT engine
Use this command below to export TensorRT Engine(.engine) from Yolo PyTorch(.pt):

```bash
$ yolo export model=new.pt format=engine
```
this will take a while, if it is successful the .onnx and .engine files will appear in your directory.
![Image](https://github.com/user-attachments/assets/cbedd416-15c4-42ad-ac25-42e9c398d11d)

**note: change the model file into your file name.**

## Results
The FPS value generated from the TensorRT model is up to 33 FPS and is greater than the YOLOv8 model. In addition, the average speed of object detection reached 30 ms.
for running the program use this command below:
```bash
$ python3 vision.py
```
make sure you're in the right directory

![image](https://github.com/user-attachments/assets/63a42e72-4323-4ee5-a381-a91e319cce01)

## Conclusion
Based on the test results, it can be concluded that the YOLOv8 detection model trained for 300 epochs produces the best object detection with precision, recall, and accuracy values of 91%, 93%, and 95%, respectively, and an average detection speed of 45 ms (22 FPS), while the TensorRT model has a faster detection speed of 30 ms (33 FPS).
