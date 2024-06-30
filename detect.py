import cv2
import time
from ultralytics import YOLO

model = YOLO('best.engine', task='detect')
print(model.names)
webcamera = cv2.VideoCapture(0)
webcamera.set(3, 640)
webcamera.set(4, 480)

prev_frame_time = 0
new_frame_time = 0

while True:
    ret, frame = webcamera.read()
    
    font = cv2.FONT_HERSHEY_SIMPLEX 
    # time when we finish processing for this frame 
    new_frame_time = time.time() 
  
    # Calculating the fps 
  
    # fps will be number of frame processed in given time frame 
    # since their will be most of time error of 0.001 second 
    # we will be subtracting it to get more accurate result 
    fps = 1/(new_frame_time-prev_frame_time) 
    prev_frame_time = new_frame_time 
  
    # converting the fps into integer 
    fps = int(fps) 
  
    # converting the fps to string so that we can display it on frame 
    # by using putText function 
    fps = str(fps) 
  
    # putting the FPS count on the frame 
    

    results = model.predict(frame, classes=[0,1], conf=0.5, imgsz=640)
    cv2.putText(frame, f"Total: {len(results[0].boxes)}", (10, 320), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)
    cv2.putText(frame, fps, (7, 20), font, 1, (100, 255, 0), 1, cv2.LINE_AA) 
    cv2.imshow("Live Camera", results[0].plot())

    if cv2.waitKey(1) == ord('q'):
        break

webcamera.release()
cv2.destroyAllWindows()
