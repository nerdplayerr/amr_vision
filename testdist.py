import cv2
import numpy as np
from ultralytics import YOLO
import math

# Load the YOLOv8 model
model = YOLO('best.pt', task='detect')  # Choose the appropriate model variant

# Class Object
class kardus:
    x = []
    y = []
    distance = []

class orang:
    x = []
    y = []
    distance = []

# Parameters
center_frame = (320, 400)
center_x = 320  # center x frame
center_y = 340  # center y frame

# Start video capture
cap = cv2.VideoCapture(0)
cap.set(3,640)
cap.set(4,480)


while True:
    success, frame = cap.read()
    if success:
        # Run YOLOv8 tracking on the frame, persisting tracks between frames
        results = model.track(frame, persist=True)
        # results = model.predict(frame, classes=[0, 1], conf=0.5, imgsz=640)

        for r in results :
            # Get the boxes and track IDs
            boxes = r.boxes

            # Visualize the results on the frame
            annotated_frame = results[0].plot()

            #if there's no item, continue
            if results[0].boxes is None or results[0].boxes.id is None:
                continue
            track_ids = results[0].boxes.id.int().cuda().tolist()


            # Plot the tracks
            for box, track_id in zip(boxes, track_ids):
                x, y, w, h = box.xywh[0]
                id = box.id.cuda()
                classes = int(box.cls.cuda())
                print(classes)
                #    track.append((float(x), float(y)))  # x, y1 center point
                #    if len(track) > 30:  # retain 90 tracks for 90 frames
                #        track.pop(0)

                annotated_frame = cv2.line(annotated_frame, center_frame, (int(x),int(y)), (0,255,0), 5)
                jarak =  math.sqrt((int(x) - center_frame[0])**2 + (int(y) - center_frame[1])**2)
                # print("jarak")
                object_center = (x + w) / 2
                # direction = 'right' if object_center < center_frame else 'left'

                if classes == 0:
                    kardus.distance.append((jarak, track_id, (int(x), int(y))))
                    # print(kardus.distance)
                    cv2.putText(annotated_frame, f"Distance: {jarak:.2f}m", (int(x), int(y) - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    # cv2.putText(annotated_frame, f"Direction: {direction}", (int(x), int(y2) - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)                

                elif classes == 1:
                    orang.distance.append((jarak, track_id, (int(x), int(y))))
                    # print(orang.distance)
                    cv2.putText(annotated_frame, f"Distance: {jarak:.2f}m", (int(x), int(y) - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    # cv2.putText(annotated_frame, f"Direction: {direction}", (int(x), int(y2) - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                
                # Draw bounding box and distance
                # cv2.rectangle(annotated_frame, (int(x), int(y)), (int(w), int(h)), (0, 255, 0), 2)
                cv2.putText(annotated_frame, f"Distance: {jarak:.2f}m", (int(x), int(y) - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                # cv2.putText(annotated_frame, f"Direction: {direction}", (int(x), int(h) - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                # print(jarak)
        cv2.putText(annotated_frame, f"Total: {len(results[0].boxes)}", (10, 380), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)
        cv2.imshow("Frame", annotated_frame)
        
    # Exit on pressing 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
