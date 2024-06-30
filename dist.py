import cv2
import time
from ultralytics import YOLO

# Load YOLOv8 model
model = YOLO('best.pt', task='detect')
print(model.names)

# Initialize web camera
webcamera = cv2.VideoCapture(0)
webcamera.set(3,640)
webcamera.set(4,480)
# Parameters for distance estimation
KNOWN_HEIGHT = 0.3  # meters (example: height of a person)
FOCAL_LENGTH = 640 # example value in pixels (calibrate your camera to get this value)

prev_frame_time = 0
new_frame_time = 0

def estimate_distance(object_height, perceived_height, focal_length):
    """Estimate the distance from the camera to the object."""
    return (object_height * focal_length) / perceived_height

while True:
    ret, frame = webcamera.read()
    if not ret:
        break

    font = cv2.FONT_HERSHEY_SIMPLEX
    new_frame_time = time.time()
    fps = 1 / (new_frame_time - prev_frame_time)
    prev_frame_time = new_frame_time
    fps = int(fps)
    fps = str(fps)

    # Perform object detection
    results = model.predict(frame, classes=[0, 1], conf=0.5, imgsz=640)
    
    # Calculate the center of the image for direction calculation
    image_center = frame.shape[1] / 2
    
    # Draw FPS on the frame
    cv2.putText(frame, fps, (7, 20), font, 1, (100, 255, 0), 1, cv2.LINE_AA)

    # Loop through detected objects
    for result in results[0].boxes:
        x, y, w, h = result.xywh[0]
        perceived_height = h - y
        distance = estimate_distance(KNOWN_HEIGHT, perceived_height, FOCAL_LENGTH)
        object_center = (x + w) / 2
        direction = 'right' if object_center < image_center else 'left'
        
        # if 0 < distance <=30:
        #     command -

        # Draw bounding box and annotations on the frame
        cv2.rectangle(frame, (int(x), int(y)), (int(w), int(h)), (0, 255, 0), 2)
        cv2.putText(frame, f'Distance: {distance:.2f}cm', (int(x), int(y) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        cv2.putText(frame, f'Direction: {direction}', (int(x), int(h) + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    
    # Display the total count of detected objects
    cv2.putText(frame, f"Total: {len(results[0].boxes)}", (10, 320), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)

    # Show the frame with detections
    cv2.imshow("Live Camera", frame)

    if cv2.waitKey(1) == ord('q'):
        break

webcamera.release()
cv2.destroyAllWindows()