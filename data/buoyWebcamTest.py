## Uses WEBCAM

# Import libraries
from ultralytics import YOLO  # Import YOLO model from Ultralytics
import cv2                   # Import OpenCV library
import math                  # Import math module for mathematical operations


# Start webcam
cap = cv2.VideoCapture(0)     # Open 
cap.set(3, 640)               # Set frame width to 640 pixels
cap.set(4, 480)               # Set frame height to 480 pixels


model = YOLO("pep_detection_final.pt")  # Load the YOLOv8 model

classNames = ["Boat","Buoy"]

# Infinite loop to continuously capture frames from the camera
while True:
    # Read a frame from the camera
    success, img = cap.read()

    # Perform object detection using the YOLO model on the captured frame
    results = model(img, stream=True)

    # Iterate through the results of object detection
    for r in results:
        boxes = r.boxes  # Extract bounding boxes for detected objects

        # Iterate through each bounding box
        for box in boxes:
            # Extract coordinates of the bounding box
            x1, y1, x2, y2 = box.xyxy[0]
            x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)  # Convert to integer values

            # Calculate and print the confidence score of the detection
            confidence = math.ceil((box.conf[0]*100))/100
            print("Confidence --->", confidence)

            # Draw the bounding box on the frame if confidence is greater than 0.75
            if(confidence > 0.65):
                cv2.rectangle(img, (x1, y1), (x2, y2), (255, 0, 255), 3)

            # Determine and print the class name of the detected object
            cls = int(box.cls[0])
            print("Class name -->", classNames[cls])

            # Draw text indicating the class name on the frame
            org = [x1, y1]
            font = cv2.FONT_HERSHEY_SIMPLEX
            fontScale = 1
            color = (255, 0, 0)
            thickness = 2
            if(confidence > 0.75):
                cv2.putText(img, classNames[cls], org, font, fontScale, color, thickness)

    # Display the frame with detected objects in a window named "Webcam"
    cv2.imshow('Webcam', img)

    # Check for the 'q' key press to exit the loop
    if cv2.waitKey(1) == ord('q'):
        break

# Release the camera
cap.release()

# Close all OpenCV windows
cv2.destroyAllWindows()