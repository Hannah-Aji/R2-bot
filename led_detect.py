

import cv2
import numpy as np

# Capture a frame from the camera
cap = cv2.VideoCapture(0)

while True:
    # Read the frame
    ret, frame = cap.read()
    if not ret:
        break
    
    # Convert the frame to HSV format
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Convert the frame to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Apply Gaussian blur to reduce noise
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)

    # Apply a threshold to create a binary image
    _, thresh = cv2.threshold(blurred, 200, 255, cv2.THRESH_BINARY)

    # Find contours in the binary image
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Filter contours based on area and shape
    min_area = 10  # Adjust this threshold according to your needs
    leds = []
    for contour in contours:
        area = cv2.contourArea(contour)
        if area > min_area:
            leds.append(contour)

    # Iterate over detected LEDs
    for contour in leds:
        x, y, w, h = cv2.boundingRect(contour)
        
        
        # Get average color within the bounding rectangle
        hsv_color = hsv_frame[y:y+h, x:x+w].mean(axis=(0, 1))  # Calculate average HSV color
        
        # Round HSV values to the nearest whole number
        hsv_color = np.round(hsv_color).astype(int)

        # Display the HSV color of the object
        cv2.putText(frame, f"HSV: {hsv_color}", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

        # Check if the HSV color falls within the specified range for green light
        if (55 <= hsv_color[0] <= 75 and 55 <= hsv_color[1] <= 75 and 230 <= hsv_color[2] <= 255):

            #cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            # Print a message indicating detection of a green light
            print("Green light detected." + f"HSV: {hsv_color}")
        

    # Display the frame
    cv2.imshow('LED Detection', frame)

    # Check for the 'q' key to exit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the capture and close all windows
cap.release()
cv2.destroyAllWindows()
