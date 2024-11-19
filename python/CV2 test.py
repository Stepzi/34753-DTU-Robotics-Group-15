#!/usr/bin/python
import cv2
import numpy as np

# Scale of the text
scale = 2
# Camera
cap = cv2.VideoCapture(0)
print("Press 'q' to exit")

while cap.isOpened():
    # Capture frame-by-frame
    ret, frame = cap.read()
    if ret:
        # Convert to grayscale and blur the image
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)

        # Apply Canny edge detection
        canny = cv2.Canny(blurred, 100, 200)

        # Find contours and hierarchy
        contours, hierarchy = cv2.findContours(canny, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        for i, contour in enumerate(contours):
            # Calculate contour area
            area = cv2.contourArea(contour)
            if area < 500:
                continue  # Skip small contoursx``

            # Approximate the contour
            approx = cv2.approxPolyDP(contour, cv2.arcLength(contour, True) * 0.02, True)

            # Calculate circularity to check if it's a circle
            perimeter = cv2.arcLength(contour, True)
            circularity = 4 * np.pi * (area / (perimeter * perimeter)) if perimeter > 0 else 0

            if 0.7 <= circularity <= 1.2:  # Circularity close to 1 indicates a circle
                # Check hierarchy: ensure the circle is inside another shape
                if hierarchy[0][i][3] != -1:  # If it has a parent contour
                    x, y, w, h = cv2.boundingRect(contour)
                    cv2.putText(frame, "INNER CIRCLE", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 255, 0), 2)
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

        # Display the resulting frame
        cv2.imshow('frame', frame)
        cv2.imshow('canny', canny)
        if cv2.waitKey(1) & 0xFF == ord('q'):  # If 'q' is pressed
            break

# When everything is done, release the capture
cap.release()
cv2.destroyAllWindows()
