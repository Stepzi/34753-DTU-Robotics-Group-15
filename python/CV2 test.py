#!/usr/bin/python
import math
import numpy as np
import cv2

# Scale of the text
scale = 2
# Camera
cap = cv2.VideoCapture(1)
print("Press 'q' to exit")

# Define the codec and create VideoWriter object
fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter('output.avi', fourcc, 20.0, (640, 480))

# Calculate angle
def angle(pt1, pt2, pt0):
    dx1 = pt1[0][0] - pt0[0][0]
    dy1 = pt1[0][1] - pt0[0][1]
    dx2 = pt2[0][0] - pt0[0][0]
    dy2 = pt2[0][1] - pt0[0][1]
    return float((dx1 * dx2 + dy1 * dy2)) / math.sqrt(
        float((dx1 * dx1 + dy1 * dy1)) * (dx2 * dx2 + dy2 * dy2) + 1e-10
    )

while cap.isOpened():
    # Capture frame-by-frame
    ret, frame = cap.read()
    if ret:
        # Grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # Apply GaussianBlur to reduce noise
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        # Canny edge detection
        canny = cv2.Canny(blurred, 100, 200, 3)

        # Find contours
        contours, hierarchy = cv2.findContours(canny, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Draw all detected contours in green
        cv2.drawContours(frame, contours, -1, (0, 255, 0), 2)

        for contour in contours:
            # Approximate the contour
            approx = cv2.approxPolyDP(contour, cv2.arcLength(contour, True) * 0.02, True)

            # Skip small or non-convex objects
            if abs(cv2.contourArea(contour)) < 500 or not cv2.isContourConvex(approx):
                continue

            # Check for cross shape
            bounding_rect = cv2.boundingRect(contour)
            x, y, w, h = bounding_rect

            # Ensure the bounding rectangle is approximately square
            aspect_ratio = float(w) / h
            if 0.8 <= aspect_ratio <= 1.2:
                # Check for intersecting lines within the bounding box
                roi = gray[y:y + h, x:x + w]
                lines = cv2.HoughLinesP(
                    cv2.Canny(roi, 50, 150), 1, np.pi / 180, threshold=30, minLineLength=w // 2, maxLineGap=10
                )
                if lines is not None and len(lines) >= 2:
                    # Look for approximately perpendicular lines
                    for i in range(len(lines)):
                        for j in range(i + 1, len(lines)):
                            line1 = lines[i][0]
                            line2 = lines[j][0]
                            angle_diff = abs(
                                np.arctan2(line1[3] - line1[1], line1[2] - line1[0]) -
                                np.arctan2(line2[3] - line2[1], line2[2] - line2[0])
                            )
                            # If the angle difference is close to 90 degrees
                            if 85 <= np.degrees(angle_diff) <= 95:
                                cv2.putText(frame, 'CROSS', (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 255, 255), 2, cv2.LINE_AA)
                                break

        # Display the resulting frame
        out.write(frame)
        cv2.imshow('frame', frame)
        cv2.imshow('canny', canny)
        if cv2.waitKey(1) & 0xFF == ord('q'):  # If 'q' is pressed
            break

# When everything is done
