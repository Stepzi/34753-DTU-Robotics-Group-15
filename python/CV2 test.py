#!/usr/bin/python
import cv2
import numpy as np

# Scale of the text
scale = 2
# Camera
cap = cv2.VideoCapture(1)
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

        # Find contours
        contours, _ = cv2.findContours(canny, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            # Filter small or irrelevant contours by area
            area = cv2.contourArea(contour)
            if area < 500:
                continue

            # Approximate the contour to a polygon
            approx = cv2.approxPolyDP(contour, cv2.arcLength(contour, True) * 0.02, True)
            x, y, w, h = cv2.boundingRect(approx)

            # Calculate the aspect ratio to filter out non-symmetrical shapes
            aspect_ratio = float(w) / h
            if 0.8 <= aspect_ratio <= 1.2:
                # Check for circles using circularity
                perimeter = cv2.arcLength(contour, True)
                circularity = 4 * np.pi * (area / (perimeter * perimeter))
                if 0.7 <= circularity <= 1.2:  # Circularity close to 1 indicates a circle
                    cv2.putText(frame, "CIRCLE", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 255, 0), 2)
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                else:
                    # Detect X marks by checking for two intersecting lines
                    roi = gray[y:y + h, x:x + w]
                    lines = cv2.HoughLinesP(cv2.Canny(roi, 50, 150), 1, np.pi / 180, 50, minLineLength=w // 2, maxLineGap=10)
                    if lines is not None and len(lines) >= 2:
                        for i in range(len(lines)):
                            for j in range(i + 1, len(lines)):
                                line1 = lines[i][0]
                                line2 = lines[j][0]
                                angle_diff = abs(
                                    np.arctan2(line1[3] - line1[1], line1[2] - line1[0]) -
                                    np.arctan2(line2[3] - line2[1], line2[2] - line2[0])
                                )
                                # Check if two lines form an approximate 90-degree angle (cross)
                                if 85 <= np.degrees(angle_diff) <= 95:
                                    cv2.putText(frame, "X MARK", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 255), 2)
                                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
                                    break

        # Display the resulting frame
        cv2.imshow('frame', frame)
        cv2.imshow('canny', canny)
        if cv2.waitKey(1) & 0xFF == ord('q'):  # If 'q' is pressed
            break

# When everything is done, release the capture
cap.release()
cv2.destroyAllWindows()
