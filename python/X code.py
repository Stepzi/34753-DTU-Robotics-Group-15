import cv2
import numpy as np

# Function to check if two lines intersect
def do_lines_intersect(p1, p2, p3, p4):
    # Calculate the determinant (cross product) to check if lines intersect
    d1 = (p2[0] - p1[0]) * (p4[1] - p3[1]) - (p2[1] - p1[1]) * (p4[0] - p3[0])
    d2 = (p1[0] - p3[0]) * (p4[1] - p3[1]) - (p1[1] - p3[1]) * (p4[0] - p3[0])
    d3 = (p1[0] - p3[0]) * (p2[1] - p1[1]) - (p1[1] - p3[1]) * (p2[0] - p1[0])

    return d1 != 0 and d2 != 0 and d3 != 0  # Check if lines are not parallel

def run_ex6():
    # Start video capture from the webcam (0 for default camera)
    cap = cv2.VideoCapture(0)  # Open the default camera
    if not cap.isOpened():  # Check if camera is opened
        print("Error: Could not open camera.")
        return -1

    x_count = 0  # Variable to count detected X shapes

    while True:
        ret, frame = cap.read()  # Capture frame from the camera
        if not ret:  # If frame is empty, break the loop
            print("Error: Empty frame.")
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # Convert to grayscale
        blurred = cv2.GaussianBlur(gray, (5, 5), 1.5)  # Apply Gaussian blur to reduce noise
        edges = cv2.Canny(blurred, 50, 150, 3)  # Apply Canny edge detection

        # Use Hough Line Transform to detect lines
        lines = cv2.HoughLines(edges, 1, np.pi / 180, 150)  # Hough Transform to detect lines

        intersections = []  # List to store intersection points

        # If lines are detected
        if lines is not None:
            for i in range(len(lines)):
                rho, theta = lines[i][0]
                a = np.cos(theta)
                b = np.sin(theta)
                x0 = a * rho
                y0 = b * rho
                pt1 = (int(x0 + 1000 * (-b)), int(y0 + 1000 * (a)))
                pt2 = (int(x0 - 1000 * (-b)), int(y0 - 1000 * (a)))

                # Draw the detected lines
                cv2.line(frame, pt1, pt2, (0, 255, 0), 2)

                # Check for intersections with every other line
                for j in range(i + 1, len(lines)):
                    rho2, theta2 = lines[j][0]
                    a2 = np.cos(theta2)
                    b2 = np.sin(theta2)
                    x02 = a2 * rho2
                    y02 = b2 * rho2
                    pt3 = (int(x02 + 1000 * (-b2)), int(y02 + 1000 * (a2)))
                    pt4 = (int(x02 - 1000 * (-b2)), int(y02 - 1000 * (a2)))

                    # Check if the two lines intersect
                    if do_lines_intersect(pt1, pt2, pt3, pt4):
                        intersections.append(pt1)  # Store intersection point (could be any point on the lines)

        # If there are two or more intersections, we have detected an "X"
        if len(intersections) >= 2:
            x_count += 1  # Increment the "X" detection count
            print(f"X detected ({x_count})")  # Output the detection count

        # Display the resulting frame with the detected "X"
        cv2.imshow("X Shape Detection", frame)

        # Break the loop if the user presses the 'q' key
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the capture and close the window
    cap.release()  # Release the webcam
    cv2.destroyAllWindows()  # Close all OpenCV windows

# Run the function
if __name__ == "__main__":
    run_ex6()
