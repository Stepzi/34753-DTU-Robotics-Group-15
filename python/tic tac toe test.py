import cv2
import numpy as np

# Open a connection to the default camera (0)
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Error: Could not access the camera.")
    exit()

print("Press 'q' to quit.")

# Function to identify the cell in which a circle is detected
def get_cell(grid, x, y):
    grid_x, grid_y, grid_w, grid_h = cv2.boundingRect(grid)
    cell_width = grid_w // 3
    cell_height = grid_h // 3

    col = (x - grid_x) // cell_width
    row = (y - grid_y) // cell_height

    if 0 <= row < 3 and 0 <= col < 3:
        return int(row), int(col)
    return None

# Store the board state
board = [["" for _ in range(3)] for _ in range(3)]

while True:
    # Capture frame-by-frame
    ret, frame = cap.read()
    if not ret:
        print("Error: Failed to capture frame.")
        break

    # Convert to grayscale and blur the image
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)

    # Apply edge detection
    edges = cv2.Canny(blurred, 50, 150)

    # Find contours
    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    detected_grid = None

    for contour in contours:
        # Approximate the contour to a polygon
        epsilon = 0.02 * cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, epsilon, True)

        # Check if the polygon has 4 sides (rectangle)
        if len(approx) == 4:
            # Get the bounding box of the polygon
            x, y, w, h = cv2.boundingRect(approx)
            aspect_ratio = float(w) / h

            # Check if the rectangle is roughly square and of reasonable size
            if 0.8 <= aspect_ratio <= 1.2 and w > 100 and h > 100:
                detected_grid = approx
                break

    # Detect and process circles
    if detected_grid is not None:
        # Draw the detected grid
        cv2.drawContours(frame, [detected_grid], -1, (0, 255, 0), 3)
        cv2.putText(frame, "Tic-Tac-Toe Grid Detected", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        # Mask the grid area for circle detection
        mask = np.zeros_like(gray)
        cv2.drawContours(mask, [detected_grid], -1, 255, -1)
        masked_gray = cv2.bitwise_and(gray, gray, mask=mask)

        # Detect circles using HoughCircles
        circles = cv2.HoughCircles(
            masked_gray, cv2.HOUGH_GRADIENT, dp=1.2, minDist=20,
            param1=50, param2=30, minRadius=10, maxRadius=50
        )

        if circles is not None:
            circles = np.round(circles[0, :]).astype("int")
            for (x, y, r) in circles:
                # Determine the cell of the circle
                cell = get_cell(detected_grid, x, y)
                if cell and board[cell[0]][cell[1]] == "":
                    board[cell[0]][cell[1]] = "O"
                    cv2.circle(frame, (x, y), r, (0, 255, 0), 2)
                    cv2.putText(frame, f"Circle in {cell}", (x - 40, y - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    # Display the frame
    cv2.imshow("Tic-Tac-Toe Detection", frame)

    # Exit if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the camera and close all OpenCV windows
cap.release()
cv2.destroyAllWindows()
