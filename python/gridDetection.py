import os
import cv2
import numpy as np
from scipy.spatial import distance
import time

def image_processing(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    _, thresholded = cv2.threshold(gray, 20  , 255, cv2.THRESH_BINARY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    return blurred

def detect_intersections(image, epsilon=20):
    """
    Detects the intersections of the Tic Tac Toe grid in the image and filters duplicates.
    
    Args:
    - image: The original image.
    - epsilon: The maximum distance to consider points as duplicates.

    Returns:
    - List of unique intersection points.
    """
    
    edges = cv2.Canny(image, 50, 150, apertureSize=3)
    lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=50, minLineLength=50, maxLineGap=10)

    vertical_lines = []
    horizontal_lines = []

    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            if abs(x1 - x2) < epsilon:  # Treat almost-vertical as vertical
                vertical_lines.append((x1, y1, x2, y2))
            elif abs(y1 - y2) < epsilon:  # Treat almost-horizontal as horizontal
                horizontal_lines.append((x1, y1, x2, y2))

    intersections = []
    for vert in vertical_lines:
        for hor in horizontal_lines:
            x1, y1, x2, y2 = vert
            x3, y3, x4, y4 = hor
            intersection_x = x1  # Vertical line's x-coordinate
            intersection_y = y3  # Horizontal line's y-coordinate
            intersections.append((intersection_x, intersection_y))

    # Filter duplicate intersections
    unique_intersections = []
    for pt in intersections:
        if not any(distance.euclidean(pt, existing) < epsilon for existing in unique_intersections):
            unique_intersections.append(pt)

    return unique_intersections

def calculate_centroids(intersections):
    """
    Calculate the centroid of the four groups of intersections corresponding to the 4 cells.
    """
    intersections = sorted(intersections, key=lambda x: x[1])

    top_left = []
    top_right = []
    bottom_left = []
    bottom_right = []

    for intersection in intersections:
        x, y = intersection
        if x < np.median([i[0] for i in intersections]):  # left side
            if y < np.median([i[1] for i in intersections]):  # top side
                top_left.append(intersection)
            else:  # bottom side
                bottom_left.append(intersection)
        else:  # right side
            if y < np.median([i[1] for i in intersections]):  # top side
                top_right.append(intersection)
            else:  # bottom side
                bottom_right.append(intersection)

    centroids = []
    for group in [top_left, top_right, bottom_left, bottom_right]:
        if group:
            centroid_x = np.mean([pt[0] for pt in group])
            centroid_y = np.mean([pt[1] for pt in group])
            centroids.append((centroid_x, centroid_y))

    return centroids

def sort_centroids(centroids):
    """
    Sort the centroids to identify top-left, top-right, bottom-left, and bottom-right.
    
    Args:
    - centroids: List of centroids [(x, y)].
    
    Returns:
    - Sorted list of centroids: [top-left, top-right, bottom-left, bottom-right]
    """
    # Sort centroids based on x first (left to right) then by y (top to bottom)
    sorted_by_x = sorted(centroids, key=lambda x: x[0])  # Sort by x-coordinate
    top_left = sorted_by_x[0]  # Top-left is the one with the smallest x
    top_right = sorted_by_x[1]  # Second smallest x is the top-right
    bottom_left = sorted_by_x[2]  # Third smallest x is the bottom-left
    bottom_right = sorted_by_x[3]  # Largest x is the bottom-right
    
    return [top_left, top_right, bottom_left, bottom_right]

def generate_grid_map(centroid, centroids, image_width, image_height):
    """
    Generate fake centroids based on the given centroid and offsets from surrounding centroids.
    Uses the top-left centroid (with smallest x and y values) as reference.
    """
    # Find the top-left centroid by picking the one with smallest x and y coordinates
    top_left_centroid = min(centroids, key=lambda pt: (pt[0], pt[1]))  # Smallest x and y
    
    x, y = top_left_centroid  # Now we are guaranteed to use the top-left centroid
    x_dist = abs(top_left_centroid[0] - centroids[1][0])  # Difference in x with top-right
    y_dist = abs(top_left_centroid[1] - centroids[2][1])  # Difference in y with bottom-left
    offset = (x_dist + y_dist) / 2  # Average of these distances for the offset

    grid_map = [
        (x - offset, y - offset),  # 1
        (x, y - offset),           # 2
        (x + offset, y - offset),  # 3
        (x + 2 * offset, y - offset),  # 4
        (x - offset, y),           # 5
        (x, y),                    # 6
        (x + offset, y),           # 7
        (x + 2 * offset, y),       # 8
        (x - offset, y + offset),  # 9
        (x, y + offset),           # 10
        (x + offset, y + offset),  # 11
        (x + 2 * offset, y + offset),  # 12
        (x - offset, y + 2 * offset),  # 13
        (x, y + 2 * offset),       # 14
        (x + offset, y + 2 * offset),  # 15
        (x + 2 * offset, y + 2 * offset),  # 16
    ]
    
    return grid_map

def display_centroids(image, centroids, color=(0, 255, 0)):
    """
    Display the centroids on the image.
    """
    image_with_centroids = image.copy()
    for centroid in centroids:
        cv2.circle(image_with_centroids, (int(centroid[0]), int(centroid[1])), 10, color, -1)
    return image_with_centroids

def crop_square(image, corners):
    """
    Crop a square from the given image based on the corners provided.
    """
    x_min = int(min(corner[0] for corner in corners))
    x_max = int(max(corner[0] for corner in corners))
    y_min = int(min(corner[1] for corner in corners))
    y_max = int(max(corner[1] for corner in corners))
    cropped_image = image[y_min:y_max, x_min:x_max]
    return cropped_image
 
def click_event(event, x, y, flags, param):
    """
    Callback function to handle mouse click events.
    """
    if event == cv2.EVENT_LBUTTONDOWN:
        print(f"Clicked at: ({x}, {y})")

def grid_detection(image, debug):
    cv2.imshow("Tic Tac Toe Grid", image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    image_height, image_width = image.shape[:2]

    # Detect intersections and centroids
    intersections = detect_intersections(image)
    centroids = calculate_centroids(intersections)
    grid_map = generate_grid_map(centroids[0], centroids, image_width, image_height)

    if debug:
        # Print all unique intersections
        print("Unique Intersection Coordinates:")
        for i, intersection in enumerate(intersections):
            print(f"Intersection {i + 1}: {intersection}")

        # Display intersections
        intersection_image = image.copy()
        for intersection in intersections:
            cv2.circle(intersection_image, intersection, 5, (0, 255, 0), -1)
        cv2.imshow("Intersections", intersection_image)
        cv2.setMouseCallback("Intersections", click_event)
        cv2.waitKey(0)

        # Display centroids
        original_centroid_image = display_centroids(image, centroids, color=(0, 255, 0))
        cv2.imshow("Original Centroids", original_centroid_image)
        cv2.waitKey(0)

        # Print fake centroids for verification
        print("Grid Map Coordinates:")
        for i, fc in enumerate(grid_map):
            print(f"Grid {i + 1}: {fc}")

        grid_map_image = display_centroids(image, grid_map, color=(255, 0, 0))
        cv2.imshow("Grid Map ", grid_map_image)
        cv2.waitKey(0)

    # Crop and display all 9 squares 
    square_coords = [
        [0, 1, 4, 5], [1, 2, 5, 6], [2, 3, 6, 7],
        [4, 5, 8, 9], [5, 6, 9, 10], [6, 7, 10, 11],
        [8, 9, 12, 13], [9, 10, 13, 14], [10, 11, 14, 15]
    ] 

    squares = []  # List to store the cropped square images

    for coords in square_coords:
        corners = [grid_map[coords[0]], grid_map[coords[1]], grid_map[coords[2]], grid_map[coords[3]]]
        square_image = crop_square(image, corners)
        squares.append(square_image)

        if debug:
            square_height, square_width = squares[0].shape[:2]
            start_x, start_y = 20, 20
            spacing = 10
            window_extra_width = 16  # Typical width for window borders
            window_extra_height = 39  # Typical height for title bar and borders
            adjusted_width = square_width + window_extra_width
            adjusted_height = square_height + window_extra_height

            for i, square in enumerate(squares):
                x_offset = start_x + (i % 3) * (adjusted_width + spacing)  
                y_offset = start_y + (i // 3) * (adjusted_height + spacing)   

                window_name = f"Square {i + 1}"
                cv2.imshow(window_name, square)
                cv2.moveWindow(window_name, x_offset, y_offset)    
            cv2.waitKey(0)
            
    cv2.destroyAllWindows()
    return squares  # Return the list of squares

def pixel_average(squares, debug):
    averages = []
    for i, square in enumerate(squares):
        avg_value = np.mean(square)
        averages.append(avg_value)
        if debug:
            print(f"Avg {i+1}: {float(avg_value)}")
    return averages
 
def pixel_difference(start_averages, averages, debug, threshold):
    if start_averages is None:
        print("Error: start_averages is None.")
        return
    if averages is None:
        print("Error: averages is None.")
        return

    returning_values = []

    for i, (start_avg, avg) in enumerate(zip(start_averages, averages)):
        diff = start_avg - avg
        if diff > threshold:  # Check if the difference exceeds the threshold
            returning_values.append(i + 1)  # Append 1-based index
            if debug:
                print(f"Diff {i + 1}: {float(diff)}")  
            return i+1
    return None

def determine_inputs(pixel_diffs, robot_move, threshold):
    changed_squares = []
    for i, diff_value in enumerate(pixel_diffs):
        if abs(diff_value) > threshold:  
            changed_squares.append(i)

    if len(changed_squares) == 2:
        human_square = next(i for i in changed_squares if i != robot_move)
        print(f"Robot's move was in square {robot_move + 1}.")
        print(f"Human's move was in square {human_square + 1}.")
    else:
        print("There was an unexpected number of changes. Something went wrong.")

def int_to_grid(input):
	cell = (0,0)
	if input == 1:
		cell = (0,0)
	if input == 2:
		cell = (0,1)
	if input == 4:
		cell = (1,0)
	if input == 3:
		cell = (0,2)
	if input == 5:
		cell = (1,1)
	if input == 6:
		cell = (1,2)
	if input == 7:
		cell = (2,0)
	if input == 8:
		cell = (2,1)
	if input == 9:
		cell = (2,2)

	return cell

def move_detection():
    debug = False 
    live_feed = True

    if live_feed:
        cap = cv2.VideoCapture(1)  # 0 for the default webcam, change to 1 or 2 for other webcams
        if not cap.isOpened():
            print("Error: Could not open webcam.")
            exit()
        ret, raw_image = cap.read()
        cv2.imwrite("python/Pictures/img1.png", raw_image)
    else:
        image_path = "python/Pictures/img1.png"
        raw_image = cv2.imread(image_path)
        if raw_image is None:
            raise FileNotFoundError(f"Image file '{image_path}' not found.")    

    image = image_processing(raw_image)
    squares = grid_detection(image, debug)
    start_averages = pixel_average(squares, debug)

    if live_feed:
        ret, raw_image = cap.read()
        cv2.imwrite("python/Pictures/img2.png", raw_image)
    else:
        image_path = "python/Pictures/img2.png"
        raw_image = cv2.imread(image_path)
        if raw_image is None:
            raise FileNotFoundError(f"Image file '{image_path}' not found.")

    image = image_processing(raw_image)
    squares = grid_detection(image, debug)
    averages = pixel_average(squares, debug)

    diff = pixel_difference(start_averages, averages, debug, threshold=5)
    print(diff)
    return int_to_grid(diff)

cell = move_detection()
print(cell)