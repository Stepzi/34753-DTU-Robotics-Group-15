import cv2
import numpy as np


class ComputerVision:
    def __init__(self,ttt):
        
        # Start the webcam feed
        print("Opening Camera...")
        self.videoCapture = cv2.VideoCapture(1)
        if self.videoCapture.isOpened():
            # Set HD resolution (1280x720)
            width = 1280  # HD width
            height = 720  # HD height
            self.videoCapture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            self.videoCapture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            self.has_camera = True
        else:
            self.has_camera = False
            
       

        self.ttt = ttt

    def captureImage(self):
         # do computer vision

        # Wait for the user to press 's' or 'q'
        print("Hit s to capture image")
        while True:
            if self.has_camera:
                ret, frame = self.videoCapture.read()
                if not ret:
                    print("Error: Unable to capture frame.")
                    break
            else:
                frame = cv2.imread("frame.jpg")

            # Display the live feed
            cv2.imshow("Webcam Feed", frame)

            # Wait for the user to press 's' or 'q'
            
            key = cv2.waitKey(1) & 0xFF
            if key == ord("s"):
                print("Image captured.")
                break
            elif key == ord("q"):
                print("Game terminated.")
                self.videoCapture.release()
                cv2.destroyAllWindows()
                exit()
            
            
        print('Performing computer vision')
        self.analyzeFrame(frame)

    def analyzeFrame(self,frame):

        # Grid parameters
        rows = 3
        columns = 3
        grid_line_thickness = 15  # Thickness of the grid lines
        black_pixel_threshold = 20000     # Threshold for number of black pixels


        scale_percent = 50  # Adjust this percentage to scale the captured image


        # Dynamically calculate the grid dimensions based on the frame
        grid_x_start = 0  # Top-left corner of the frame
        grid_y_start = 0
        grid_width = frame.shape[1]  # Use the frame's width
        grid_height = frame.shape[0]  # Use the frame's height
        cell_width = grid_width // columns
        cell_height = grid_height // rows

        median = cv2.medianBlur(frame,5)
        # Convert to grayscale
        gray = cv2.cvtColor(median, cv2.COLOR_BGR2GRAY)

        # Apply binary thresholding
        _, binary = cv2.threshold(gray, 50, 255, cv2.THRESH_BINARY)

        # Create a mask to ignore the grid lines
        grid_mask = np.ones_like(binary, dtype=np.uint8) * 255

        # Draw the grid lines on the mask
        for row in range(1, rows):
            y = grid_y_start + row * cell_height
            cv2.line(grid_mask, (grid_x_start, y), (grid_x_start + grid_width, y), 0, grid_line_thickness)
        for col in range(1, columns):
            x = grid_x_start + col * cell_width
            cv2.line(grid_mask, (x, grid_y_start), (x, grid_y_start + grid_height), 0, grid_line_thickness)

        # Apply the mask to the binary image
        masked_binary = cv2.bitwise_and(binary, grid_mask)

        # Convert the binary image to color for annotation
        annotated_binary = cv2.cvtColor(masked_binary, cv2.COLOR_GRAY2BGR)

        # Draw the grid lines dynamically
        for row in range(1, rows):
            y = grid_y_start + row * cell_height
            cv2.line(annotated_binary, (grid_x_start, y), (grid_x_start + grid_width, y), (0, 255, 0), grid_line_thickness)
        for col in range(1, columns):
            x = grid_x_start + col * cell_width
            cv2.line(annotated_binary, (x, grid_y_start), (x, grid_y_start + grid_height), (0, 255, 0), grid_line_thickness)

        # Loop through each grid cell
        for row in range(rows):
            for col in range(columns):
                x1 = grid_x_start + col * cell_width
                x2 = x1 + cell_width
                y1 = grid_y_start + row * cell_height
                y2 = y1 + cell_height

                # Crop the binary image region corresponding to the cell
                cell_region = masked_binary[y1:y2, x1:x2]

                # Count the number of black pixels (0 in the binary image)
                black_pixel_count = np.sum(cell_region == 0)

                # Display the black pixel count and threshold on the binary image
                cv2.putText(
                    annotated_binary,
                    f"Cell ({row},{col}): {black_pixel_count}",
                    (x1 + 5, y1 + 20),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.4,
                    (0, 255, 0),
                    1,
                )

                # Check if the cell is occupied (above the threshold)
                if black_pixel_count > black_pixel_threshold:
                    if self.ttt.board[row][col] is None:
                        self.ttt.board[row][col] = False  # Opponent's move
                        print(f"Object detected in cell ({row},{col}).")

        # Draw the grid and current game state
        for row in range(rows):
            for col in range(columns):
                x1 = grid_x_start + col * cell_width
                x2 = x1 + cell_width
                y1 = grid_y_start + row * cell_height
                y2 = y1 + cell_height

                # Draw grid on the original frame with thicker lines
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), grid_line_thickness)

                # Draw "X" or "O" based on the game board
                if self.ttt.board[row][col] == True:
                    cv2.line(frame, (x1, y1), (x2, y2), (0, 0, 255), 4)
                    cv2.line(frame, (x2, y1), (x1, y2), (0, 0, 255), 4)
                elif self.ttt.board[row][col] == False:
                    cv2.circle(frame, ((x1 + x2) // 2, (y1 + y2) // 2), cell_width // 4, (255, 0, 0), 4)

        # Display the binary and greyscale feeds
        #cv2.imshow("Binary Image with Annotations", annotated_binary)
        #cv2.imshow("Greyscale Feed", gray)
        cv2.imshow("Tic Tac Toe", frame)
        cv2.waitKey()

    def close(self):
        self.videoCapture.release()
        cv2.destroyAllWindows()