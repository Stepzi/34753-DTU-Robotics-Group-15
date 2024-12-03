import cv2
import numpy as np
import time


class tttAI:
    def __init__(self, mark, cellWidth=0.05, topleft=[0.0, 0.0, 0.0]):
        self.board = [[None, None, None], [None, None, None], [None, None, None]]
        self.cellWidth = cellWidth
        self.topleft = topleft
        self.mark = mark

    def isMovesLeft(self, board):
        for i in range(3):
            for j in range(3):
                if board[i][j] == None:
                    return True
        return False

    def evaluate(self, b):
        for row in range(3):
            if b[row][0] == b[row][1] == b[row][2]:
                if b[row][0] == True:
                    return 10
                elif b[row][0] == False:
                    return -10
        for col in range(3):
            if b[0][col] == b[1][col] == b[2][col]:
                if b[0][col] == True:
                    return 10
                elif b[0][col] == False:
                    return -10
        if b[0][0] == b[1][1] == b[2][2]:
            if b[0][0] == True:
                return 10
            elif b[0][0] == False:
                return -10
        if b[0][2] == b[1][1] == b[2][0]:
            if b[0][2] == True:
                return 10
            elif b[0][2] == False:
                return -10
        return 0

    def minimax(self, board, depth, isMax):
        score = self.evaluate(board)
        if score == 10:
            return score
        if score == -10:
            return score
        if not self.isMovesLeft(board):
            return 0
        if isMax:
            best = -1000
            for i in range(3):
                for j in range(3):
                    if board[i][j] == None:
                        board[i][j] = True
                        best = max(best, self.minimax(board, depth + 1, not isMax))
                        board[i][j] = None
            return best
        else:
            best = 1000
            for i in range(3):
                for j in range(3):
                    if board[i][j] == None:
                        board[i][j] = False
                        best = min(best, self.minimax(board, depth + 1, not isMax))
                        board[i][j] = None
            return best

    def findBestMove(self):
        bestVal = -1000
        bestMove = (-1, -1)
        for i in range(3):
            for j in range(3):
                if self.board[i][j] == None:
                    self.board[i][j] = True
                    moveVal = self.minimax(self.board, 0, False)
                    self.board[i][j] = None
                    if moveVal > bestVal:
                        bestMove = (i, j)
                        bestVal = moveVal
        return bestMove


# Grid parameters
grid_x_start = 50
grid_y_start = 50
grid_width = 400
grid_height = 400
rows = 3
columns = 3
cell_width = grid_width // columns
cell_height = grid_height // rows
grid_line_thickness = 10  # Thickness of the grid lines
black_pixel_threshold = 500  # Threshold for number of black pixels
buffer_time = 5  # Time in seconds to wait after threshold is crossed

# Initialize the webcam and AI
videoCapture = cv2.VideoCapture(1)
tictactoe = tttAI(mark="X")

# Dictionary to track the time when a threshold was crossed for each cell
cell_timers = {}

while True:
    ret, frame = videoCapture.read()
    if not ret:
        print("Error: Unable to capture frame.")
        break

    # Convert to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Apply binary thresholding
    _, binary = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)

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
    binary = cv2.bitwise_and(binary, grid_mask)

    # Create a copy of the binary image for annotations
    annotated_binary = cv2.cvtColor(binary, cv2.COLOR_GRAY2BGR)

    # Loop through each grid cell
    current_time = time.time()
    for row in range(rows):
        for col in range(columns):
            x1 = grid_x_start + col * cell_width
            x2 = x1 + cell_width
            y1 = grid_y_start + row * cell_height
            y2 = y1 + cell_height

            # Crop the binary image region corresponding to the cell
            cell_region = binary[y1:y2, x1:x2]

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
                if tictactoe.board[row][col] is None:
                    # Check if we need to start the timer for this cell
                    if (row, col) not in cell_timers:
                        cell_timers[(row, col)] = current_time
                    elif current_time - cell_timers[(row, col)] >= buffer_time:
                        # Add the opponent's move (O) after the buffer time
                        tictactoe.board[row][col] = False
                        print(f"Object detected in cell ({row},{col}).")

                        # Get the AI's best move
                        bestMove = tictactoe.findBestMove()
                        if bestMove != (-1, -1):
                            tictactoe.board[bestMove[0]][bestMove[1]] = True  # Robot's move (X)
                            print(f"AI plays at ({bestMove[0]},{bestMove[1]}).")
                        cell_timers.pop((row, col))  # Reset the timer for this cell
            else:
                # Reset the timer if the cell goes below the threshold
                if (row, col) in cell_timers:
                    cell_timers.pop((row, col))

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
            if tictactoe.board[row][col] == True:
                cv2.line(frame, (x1, y1), (x2, y2), (0, 0, 255), 4)
                cv2.line(frame, (x2, y1), (x1, y2), (0, 0, 255), 4)
            elif tictactoe.board[row][col] == False:
                cv2.circle(frame, ((x1 + x2) // 2, (y1 + y2) // 2), cell_width // 4, (255, 0, 0), 4)

    # Display the binary and annotated game feeds
    cv2.imshow("Binary Image with Annotations", annotated_binary)
    cv2.imshow("Tic Tac Toe", frame)

    # Exit the loop when 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

# Release resources
videoCapture.release()
cv2.destroyAllWindows()
 