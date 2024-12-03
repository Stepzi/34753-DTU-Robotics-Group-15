import numpy as np
import cv2

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
black_pixel_threshold = 350  # Threshold for number of black pixels

videoCapture = cv2.VideoCapture(1)
tictactoe = tttAI(mark="X")

while True:
    ret, image = videoCapture.read()
    if not ret:
        print("Error: Unable to capture frame")
        break

    # Convert to grayscale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Thresholding to create a binary image
    _, binary = cv2.threshold(gray, 50, 255, cv2.THRESH_BINARY_INV)  # Binary image (black = object)

    # Create a copy of the grayscale feed for visualization
    grayscale_feed = gray.copy()
    grayscale_feed = cv2.cvtColor(grayscale_feed, cv2.COLOR_GRAY2BGR)  # Convert to BGR for annotations

    # Loop through each grid cell
    for row in range(rows):
        for col in range(columns):
            x1 = grid_x_start + col * cell_width
            x2 = x1 + cell_width
            y1 = grid_y_start + row * cell_height
            y2 = y1 + cell_height

            # Crop the cell region
            cell_region = binary[y1:y2, x1:x2]

            # Count the number of black pixels
            black_pixel_count = np.sum(cell_region == 255)  # Binary INV: black = 255

            # Display the threshold and current black pixel count on the grayscale feed
            cv2.putText(
                grayscale_feed,
                f"Threshold: {black_pixel_threshold}",
                (10, 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0),
                1,
            )
            cv2.putText(
                grayscale_feed,
                f"Cell ({row},{col}): {black_pixel_count}",
                (x1 + 5, y1 + 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 255, 0),
                1,
            )

            if black_pixel_count > black_pixel_threshold:
                # Mark the cell in the Tic Tac Toe grid
                if tictactoe.board[row][col] is None:
                    tictactoe.board[row][col] = False  # Opponent's move (O)
                    print(f"Object detected in cell ({row}, {col}).")

                    # Get the AI's best move
                    bestMove = tictactoe.findBestMove()
                    if bestMove != (-1, -1):
                        tictactoe.board[bestMove[0]][bestMove[1]] = True  # Robot's move (X)
                        print(f"AI plays at ({bestMove[0]}, {bestMove[1]}).")

                # Visualize on grayscale feed: Add a red bounding box and text
                cv2.rectangle(grayscale_feed, (x1, y1), (x2, y2), (0, 0, 255), 2)
                cv2.putText(
                    grayscale_feed,
                    "Detected",
                    (x1 + 5, y1 + 40),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 0, 255),
                    1,
                )

    # Draw the Tic Tac Toe grid and board state on the main feed
    for row in range(rows):
        for col in range(columns):
            x1 = grid_x_start + col * cell_width
            x2 = x1 + cell_width
            y1 = grid_y_start + row * cell_height
            y2 = y1 + cell_height
            cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)
            if tictactoe.board[row][col] == True:
                cv2.line(image, (x1, y1), (x2, y2), (0, 0, 255), 4)
                cv2.line(image, (x2, y1), (x1, y2), (0, 0, 255), 4)
            elif tictactoe.board[row][col] == False:
                cv2.circle(image, ((x1 + x2) // 2, (y1 + y2) // 2), cell_width // 4, (255, 0, 0), 4)

    # Display the grayscale feed with annotations
    cv2.imshow("Grayscale Feed", grayscale_feed)

    # Display the main Tic Tac Toe grid
    cv2.imshow("Tic Tac Toe", image)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

videoCapture.release()
cv2.destroyAllWindows()
