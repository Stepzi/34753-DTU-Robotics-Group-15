import numpy as np
import cv2
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

# Initialize grid parameters
grid_x_start = 50
grid_y_start = 50
grid_width = 400
grid_height = 400
rows = 3
columns = 3
cell_width = grid_width // columns
cell_height = grid_height // rows

videoCapture = cv2.VideoCapture(0)
tictactoe = tttAI(mark="X")

def get_cell(x, y):
    if grid_x_start <= x < grid_x_start + grid_width and grid_y_start <= y < grid_y_start + grid_height:
        col = (x - grid_x_start) // cell_width
        row = (y - grid_y_start) // cell_height
        return int(row), int(col)
    return None

while True:
    ret, image = videoCapture.read()
    if not ret:
        print("Error: Unable to capture frame")
        break

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    gray = cv2.blur(gray, (5, 5))

    circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1.4, 30)

    if circles is not None:
        circles = np.round(circles[0, :]).astype("int")
        for (x, y, r) in circles:
            cell = get_cell(x, y)
            if cell is not None:
                row, col = cell
                if tictactoe.board[row][col] is None:
                    tictactoe.board[row][col] = False  # Opponent's move (O)
                    print(f"Circle detected at ({row}, {col}).")

                    # Get the AI's best move
                    bestMove = tictactoe.findBestMove()
                    if bestMove != (-1, -1):
                        tictactoe.board[bestMove[0]][bestMove[1]] = True  # Robot's move (X)
                        print(f"AI plays at ({bestMove[0]}, {bestMove[1]}).")

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

    cv2.imshow("Tic Tac Toe", image)
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

videoCapture.release()
cv2.destroyAllWindows()
 