#################################################################
## "Tic Tac Toe AI" - finds the next optimal move for a player ##
#################################################################
"""
The algorithm assumes that after it's move, the opponent will make the smartest possible move.
Therefore, 
"""
# the algorithm assumes that the opponent makes the smartes possible moves, 

# based on:
# https://www.geeksforgeeks.org/finding-optimal-move-in-tic-tac-toe-using-minimax-algorithm-in-game-theory/

class tttAI():
	def __init__(self, cellWidth = 0.05, topleft = [0.0,0.0,0.0]):
		# adjust the cell width to the grid cell width/height in meters
		self.board = [ 
			[ None, None, None ], 
			[ None, None, None ], 
			[ None, None, None ] 
		]
		self.cellWidth = cellWidth
		self.topleft = topleft


	def isMovesLeft(self, board) : 
		# returns true if there are moves remaining on the board
		# false if no moves left to play
		for i in range(3) : 
			for j in range(3) : 
				if (board[i][j] == None) : 
					return True
		return False

	def evaluate(self, b) : 
		# evaluation function, minimax algorithm: ( http://goo.gl/sJgv68 ) 

		# Checking for Rows for X or O victory. 
		for row in range(3) :	 
			if (b[row][0] == b[row][1] and b[row][1] == b[row][2]) :		 
				if (b[row][0] == True) : 
					return 10
				elif (b[row][0] == False) : 
					return -10

		# Checking for Columns for X or O victory. 
		for col in range(3) : 
		
			if (b[0][col] == b[1][col] and b[1][col] == b[2][col]) : 
			
				if (b[0][col] == True) : 
					return 10
				elif (b[0][col] == False) : 
					return -10

		# Checking for Diagonals for X or O victory. 
		if (b[0][0] == b[1][1] and b[1][1] == b[2][2]) : 
		
			if (b[0][0] == True) : 
				return 10
			elif (b[0][0] == False) : 
				return -10

		if (b[0][2] == b[1][1] and b[1][1] == b[2][0]) : 
		
			if (b[0][2] == True) : 
				return 10
			elif (b[0][2] == False) : 
				return -10

		# Else if none of them have won then return 0 
		return 0

	def minimax(self, board, depth, isMax) : 
		# considers all possible ways the game can go, and returns the value of board
		
		score = self.evaluate(board) 

		# If Maximizer has won the game return his/her 
		# evaluated score 
		if (score == 10) : 
			return score 

		# If Minimizer has won the game return his/her 
		# evaluated score 
		if (score == -10) : 
			return score 

		# If there are no more moves and no winner then 
		# it is a tie 
		if (self.isMovesLeft(board) == False) : 
			return 0

		# If this maximizer's move 
		if (isMax) :	 
			best = -1000

			# Traverse all cells 
			for i in range(3) :		 
				for j in range(3) : 
				
					# Check if cell is empty 
					if (board[i][j]==None) : 
					
						# Make the move 
						board[i][j] = True 

						# Call minimax recursively and choose 
						# the maximum value 
						best = max( best, self.minimax(board, 
												depth + 1, 
												not isMax) ) 

						# Undo the move 
						board[i][j] = None
			return best 

		# If this minimizer's move 
		else : 
			best = 1000

			# Traverse all cells 
			for i in range(3) :		 
				for j in range(3) : 
				
					# Check if cell is empty 
					if (board[i][j] == None) : 
					
						# Make the move 
						board[i][j] = False

						# Call minimax recursively and choose 
						# the minimum value 
						best = min(best, self.minimax(board, depth + 1, not isMax)) 

						# Undo the move 
						board[i][j] = None
			return best 

	def findBestMove(self) : 
		# returns the best possible move for the player (in this case the robot)
		bestVal = -1000
		bestMove = (-1, -1) 

		# Traverse all cells, evaluate minimax function for 
		# all empty cells. And return the cell with optimal 
		# value. 
		for i in range(3) :	 
			for j in range(3) : 
			
				# Check if cell is empty 
				if (self.board[i][j] == None) : 
				
					# Make the move 
					self.board[i][j] = True 

					# compute evaluation function for this 
					# move. 
					moveVal = self.minimax(self.board, 0, False) 

					# Undo the move 
					self.board[i][j] = None

					# If the value of the current move is 
					# more than the best value, then update 
					# best/ 
					if (moveVal > bestVal) :				 
						bestMove = (i, j) 
						bestVal = moveVal 

		
		#print("The value of the best Move is :", bestVal) 
		#print() 
		"""
		If bestVal == +10, the computer has a winning move.
		If bestVal == -10, the opponent has a winning move.
		If no moves are left and evaluate is 0, the game is a draw.
		"""
		if bestVal == 10:
			print("Robot has a winning move!")
		elif bestVal == -10:
			print("Opponent has a winning move!")
		return bestMove 
	
	def translateMove(self, move):
		"""
    	translate next move from (row,col) to (x,y,z) coordinates.
		rows on negative y axis, col on x axis

    	:param move: (row,col) of next move
    	:return: returns 3d coordinates [x,y,z]
    	""" 
		
		x = self.topleft[0] + self.cellWidth*(move[1]) + 0.5*self.cellWidth
		y = self.topleft[1] - self.cellWidth*(move[0]) - 0.5*self.cellWidth
		z = self.topleft[2]
		point3d = [x, y, z]

		return point3d
	
	def checkGameState(self):
		"""
    	This function checks the current state of the game.
    	Returns "Robot wins", "Opponent wins", "Draw", or "Game continues"
    	""" 

		score = self.evaluate(self.board)

		# Check if either player has won
		if score == 10:
			return "Robot wins"
		elif score == -10:
			return "Opponent wins"

		# Check for a draw (no moves left)
		if not any(None in row for row in self.board):  # No empty cells (None)
			return "Draw"

		return "Game continues"


if __name__ == "__main__":
	# create tictactoe object:
	tictacrobot = tttAI() # board always starts as empty

	# the board is now updated using our advanced computer vision:
	print("**Simulated update of the board**")
	tictacrobot.board = [ 
		[ True, False, True ], 
		[ False, False, True ], 
		[ None, None, None ] 
	] 

	# check state of game before you find best move!
	# If result is not "Game continues", the game is over and should not proceed.
	result = tictacrobot.checkGameState()
	if result != "Game continues":
		print(result)

	# we call "find best move", based on the current board
	bestMove = tictacrobot.findBestMove() 
	print("The Optimal Move is :") 
	print("ROW:", bestMove[0], " COL:", bestMove[1]) # (0,0) is top-left cell of grid
	# translate "the best move" into global coordinates for next point
	nextpoint = tictacrobot.translateMove(bestMove)
	print("The next point is: ", nextpoint)

	# after "nextpoint" has been calculated, we send it to the inverse kinematic function...

	# we use the suggested move:
	print("**Simulated update of the board**")
	tictacrobot.board = [ 
		[ True, False, True ], 
		[ False, False, True ], 
		[ None, None, True ] 
	] 
	# and check the state:
	print(tictacrobot.checkGameState())





