import time
import numpy as np
from dynamixelArm import RobotArm 
from tictactoeAI import tttAI

def main():
    arm = RobotArm(device_name="/dev/ttyACM0",end_effector="straight")
    botA = tttAI(topleft=[0.025,0.075,0],mark="X")
    botB = tttAI(topleft=[0.025,0.075,0],mark="O")

    player = [botA,botB]
    i = 0

    while botA.checkGameState() == "Game continues":
        try:
            print(f"Player {i+1} play with {player[i].mark}")
            i_next = (i+1)%2
            # we call "find best move", based on the current board
            bestMove = player[i].findBestMove()
            player[i].board[bestMove[0]][bestMove[1]] = True
            player[i_next].board[bestMove[0]][bestMove[1]] = False
            player[i].drawBoard()
            print("The Optimal Move is :") 
            print("ROW:", bestMove[0], " COL:", bestMove[1]) # (0,0) is top-left cell of grid
            nextpoint = player[i].translateMove(bestMove)
            print("The next point is: ", nextpoint)
           
            if player[i].mark == "X":
                thread, DONE = arm.patterns.cross(center=nextpoint,frame_no=4,gamma=-np.deg2rad(60))
            if player[i].mark == "O":
                thread, DONE = arm.patterns.circle(center=nextpoint,frame_no=4,radius=0.02,gamma=-np.deg2rad(60))
            while(not DONE.is_set()):
                arm.twin.draw_arm(draw_jointSpace=False)
                time.sleep(0.005)
            
            i = i_next
            input("Hit Enter for next move...")
                

        
        except KeyboardInterrupt:
            print('Interrupted')
            break
        
            

    print(botA.checkGameState())
    arm.close()    
            

if __name__ == "__main__":
    main()
    
