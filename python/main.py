import time
import numpy as np
from dynamixelArm import RobotArm 
from tictactoeAI import tttAI

def main():
    arm = RobotArm(device_name="/dev/ttyACM0",end_effector="angled")
    frame_no = 5
    tttR = tttAI(topleft=[0.025,0.075,0],mark="X",)
    tttR.drawBoard()

    arm.move_to_angles([0,np.deg2rad(30),np.deg2rad(-60),0])

    while tttR.checkGameState() == "Game continues":
        try:
        
            print("Opponent Plays:")
            row = input("Row: ")
            col = input("Column: ")
            while row not in ["0","1","2"]:
                print("Wrong Row input, try again!")
                row = input("Row: ")
            while col not in ["0","1","2"]:
                print("Wrong column input, try again!")
                col = input("Col: ")

            tttR.board[int(float(row))][int(float(col))] = False
            tttR.drawBoard()

            # we call "find best move", based on the current board
            bestMove = tttR.findBestMove()
            #tttR.board[bestMove[0]][bestMove[1]] = True
            tttR.drawBoard()
            print("The Optimal Move is :") 
            print("ROW:", bestMove[0], " COL:", bestMove[1]) # (0,0) is top-left cell of grid
            # translate "the best move" into global coordinates for next point
            nextpoint = tttR.translateMove(bestMove)
            print("The next point is: ", nextpoint)

           
            if tttR.mark == "X":
                thread, DONE = arm.patterns.cross(center=nextpoint,frame_no=frame_no)
            if tttR.mark == "O":
                thread, DONE = arm.patterns.circle(center=nextpoint,frame_no=frame_no)
            while(not DONE.is_set()):
                arm.twin.draw_arm(draw_jointSpace=False)
                time.sleep(0.005)

        
        except KeyboardInterrupt:
            print('Interrupted')
            break
        
            

    print(tttR.checkGameState())
    arm.close()    
            

if __name__ == "__main__":
    main()
    