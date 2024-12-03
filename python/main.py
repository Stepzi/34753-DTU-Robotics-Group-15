import time
import numpy as np
from dynamixelArm import RobotArm 
from tictactoeAI import tttAI

def main():
    arm = RobotArm(device_name="/dev/ttyACM0",end_effector="angled")
    frame_no = 5
    tttR = tttAI(topleft=[0.05,0.075,0],mark="O",)
    tttR.drawBoard()


    while tttR.checkGameState() == "Game continues":
        try:

            wait = arm.joint_polyTraj(frame_no=4, 
                          A={'gamma': None, 'origin': None, 'elbow':"up", 'v': [0,0,0], 'gamma_d': 0},
                          B={'gamma': -np.deg2rad(30), 'origin': [0,0.1,0.2], 'elbow':"up", 'v': [0,0,0], 'gamma_d': 0},
                          tA = 0,
                          tB = 3,
                          order = 3)
            
            thread, DONE = arm.run_in_thread(arm.follow_traj,[wait],Ts=0.1)
            while(not DONE.is_set()):
                arm.twin.draw_arm(draw_jointSpace=False)
                time.sleep(0.005)

           
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
    