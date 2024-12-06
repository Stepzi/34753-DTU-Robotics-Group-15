import time
import numpy as np
from inc.dynamixelArm import RobotArm 
from inc.tictactoeAI import tttAI
from inc.computerVision import ComputerVision


def main():
    arm = RobotArm(device_name="COM7",end_effector="angled")
    frame_no = 5
    tttR = tttAI(topleft=[0.0,0.0,0],mark="X",cellWidth=0.03)
    CV = ComputerVision(tttR)
    tttR.drawBoard()


    while tttR.checkGameState() == "Game continues":
        try:
            
            wait = arm.joint_polyTraj(frame_no=5, 
                          A={'gamma': None, 'origin': None, 'elbow':"up", 'v': [0,0,0], 'gamma_d': 0},
                          B={'gamma': -np.deg2rad(90), 'origin': [0.045,-0.045,0.1], 'elbow':"up", 'v': [0,0,0], 'gamma_d': 0},
                          tA = 0,
                          tB = 3,
                          order = 3)
            
            thread, DONE = arm.run_in_thread(arm.follow_traj,[wait],Ts=0.1,frame_no=5)
            while(not DONE.is_set()):
                arm.twin.draw_arm(draw_jointSpace=False)
                time.sleep(0.005)
            

                  

            CV.captureImage()

            print('The board has been updated with opponent move:')
            tttR.drawBoard()
            if tttR.checkGameState() != "Game continues":
                break
            # we call "find best move", based on the current board
            print('robot is calculating its move')
            bestMove = tttR.findBestMove()
            print("The Optimal Move is :") 
            print("ROW:", bestMove[0], " COL:", bestMove[1]) # (0,0) is top-left cell of grid
            print('The updated board is')
            tttR.drawBoard()
            # translate "the best move" into global coordinates for next point
            nextpoint = tttR.translateMove(bestMove)
            print("The next point for inverse kin is: ", nextpoint)
            print('Moving the robot arm...')
           
            if tttR.mark == "X":
                thread, DONE = arm.patterns.cross(center=nextpoint,frame_no=frame_no,size=0.02)
            if tttR.mark == "O":
                thread, DONE = arm.patterns.circle(center=nextpoint,frame_no=frame_no,radius=0.01)
            while(not DONE.is_set()):
                arm.twin.draw_arm(draw_jointSpace=False)
                time.sleep(0.005)

        
        except KeyboardInterrupt:
            print('Interrupted')
            break

    CV.close()
        
            

    print(tttR.checkGameState())
    arm.close()    
            

if __name__ == "__main__":
    main()
    
    