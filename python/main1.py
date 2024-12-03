import time
import numpy as np
from dynamixelArm import RobotArm 
from tictactoeAI import tttAI
#from TakingPicnext import init
import cv2

def computervision(frame, tttAI):

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
                if tttAI.board[row][col] is None:
                    tttAI.board[row][col] = False  # Opponent's move
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
            if tttAI.board[row][col] == True:
                cv2.line(frame, (x1, y1), (x2, y2), (0, 0, 255), 4)
                cv2.line(frame, (x2, y1), (x1, y2), (0, 0, 255), 4)
            elif tttAI.board[row][col] == False:
                cv2.circle(frame, ((x1 + x2) // 2, (y1 + y2) // 2), cell_width // 4, (255, 0, 0), 4)

    # Display the binary and greyscale feeds
    #cv2.imshow("Binary Image with Annotations", annotated_binary)
    #cv2.imshow("Greyscale Feed", gray)
    cv2.imshow("Tic Tac Toe", frame)
    cv2.waitKey()


def main():
    arm = RobotArm(device_name="COM7",end_effector="angled")
    frame_no = 5
    tttR = tttAI(topleft=[0.0,0.0,0],mark="X",cellWidth=0.03)
    tttR.drawBoard()

    # Start the webcam feed
    print("Opening Camera...")
    videoCapture = cv2.VideoCapture(1)

    # Set HD resolution (1280x720)
    width = 1280  # HD width
    height = 720  # HD height
    videoCapture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    videoCapture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

    print(f"Resolution set to: {width}x{height}")
    


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
            

            # move to viewing positon
            time.sleep(5)
           
            # do computer vision
            ret, frame = videoCapture.read()
            if not ret:
                print("Error: Unable to capture frame.")
                break

            # Display the live feed
            cv2.imshow("Webcam Feed", frame)

            # Wait for the user to press 's' or 'q'
            print("Hit s to capture image")
            while True:
                ret, frame = videoCapture.read()
                if not ret:
                    print("Error: Unable to capture frame.")
                    break

                # Display the live feed
                cv2.imshow("Webcam Feed", frame)

                # Wait for the user to press 's' or 'q'
                
                key = cv2.waitKey(1) & 0xFF
                if key == ord("s"):
                    print("Image captured.")
                    break
                elif key == ord("q"):
                    print("Game terminated.")
                    videoCapture.release()
                    cv2.destroyAllWindows()
                    exit()
                
               
            print('Performing computer vision')
            computervision(frame, tttR)

            #tttR.board[int(float(row))][int(float(col))] = False
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
            nextpoint[2] += 0.025
            nextpoint[1] += 0.01
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

    videoCapture.release()
    cv2.destroyAllWindows()
        
            

    print(tttR.checkGameState())
    arm.close()    
            

if __name__ == "__main__":
    main()
    
    