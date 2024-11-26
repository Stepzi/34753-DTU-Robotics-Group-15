import cv2
import numpy as np
import glob

class CameraCalibrator:
    def __init__(self, chessboard_size=(9, 6), square_size=1.0):
        """
        Initialize the CameraCalibrator class.
        :param chessboard_size: Tuple representing the number of inner corners in the chessboard pattern (columns, rows).
        :param square_size: Size of each square in the chessboard (arbitrary units).
        """
        self.chessboard_size = chessboard_size
        self.square_size = square_size
        self.obj_points = []  # 3D points in the real world
        self.img_points = []  # 2D points in the image
        self.camera_matrix = None
        self.dist_coeffs = None
        self.rvecs = None
        self.tvecs = None

        # Prepare object points for the chessboard pattern
        self.objp = np.zeros((chessboard_size[0] * chessboard_size[1], 3), np.float32)
        self.objp[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2) * square_size

    def collect_calibration_points(self, image_files):
        """
        Detect chessboard corners in the provided images and collect points for calibration.
        :param image_files: List of file paths to calibration images.
        """
        for file in image_files:
            image = cv2.imread(file)
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

            # Find the chessboard corners
            ret, corners = cv2.findChessboardCorners(gray, self.chessboard_size, None)

            if ret:
                self.obj_points.append(self.objp)
                self.img_points.append(corners)

                # Draw and display the corners for visualization (optional)
                cv2.drawChessboardCorners(image, self.chessboard_size, corners, ret)
                cv2.imshow('Chessboard', image)
                cv2.waitKey(500)
            else:
                print(f"Chessboard corners not found in {file}")

        cv2.destroyAllWindows()

    def calibrate(self, image_size):
        """
        Perform camera calibration using the collected points.
        :param image_size: Tuple (width, height) of the images used for calibration.
        :return: Calibration results as a dictionary.
        """
        ret, self.camera_matrix, self.dist_coeffs, self.rvecs, self.tvecs = cv2.calibrateCamera(
            self.obj_points, self.img_points, image_size, None, None
        )

        return {
            "ret": ret,
            "camera_matrix": self.camera_matrix,
            "dist_coeffs": self.dist_coeffs,
            "rvecs": self.rvecs,
            "tvecs": self.tvecs
        }

    def undistort(self, image):
        """
        Undistort an image using the calculated camera matrix and distortion coefficients.
        :param image: Input distorted image.
        :return: Undistorted image.
        """
        if self.camera_matrix is None or self.dist_coeffs is None:
            raise ValueError("Camera is not calibrated. Perform calibration first.")

        h, w = image.shape[:2]
        new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(
            self.camera_matrix, self.dist_coeffs, (w, h), 1, (w, h)
        )
        undistorted = cv2.undistort(image, self.camera_matrix, self.dist_coeffs, None, new_camera_matrix)

        # Crop the image to the region of interest
        x, y, w, h = roi
        return undistorted[y:y + h, x:x + w]

# Example usage
if __name__ == "__main__":
    calibrator = CameraCalibrator(chessboard_size=(9, 6), square_size=0.025)
    
    # Provide a path to your calibration images
    image_files = glob.glob('calib/*.jpg')  # Replace with your path
    calibrator.collect_calibration_points(image_files)

    # Assuming images are of size 1920x1080
    calibration_results = calibrator.calibrate(image_size=(1280, 720))
    print("Camera Matrix:\n", calibration_results["camera_matrix"])
    print("Distortion Coefficients:\n", calibration_results["dist_coeffs"])

    # Undistort a sample image
    sample_image = cv2.imread('calib/sample/sample.jpg')  # Replace with your path
    undistorted_image = calibrator.undistort(sample_image)
    cv2.imshow("Undistorted Image", undistorted_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
