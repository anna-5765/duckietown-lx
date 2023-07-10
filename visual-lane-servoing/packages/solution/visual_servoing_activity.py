from typing import Tuple

import numpy as np
import cv2

def get_steer_matrix_left_lane_markings(shape: Tuple[int, int]) -> np.ndarray:
    """
    Args:
        shape:              The shape of the steer matrix.

    Return:
        steer_matrix_left:  The steering (angular rate) matrix for Braitenberg-like control
                            using the masked left lane markings (numpy.ndarray)
    """

    #print(shape) (480, 512)

    # create left weighted matrix
    
    steer_matrix_left = np.zeros(shape=shape, dtype="float32")
    steer_matrix_left[380:480, 128:256] = -0.75
    steer_matrix_left[440:480, 64:128] = -0.5
    
    return steer_matrix_left

def get_steer_matrix_right_lane_markings(shape: Tuple[int, int]) -> np.ndarray:
    """
    Args:
        shape:               The shape of the steer matrix.

    Return:
        steer_matrix_right:  The steering (angular rate) matrix for Braitenberg-like control
                             using the masked right lane markings (numpy.ndarray)
    """

    #print(shape) (480, 512)

    # create right weighted matrix

    steer_matrix_right = np.zeros(shape=shape, dtype="float32")
    steer_matrix_right[300:480, 256:384] = 0.75
    steer_matrix_right[360:480, 384:475] = 0.5

    return steer_matrix_right

def detect_lane_markings(image: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """
    Args:
        image: An image from the robot's camera in the BGR color space (numpy.ndarray)
    Return:
        left_masked_img:   Masked image for the dashed-yellow line (numpy.ndarray)
        right_masked_img:  Masked image for the solid-white line (numpy.ndarray)
    """
    # IMAGE PREP
    # convert image to hsv for color-based filtering
    img_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # create grayscale
    img_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # find image-to-ground homography (skipping)
    # (not masking out the horizon as per instructions the robot and simulation will do this automatically)
    # 
    # blur/smooth image using Gaussian kernel
    # standard deviation (sigma) found to remove noise while not eliminating too much valid content
    sigma = 4
    img_gaussian_filter = cv2.GaussianBlur(img_gray, (0,0), sigma)
    
    # detect lane markings using sobel edge detection
    #  convolve image with sobel operator
    sobelx = cv2.Sobel(img_gaussian_filter, cv2.CV_64F, 1, 0)
    sobely = cv2.Sobel(img_gaussian_filter, cv2.CV_64F, 0, 1)

    #  compute mag of gradients
    Gmag = np.sqrt(sobelx*sobelx + sobely*sobely)

    #  compute orientation of gradients
    Gdir = cv2.phase(np.array(sobelx, np.float32), np.array(sobely, np.float32), angleInDegrees=True)

    # find threshold for gradient magnitude, filter out weaker edges
    threshold = 40 #use test image to set
    mask_mag = (Gmag > threshold)

    # Color based-masking
    white_lower_hsv = np.array([0, 0, 90]) #gray hue in simulation
    white_upper_hsv = np.array([120, 50, 255])
    yellow_lower_hsv = np.array([15,50, 50])
    yellow_upper_hsv = np.array([80, 255, 255]) #simulation has green hue

    mask_white = cv2.inRange(img_hsv, white_lower_hsv, white_upper_hsv)
    mask_yellow = cv2.inRange(img_hsv, yellow_lower_hsv, yellow_upper_hsv)

    # Left/right edge based masking, may need to comment out depending on usefulness
    width = image.shape[1]
    mask_left = np.ones(sobelx.shape)
    mask_left[:,int(np.floor(width/2)):width+1] = 0
    mask_right = np.ones(sobelx.shape)
    mask_right[:,0:int(np.floor(width/2))] = 0


    #  mask left and right half of image
    #  mask edges based on derivatives
    mask_sobelx_pos = (sobelx > 0)
    mask_sobelx_neg = (sobelx < 0)
    mask_sobely_pos = (sobely > 0)
    mask_sobely_neg = (sobely < 0)


    # TODO: implement your own solution here
    shape = image.shape
    mask_left_edge = np.zeros(shape=shape, dtype="float32")
    mask_right_edge = np.zeros(shape=shape, dtype="float32")

    mask_left_edge =  mask_left * mask_mag * mask_sobelx_neg * mask_sobely_neg * mask_yellow
    mask_right_edge = mask_right * mask_mag * mask_sobelx_pos * mask_sobely_neg * mask_white

    return mask_left_edge, mask_right_edge
