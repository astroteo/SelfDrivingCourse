import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np
import cv2
import os

def grayscale(img):
    """Applies the Grayscale transform
    This will return an image with only one color channel
    but NOTE: to see the returned image as grayscale
    (assuming your grayscaled image is called 'gray')
    you should call plt.imshow(gray, cmap='gray')"""
    return #TODO call openCV function grayscale
    
def canny(img, low_threshold, high_threshold):
    """Applies the Canny transform"""
    return #TODO

def gaussian_blur(img, kernel_size):
    """Applies a Gaussian Noise kernel"""
    return #TODO

def region_of_interest(img, vertices):
    """
    Applies an image mask.
    
    Only keeps the region of the image defined by the polygon
    formed from `vertices`. The rest of the image is set to black.
    """
    #defining a blank mask to start with
    mask = np.zeros_like(img)   
    
    #defining a 3 channel or 1 channel color to fill the mask with depending on the input image
    if len(img.shape) > 2:
        channel_count = img.shape[2]  # i.e. 3 or 4 depending on your image
        ignore_mask_color = (255,) * channel_count
    else:
        ignore_mask_color = 255
        
    #filling pixels inside the polygon defined by "vertices" with the fill color    
    cv2.fillPoly(mask, vertices, ignore_mask_color)
    
    #returning the image only where mask pixels are nonzero
    masked_image = cv2.bitwise_and(img, mask)
    return masked_image

def drawLine(img, x, y, color=[255, 0, 0], thickness=10):
    """
    Point interpolation
    Adjust a line to the points [`x`, `y`] and draws it on the image `img` using `color` and `thickness` for the line.
    """
    if len(x) == 0: 
        return
    
    lineParameters = np.polyfit(x, y, 1) 
    
    m = lineParameters[0]
    b = lineParameters[1]
    
    maxY = img.shape[0]
    maxX = img.shape[1]
    y1 = maxY
    x1 = int((y1 - b)/m)
    y2 = int((maxY/2)) + 60
    x2 = int((y2 - b)/m)
    cv2.line(img, (x1, y1), (x2, y2), [255, 0, 0], thickness)

def draw_lines(img, lines, color=[255, 0, 0], thickness=10):
    """
    NOTE: this is the function you might want to use as a starting point once you want to 
    average/extrapolate the line segments you detect to map out the full
    extent of the lane (going from the result shown in raw-lines-example.mp4
    to that shown in P1_example.mp4).  
    
    Think about things like separating line segments by their 
    slope ((y2-y1)/(x2-x1)) to decide which segments are part of the left
    line vs. the right line.  Then, you can average the position of each of 
    the lines and extrapolate to the top and bottom of the lane.
    
    This function draws `lines` with `color` and `thickness`.    
    Lines are drawn on the image inplace (mutates the image).
    If you want to make the lines semi-transparent, think about combining
    this function with the weighted_img() function below
    """
    
    leftPointsX = []
    leftPointsY = []
    rightPointsX = []
    rightPointsY = []

    for line in lines:
        for x1,y1,x2,y2 in line:
            m = #TODO (compute the slope)
            if m < 0:
                leftPointsX.append(x1)
                leftPointsY.append(y1)
                leftPointsX.append(x2)
                leftPointsY.append(y2)
            else:
                rightPointsX.append(x1)
                rightPointsY.append(y1)
                rightPointsX.append(x2)
                rightPointsY.append(y2)

    drawLine(img, leftPointsX, leftPointsY, color, thickness)
        
    drawLine(img, rightPointsX, rightPointsY, color, thickness)

def create_directory(directory):
    if not os.path.exists(directory):
        os.makedirs(directory)
 
def hough_lines(img, rho, theta, threshold, min_line_len, max_line_gap):
    """
    `img` should be the output of a Canny transform.
        
    Returns an image with hough lines drawn.
    """
    lines = cv2.HoughLinesP(img, rho, theta, threshold, np.array([]), minLineLength=min_line_len, maxLineGap=max_line_gap)

    line_img = np.zeros((img.shape[0], img.shape[1], 3), dtype=np.uint8)
 
    draw_lines(line_img, lines)
    return line_img
 

    
def process_image_pipeline(image):

    #1 grayscale the image
    gray = grayscale(image)

    #2 Apply Gaussian smoothing
    blur_gray = gaussian_blur(image, kernel_size = 5)

    #3 Apply Canny in order to perform the edge detection
    edges = canny(blur_gray, low_threshold = #TODO, high_threshold = #TODO)

    #4 This time we are defining a four sided polygon to mask
    imshape = image.shape
    #vertices that defines our region of interest!
    vertices = np.array([[(#TODOx,#TODOy),(#TODO, #TODO), (#TODO, #TODO), (#TODO,#TODO)]], dtype=np.int32) 
    masked_edges=region_of_interest(edges, vertices)
    

    #5 Define the Hough transform parameters (based on guess and looking which was the output :p)
    # Make a blank the same size as our image to draw on
    rho = 1 # distance resolution in pixels of the Hough grid
    theta = np.pi/180 # angular resolution in radians of the Hough grid
    threshold = #TODO     # minimum number of votes (intersections in Hough grid cell)
    min_line_length = #TODO #minimum number of pixels making up a line
    max_line_gap = #TODO    # maximum gap in pixels between connectable line segments
    line_image = np.copy(image) # creating a blank to draw lines on
    
    #6 Run Hough on edge detected image
    # Output "lines" is an array containing endpoints of detected line segments
    # Define the Hough transform parameters
    lines =  hough_lines(masked_edges, rho, theta, threshold, min_line_length, max_line_gap)
    
    # Create a "color" binary image to combine with line image
    color_edges = np.dstack((edges, edges, edges)) 

    # Draw the lines on the edge image (this function
    image_wlines = cv2.addWeighted(lines, 0.8, line_image, 1, 0) 
    return image_wlines
 


