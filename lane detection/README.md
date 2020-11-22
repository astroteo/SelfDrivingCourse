#**Main credits to Udacity** 
#Authors: Ignacio Sañudo (2016)** 
---

**Finding Lane Lines on the Road**

The goal of this project is make a pipeline that finds lane lines on the road


### Reflection

###1. Describe your pipeline. As part of the description, explain how you modified the draw_lines() function.

My pipeline consisted of 5 steps. 

1) Firstly, I converted the image into grayscale and then applied a smoothing method in order to reduce image noise 

2) In the second step, I applied Canny in order to perform the edge detection 

3) Then a region of interest is characterized aiming to define the region of the image under consideration 

4) In this step I put in practice the hough transform, the values used in the function were defined based on: guess, previous practice and output. (Is there any way to understand which are the best fit-values?).
In order to find and draw a single line on the left and right lanes, I modified the draw_lines() function using the following algorithm:

    4.1) In the first step I separated the lines into right and left lines based on the slope
    
    4.2) Then the hough space coordinates, representing line segments, are averaged and the lines are extrapolated to the top and bottom of the lane. The top point choice is based on heuristic (image_size/2) which is based on the camera perspective.
    
5) Finally the lines "detected" are overlaped in the original image
 
These are some of my results!

![alt text][image1]
![alt text][image2]
![alt text][image3]

###2. Identify potential shortcomings with your current pipeline

One potential shortcoming would be the fact that the algorithm is not tolerant to roads with curves and slopes, but this is one feature
that I would like to improve in the next versions (maybe approaching the challenge?).

Another shortcoming could be how the algorithm is written, due to the lack of experience on python+opencv in the automotive domain (for example variable naming or function naming...). I would like to review and improve the implementation when I will be more confident with opencv+python.

###3. Suggest possible improvements to your pipeline

This is my first version of the lane detection algorithm. I would like to improve the algorithm with the following things:

1) I would like to explore the strategies to perform the lane detection with signals on the road. Maybe modifying the region of interest?

2) Improve the algorithm with slope+curve roads support

