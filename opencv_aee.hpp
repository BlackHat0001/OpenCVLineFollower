///////////////////////////////////////////////////////////////
//                                                           // 
// H61AEE: Dept. EEE, The University of Nottingham 2018      //
// Author: D. Fallows                                        //
//                                                           //
// opencv_aee.hpp: Library file to simplify OpenCV functions //
//                 Session 6/7 - Computer Vision             // 
//                                                           // 
///////////////////////////////////////////////////////////////

#ifndef OPENCV_AEE_HPP_INCLUDED
#define OPENCV_AEE_HPP_INCLUDED

#define FEATURE_MATCH_DRAW 0    // Change to 1 to draw the matches to an image when using the feature matcher

#include <stdio.h>
#include <iostream>
#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/imgproc.hpp"

using namespace cv;
using namespace cv::xfeatures2d;

// Functions
void setupCamera(int width, int height);       // Setup the camera for image capture
void resizeCamera(int width, int height);      // Resize the camera frame
Mat captureFrame(void);                         // Capture an image from the camera
Mat readImage(const char* imageLoc);           // Read an image from a file

Point templateMatch(Mat frame, Mat templ, int method, double threshold = 0);  // Locate an image inside another image and give the centre location
Point featureMatch(Mat frame, Mat object, int minHessian = 400, float scalingFactor = 3, int goodMatchLimit = 0);  // Locate an image inside another image and give the centre location. Uses feature matching to overcome angle/lighting issues
Point findContourCentre(std::vector<cv::Point> contour);    // Calculate the moment of the contour and use it to calculate the centre point
Mat transformPerspective(std::vector<Point> boundingContour, Mat frame, int x_size, int y_size);   // Transform an image so that corners fit those of a given contour
float compareImages(Mat cameraImage, Mat librarySymbol);    // Compare two images and return a percentage value related to the quality of match

void errorTrap(void);       // Lock up the program if an error occurs
void closeCV(void);     // Close the camera and clean up any open windows

#endif // OPENCV_AEE_HPP_INCLUDED
