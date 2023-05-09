/*
    File: main.cpp
	
    Author: Daniel Marcovecchio 
    Author URI: https://github.com/BlackHat0001

    Description: Symbol Recognition and Optical line following for the Raspberry Pi
                 onboard the EEEBot

    Version: 1.1.0 Release
*/

// Include files for required libraries
#include <stdio.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <cstddef>

#include "opencv_aee.hpp"
#include "main.hpp"     // You can use this file for declaring defined values and functions
#include "pi2c.h"

Pi2c car(4); // Configure the I2C interface to the Car as a global variable

float weightedAverage = 0.0f;

Mat currentCapture;

void setup(void)
{
    setupCamera(320, 240);  // Enable the camera for OpenCV


}



int LookForSymbol(int hsvArr[])
{

    Mat frame;

    frame = currentCapture;

    cv::rotate(frame, frame, cv::ROTATE_180);

    Mat gray;
    Mat finalImage;
    Mat object;
    cv::cvtColor(frame, object, COLOR_BGR2HSV);
    cv::inRange(object, cv::Scalar(hsvArr[0], hsvArr[2], hsvArr[4]), cv::Scalar(hsvArr[1], hsvArr[3], hsvArr[5]), finalImage);


    erode(finalImage, finalImage, cv::getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
    dilate(finalImage, finalImage, cv::getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
    dilate(finalImage, finalImage, cv::getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
    erode(finalImage, finalImage, cv::getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));


    Mat contourOut = finalImage.clone();
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(contourOut, contours, RETR_LIST, CHAIN_APPROX_SIMPLE);

    cv::imshow("SymbolTransformed", finalImage);
    cv::moveWindow("SymbolTransformed", 400, 400);

    //Find all quads in the image
    std::vector<std::vector<cv::Point>> quads;
    cv::Mat new_final = frame.clone();
    std::vector<cv::Point> point;
    for(int i = 0; i < contours.size(); i++)
    {

        cv::approxPolyDP(contours[i], point, cv::arcLength(contours[i], true) * 0.02, true);
        if(point.size() == 4 && cv::isContourConvex(point))
        {

            //Credit opencv docs
            double maxCosine = 0;

            for(int j=2; j<5; j++)
            {
                double cosine = std::fabs(angle(point[j%4], point[j-2], point[j-1]));
                maxCosine = MAX(maxCosine, cosine);
            }

            if(maxCosine < 0.3)
            {
                quads.push_back(point);
                cv::drawContours(new_final, quads, -1, cv::Scalar(0, 255, 255), 3);
            }


        }
    }
    cv::imshow("Quads", new_final);
    cv::moveWindow("Quads", 700, 400);

    //Filter the largest quad
    float maxArea = 0;
    int largeContour = 0;
    for(int i = 0; i < quads.size(); i++)
    {
        float area = (float)cv::contourArea(quads[i]);
        if(area > maxArea)
        {
            largeContour = i;
            maxArea = area;
        }
    }

    cv::Mat mask(frame.size(), frame.type(), cv::Scalar(0));

    cv::drawContours(mask, quads, largeContour, Scalar(255, 255, 255), FILLED);

    cv::Mat symbolRaw;

    frame.copyTo(symbolRaw, mask);




    if(!quads.empty())
    {



        Mat symbolTransformed = transformPerspective(quads[largeContour], symbolRaw, 320, 240);

        Mat circle = cv::imread("/home/pi/Desktop/OpenCV-Template/Circle.png", cv::IMREAD_GRAYSCALE);
        Mat star = cv::imread("/home/pi/Desktop/OpenCV-Template/Star.png", cv::IMREAD_GRAYSCALE);
        Mat triangle = cv::imread("/home/pi/Desktop/OpenCV-Template/Triangle.png", cv::IMREAD_GRAYSCALE);
        Mat umbrella = cv::imread("/home/pi/Desktop/OpenCV-Template/Umbrella.png", cv::IMREAD_GRAYSCALE);

        cv::threshold(circle, circle, 128, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
        cv::threshold(star, star, 128, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
        cv::threshold(triangle, triangle, 128, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
        cv::threshold(umbrella, umbrella, 128, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

        cv::resize(symbolTransformed, symbolTransformed, circle.size());

        cv::cvtColor(symbolTransformed, symbolTransformed, cv::COLOR_BGR2GRAY);
        cv::threshold(symbolTransformed, symbolTransformed, 128, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

        cv::imshow("mama", symbolTransformed);
        cv::moveWindow("mama", 1000, 400);

        float highest = 0;
        int highestID = 0;
        float compare = compareImages(symbolTransformed, circle);
        if(compare > highest)
        {
            highest = compare;
            highestID = 1;

        }
        compare = compareImages(symbolTransformed, star);
        if(compare > highest)
        {
            highest = compare;
            highestID = 2;

        }
        compare = compareImages(symbolTransformed, triangle);
        if(compare > highest)
        {
            highest = compare;
            highestID = 3;

        }
        compare = compareImages(symbolTransformed, umbrella);
        if(compare > highest)
        {
            highest = compare;
            highestID = 4;

        }

        int threshold = 0;

        if(highest > threshold)
        {
            return highestID;
        }





    }


    return 0;
}

//Credit opencv docs
double angle(cv::Point pt1, cv::Point pt2, cv::Point pt0)
{
    double dx1 = pt1.x - pt0.x;
    double dy1 = pt1.y - pt0.y;
    double dx2 = pt2.x - pt0.x;
    double dy2 = pt2.y - pt0.y;
    return (dx1*dx2 + dy1*dy2)/std::sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
}

int pointXPos = 0;

void GetLinePosition(int hsvArr[])
{

    Mat frame;

    frame = currentCapture;

    cv::rotate(frame, frame, cv::ROTATE_180);

    Mat gray;
    Mat finalImage;
    Mat object;
    cv::cvtColor(frame, object, COLOR_BGR2HSV);
    cv::inRange(object, cv::Scalar(hsvArr[0], hsvArr[2], hsvArr[4]), cv::Scalar(hsvArr[1], hsvArr[3], hsvArr[5]), finalImage);

    erode(finalImage, finalImage, cv::getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
    dilate(finalImage, finalImage, cv::getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
    dilate(finalImage, finalImage, cv::getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
    erode(finalImage, finalImage, cv::getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

    Mat contourOut = finalImage.clone();
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(contourOut, contours, RETR_LIST, CHAIN_APPROX_NONE);


    cv::Mat contourImage(finalImage.size(), CV_8UC3, cv::Scalar(0, 0, 0));
    cv::Scalar color;


    cv::imshow("Test", finalImage);
    cv::moveWindow("Test", 1000, 400);

    if(!contours.empty())
    {

        color = cv::Scalar(255, 192, 203);
        float maxArea = 0;
        int largeContour = 0;
        for(int i = 0; i < contours.size(); i++)
        {
            cv::drawContours(contourImage, contours, i, color);
            float area = (float)cv::contourArea(contours[i]);
            if(area > maxArea)
            {
                largeContour = i;
                maxArea = area;
            }
        }

        cv::RotatedRect bounding_rect = cv::minAreaRect(contours[largeContour]);

        cv::Point contourCenter = findContourCentre(contours[largeContour]);


        pointXPos = contourCenter.x;


        Point2f rect_points[4];
        bounding_rect.points(rect_points);
        for(int i = 0; i < 4; i++)
        {
            cv::line(frame, rect_points[i], rect_points[(i+1)%4], Scalar(0, 255, 0), 2);
        }

        cv::imshow("Standard", frame); //Display the image in the window
        cv::moveWindow("Standard", 400, 400);

        cv::imshow("Contours", contourImage);
        cv::moveWindow("Contours", 700, 400);

        weightedAverage = MapFunction(pointXPos - 160, -160, 160, -1, 1);
        std::cout << "Weighted Average" << weightedAverage << "\n";
    }

    cv::imshow("Standard", frame); //Display the image in the window
    cv::moveWindow("Standard", 400, 400);
    cv::imshow("Contours", contourImage);
    cv::moveWindow("Contours", 700, 400);
}

void ByteToCharArray(uint8_t b, char* array)
{
    for(int i = 0; i < 8; i++)
    {
        array[7-i] = ((b >> i) & 1) ? '1' : '0';
    }
    array[8] = '\0';
}

void Transmit()
{
    //Wire.write((byte)((x & 0xFF000000) >> 24)); // bits 32 to 25 of x
    //Wire.write((byte)((x & 0x00FF0000) >> 16)); // bits 24 to 17 of x
    /*
    char array[6];
    array[0] = (char)((weightedAverage & 0x0000FF00) >> 8);
    array[1] = (char)(leftMotor_speed & 0x000000FF);
    array[2] = (char)((rightMotor_speed & 0x0000FF00) >> 8);
    array[3] = (char)(rightMotor_speed & 0x00000FF);
    array[4] = (char)((servoAngle & 0x0000FF00) >> 8);
    array[5] = (char)(servoAngle & 0x000000FF);
    */
    car.i2cWriteArduinoInt(weightedAverage);

    //sleep(0.1f);

}

float MapFunction(float x, float inMin, float inMax, float outMin, float outMax)
{

    return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;

}

int main( int argc, char** argv )
{
    setup();    // Call a setup function to prepare IO and devices

    //cv::namedWindow("Photo");   // Create a GUI window called photo


    int redRanges[] = {340, 20, 60, 100, 60, 100};
    int blueRanges[] = {190, 260, 60, 100, 60, 100};
    int greenRanges[] = {58, 85, 101, 255, 0, 156};
    int yellowRanges[] = {46, 83, 26, 255, 0, 161};
    int blackRanges[] = {0, 179, 0, 0, 0, 55};
    int purpleRanges[] = {117, 170, 23, 255, 0, 255};

    std::cout << "hi";

    int colourToUse[] = {blackRanges[0], blackRanges[1], blackRanges[2], blackRanges[3], blackRanges[4], blackRanges[5]};

    while(1)    // Main loop to perform image processing
    {

        while(currentCapture.empty())
            currentCapture = captureFrame(); // Capture a frame from the camera and store in a new matrix variable

        int ID = (int) LookForSymbol(purpleRanges);

        

        switch (ID){

            case 1: {
                std::cout << "Detected Symbol: Circle\n";

                GetLinePosition(redRanges);

                break;
            }
            case 2: {
                std::cout << "Detected Symbol: Star\n";

                GetLinePosition(blueRanges);

                break;
            }
            case 3: {
                std::cout << "Detected Symbol: Triangle\n";

                GetLinePosition(greenRanges);

                break;
            }
            case 4: {
                std::cout << "Detected Symbol: Umbrella\n";

                GetLinePosition(yellowRanges);

                break;
            }

        }

        

        //Add a function that detects what colours are currently seen and will switch back to black the current colour is not found
        //for  more than a certain time



        

        currentCapture.release();

        int key = cv::waitKey(1);   // Wait 1ms for a keypress (required to update windows)

        Transmit();






    }

    closeCV();  // Disable the camera and close any windows

    return 0;
}



