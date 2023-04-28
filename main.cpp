// Include files for required libraries
#include <stdio.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <cstddef>

#include "opencv_aee.hpp"
#include "main.hpp"     // You can use this file for declaring defined values and functions
#include "pi2c.h"

Pi2c car(4); // Configure the I2C interface to the Car as a global variable

int leftMotor_speed = 150;
int rightMotor_speed = 150;
int servoAngle = 140;

Mat currentCapture;

void setup(void)
{
    setupCamera(320, 240);  // Enable the camera for OpenCV


}



float LookForSymbol(int hsvArr[]) {

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

    cv::imshow("SymbolTransformed", finalImage);
        cv::moveWindow("SymbolTransformed", 400, 400);


    if(!contours.empty())
    {
        //Find all quads in the image
        std::vector<std::vector<cv::Point>> quads;
        for(int i = 0; i < contours.size(); i++)
        {
            std::vector<cv::Point> point;
            cv::approxPolyDP(contours[i], point, cv::arcLength(contours[i], true) * 0.05, true);
            if(point.size() == 4 && cv::isContourConvex(point)) {
                quads.push_back(point);
            }
        }

        if(!quads.empty())
        {

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

        cv::Mat mask(frame.size(), CV_64FC1);

        cv::drawContours(mask, quads[largeContour], -1, Scalar(255));

        Mat symbolRaw;
        /*
        cv::bitwise_and(finalImage, finalImage, symbolRaw, mask);

        Mat symbolTransformed;

        transformPerspective(quads[largeContour], symbolTransformed, 350, 350);

        std::cout << "cunt";




        Mat circle = cv::imread("Circle.png");
        Mat star = cv::imread("Star.png");
        Mat triangle = cv::imread("Triangle.png");
        Mat umbrella = cv::imread("Umbrella.png");

        float highest = 0;
        float highestID = 0;
        float compare = compareImages(symbolTransformed, circle);
        if(compare > highest) {
            highest = compare;
            highestID = 1;
        }
        compare = compareImages(symbolTransformed, star);
        if(compare > highest) {
            highest = compare;
            highestID = 2;
        }
        compare = compareImages(symbolTransformed, triangle);
        if(compare > highest) {
            highest = compare;
            highestID = 3;
        }
        compare = compareImages(symbolTransformed, umbrella);
        if(compare > highest) {
            highest = compare;
            highestID = 4;
        }

        int threshold = 0;

        if(highest > threshold) {
            return highestID;
        }
        */
        }

    }

    return 0;
}

int pointXPos = 0;

float GetLinePosition(int H_lower, int H_higher, int S_lower, int S_higher, int V_lower, int V_higher)
{

    Mat frame;

    frame = currentCapture;

    cv::rotate(frame, frame, cv::ROTATE_180);

    Mat gray;
    Mat finalImage;
    Mat object;
    cv::cvtColor(frame, object, COLOR_BGR2HSV);
    cv::inRange(object, cv::Scalar(H_lower, S_lower, V_lower), cv::Scalar(H_higher, S_higher, V_higher), finalImage);

    erode(finalImage, finalImage, cv::getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
    dilate(finalImage, finalImage, cv::getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
    dilate(finalImage, finalImage, cv::getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
    erode(finalImage, finalImage, cv::getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

    Mat contourOut = finalImage.clone();
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(contourOut, contours, RETR_LIST, CHAIN_APPROX_NONE);


    cv::Mat contourImage(finalImage.size(), CV_8UC3, cv::Scalar(0, 0, 0));
    cv::Scalar color;




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
        cv::moveWindow("Standard", 0, 0);

        cv::imshow("Contours", contourImage);
        cv::moveWindow("Contours", 400, 0);

        float weightedAverage = MapFunction(pointXPos - 160, -160, 160, -1, 1);
        std::cout << weightedAverage << "\n";

        return 0;
    }

    cv::imshow("Standard", frame); //Display the image in the window
    cv::moveWindow("Standard", 0, 0);
    cv::imshow("Contours", contourImage);
    cv::moveWindow("Contours", 400, 0);


    return 0;
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

    char array[6];
    array[0] = (char)((leftMotor_speed & 0x0000FF00) >> 8);
    array[1] = (char)(leftMotor_speed & 0x000000FF);
    array[2] = (char)((rightMotor_speed & 0x0000FF00) >> 8);
    array[3] = (char)(rightMotor_speed & 0x00000FF);
    array[4] = (char)((servoAngle & 0x0000FF00) >> 8);
    array[5] = (char)(servoAngle & 0x000000FF);
    car.i2cWriteArduinoInt(pointXPos);

    //sleep(0.1f);

}

float MapFunction(float x, float inMin, float inMax, float outMin, float outMax) {

    return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;

}

int main( int argc, char** argv )
{
    setup();    // Call a setup function to prepare IO and devices

    cv::namedWindow("Photo");   // Create a GUI window called photo


    int redRanges[] = {340, 20, 60, 100, 60, 100};
    int blueRanges[] = {190, 260, 60, 100, 60, 100};
    int greenRanges[] = {80, 160, 60, 100, 60, 100};
    int yellowRanges[] = {40, 80, 60, 100, 60, 100};
    int purpleRanges[] = {135, 165, 70, 255, 70, 255};

    std::cout << "hi";

    while(1)    // Main loop to perform image processing
    {

        while(currentCapture.empty())
            currentCapture = captureFrame(); // Capture a frame from the camera and store in a new matrix variable

        //float ID = LookForSymbol(purpleRanges);

        //std::cout << "Symbol: " << ID << "\n";

        GetLinePosition(35, 86, 70, 255, 70, 255);

        currentCapture.release();

        int key = cv::waitKey(1);   // Wait 1ms for a keypress (required to update windows)

        Transmit();






    }

    closeCV();  // Disable the camera and close any windows

    return 0;
}



