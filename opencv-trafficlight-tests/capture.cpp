/*
 *
 *  Example by Sam Siewert from ECEN 5623
 *  Used as starter code
 *  Modified by Trapti Balgi for traffic light detection
 *
 */


#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>

#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

using namespace cv;
using namespace std;

#define HRES 640
#define VRES 480

string detectTrafficLightColor(const Mat& roi)
{
    Mat hsv;
    cvtColor(roi, hsv, COLOR_BGR2HSV);

    // Define HSV ranges for red, yellow, and green colors
    Scalar lowerRed1(0, 100, 100), upperRed1(10, 255, 255);
    Scalar lowerRed2(170, 100, 100), upperRed2(180, 255, 255);
    Scalar lowerYellow(20, 100, 100), upperYellow(30, 255, 255);
    Scalar lowerGreen(40, 100, 100), upperGreen(80, 255, 255);

    Mat maskRed1, maskRed2, maskYellow, maskGreen;
    inRange(hsv, lowerRed1, upperRed1, maskRed1);
    inRange(hsv, lowerRed2, upperRed2, maskRed2);
    inRange(hsv, lowerYellow, upperYellow, maskYellow);
    inRange(hsv, lowerGreen, upperGreen, maskGreen);

    // Check for the most prevalent color
    int redCount = countNonZero(maskRed1) + countNonZero(maskRed2);
    int yellowCount = countNonZero(maskYellow);
    int greenCount = countNonZero(maskGreen);

    if (redCount > yellowCount && redCount > greenCount)
        return "Red";
    else if (yellowCount > redCount && yellowCount > greenCount)
        return "Yellow";
    else if (greenCount > redCount && greenCount > yellowCount)
        return "Green";
    else
        return "Unknown";
}

int main(int argc, char** argv)
{
    namedWindow("Capture Example", WINDOW_AUTOSIZE);

    VideoCapture capture;
    Mat frame, gray, roi;
    vector<vector<Point>> rectangles;

    int dev = 0;

    if (argc > 1)
    {
        sscanf(argv[1], "%d", &dev);
        printf("Using device %s\n", argv[1]);
    }
    else if (argc == 1)
    {
        printf("Using default device\n");
    }
    else
    {
        printf("Usage: capture [dev]\n");
        exit(-1);
    }

    if (!capture.open(dev))
    {
        cerr << "Error: Unable to open the camera\n";
        return -1;
    }

    capture.set(CAP_PROP_FRAME_WIDTH, HRES);
    capture.set(CAP_PROP_FRAME_HEIGHT, VRES);

    while (true)
    {
        capture >> frame;
        if (frame.empty())
        {
            cerr << "Error: Unable to capture frame\n";
            break;
        }

        // Convert to grayscale and detect edges for rectangle detection
        cvtColor(frame, gray, COLOR_BGR2GRAY);
        Canny(gray, gray, 50, 150);

        // Find contours
        vector<vector<Point>> contours;
        findContours(gray, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

        // Detect rectangles from contours with width/height aspect ratio constraints
        for (size_t i = 0; i < contours.size(); i++)
        {
            if (contourArea(contours[i]) > 500)
            {
                Rect rect = boundingRect(contours[i]);
                
                // Check if the rectangle has a much smaller width than height (traffic light shape)
                if (rect.width < rect.height / 2)
                {
                    rectangle(frame, rect, Scalar(0, 255, 255), 2);

                    // Crop the ROI for further processing
                    roi = frame(rect);
                    
                    // Detect circles within the ROI to ensure they are traffic light components
                    Mat grayROI;
                    cvtColor(roi, grayROI, COLOR_BGR2GRAY);
                    GaussianBlur(grayROI, grayROI, Size(9, 9), 2, 2);
                    vector<Vec3f> circles;
                    HoughCircles(grayROI, circles, HOUGH_GRADIENT, 1, grayROI.rows / 8, 100, 50, 0, 0);

                    for (size_t j = 0; j < circles.size(); j++)
                    {
                        Point center(cvRound(circles[j][0]), cvRound(circles[j][1]));
                        int radius = cvRound(circles[j][2]);
                        circle(roi, center, 3, Scalar(0, 255, 0), -1);
                        circle(roi, center, radius, Scalar(0, 0, 255), 3);
                    }

                    // Detect the color in the ROI
                    string color = detectTrafficLightColor(roi);
                    putText(frame, color, Point(rect.x, rect.y - 10), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 255, 0), 2);
                }
            }
        }

        imshow("Capture Example", frame);

        char c = (char)waitKey(10);
        if (c == 27) // ESC key
        {
            break;
        }
    }

    capture.release();
    destroyWindow("Capture Example");

    return 0;
}
