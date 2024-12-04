#include "opencv2/opencv.hpp"
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <stdbool.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <syslog.h>
#include <signal.h>
#include <pthread.h>
#include <iostream>

#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

#define PORT_NUM 9000
#define BUF_SIZE 320 * 240 * 3
#define ERROR (-1)

using namespace cv;
using namespace std;

volatile unsigned caught_signal = 0;

static void signal_handler(int signal_number)
{
    caught_signal = signal_number;
}

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

int receive_and_process_data(int sockfd)
{
    syslog(LOG_DEBUG, "in receive_and_process_data");
    const int FRAME_SIZE = 320 * 240 * 3;
    std::vector<uchar> buffer(FRAME_SIZE);
    int retval = 0;
    int frame_count = 0;
    Mat frame, gray, roi;
    vector<vector<Point>> rectangles;

    while (!caught_signal)
    {
        size_t total_received = 0;
        while (total_received < FRAME_SIZE)
        {
            ssize_t length = recv(sockfd, buffer.data() + total_received, FRAME_SIZE - total_received, 0);
            if (length <= 0)
            {
                if (length == 0)
                    syslog(LOG_INFO, "Server disconnected.");
                else
                    syslog(LOG_ERR, "recv failed: %s", strerror(errno));
                return ERROR;
            }
            total_received += length;
        }

        syslog(LOG_DEBUG, "Total received: %zu bytes", total_received);

        if (total_received == FRAME_SIZE)
        {
            Mat frame(240, 320, CV_8UC3, buffer.data());
            if (frame.empty())
            {
                syslog(LOG_ERR, "Error: Received empty frame");
                continue;
            }
            
            // Convert BGR to RGB for displaying
            
            Mat display_frame;
            cvtColor(frame, display_frame, COLOR_BGR2RGB);
            
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
                    putText(display_frame, color, Point(rect.x, rect.y - 10), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 255, 0), 2);
                }
            }
        }

        imshow("Traffic Light Detection", display_frame);

            if (waitKey(1) == 27) // Exit on 'Esc' key
                break;
        }
        else
        {
            syslog(LOG_ERR, "Received incomplete frame: %zu bytes", total_received);
            retval = ERROR;
        }
    }

    return retval;
}

int main()
{
    openlog("socket_client", LOG_PID | LOG_CONS, LOG_USER);
    int sockfd;
    struct sockaddr_in server_addr;

    struct sigaction new_action;
    memset(&new_action, 0, sizeof(struct sigaction));
    new_action.sa_handler = signal_handler;

    if (sigaction(SIGTERM, &new_action, NULL) != 0)
    {
        syslog(LOG_ERR, "Sigaction for SIGTERM failed");
    }
    if (sigaction(SIGINT, &new_action, NULL) != 0)
    {
        syslog(LOG_ERR, "Sigaction for SIGINT failed");
    }

    if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) == -1)
    {
        syslog(LOG_ERR, "Failed to create socket: %s", strerror(errno));
        exit(EXIT_FAILURE);
    }

    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(PORT_NUM);

    if (inet_pton(AF_INET, SERVER_IP, &server_addr.sin_addr) <= 0)
    {
        syslog(LOG_ERR, "Invalid server IP address: %s", SERVER_IP);
        close(sockfd);
        exit(EXIT_FAILURE);
    }

    if (connect(sockfd, (struct sockaddr *)&server_addr, sizeof(server_addr)) == -1)
    {
        syslog(LOG_ERR, "Connection to server failed: %s", strerror(errno));
        close(sockfd);
        exit(EXIT_FAILURE);
    }

    syslog(LOG_INFO, "Connected to server at %s:%d", SERVER_IP, PORT_NUM);

    if (receive_and_process_data(sockfd) != 0)
    {
        syslog(LOG_ERR, "Error while receiving data");
    }

    close(sockfd);
    closelog();
    return 0;
}
