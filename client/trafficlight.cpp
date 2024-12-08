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

#define PORT_NUM (9000)
#define SERVER_IP "10.0.0.127"//"127.0.0.1" //"10.0.0.127"
#define ERROR (-1)
#define HRES (640)
#define VRES (480)
#define FRAME_SIZE (HRES * VRES * 3)

#include <wiringPi.h>

#define GREEN_LED_PIN 0 // GPIO17
#define RED_LED_PIN 1 // GPIO18
#define YELLOW_LED_PIN 3 // GPIO22

using namespace cv;
using namespace std;

volatile unsigned caught_signal = 0;

enum TrafficLightColor {
    RED,
    YELLOW,
    GREEN
};

static void signal_handler(int signal_number)
{
    caught_signal = signal_number;
}

void setupGPIO() 
{
    wiringPiSetup();
    pinMode(GREEN_LED_PIN, OUTPUT);
    digitalWrite(GREEN_LED_PIN, LOW);
    pinMode(RED_LED_PIN, OUTPUT);
    digitalWrite(RED_LED_PIN, LOW);
    pinMode(YELLOW_LED_PIN, OUTPUT);
    digitalWrite(YELLOW_LED_PIN, LOW); 
}

void controlLED(TrafficLightColor color) 
{
    digitalWrite(GREEN_LED_PIN, LOW);
    digitalWrite(YELLOW_LED_PIN, LOW);
    digitalWrite(RED_LED_PIN, LOW);

    switch (color) 
    {
        case RED:
            digitalWrite(RED_LED_PIN, HIGH);
            break;
        case YELLOW:
            digitalWrite(YELLOW_LED_PIN, HIGH);
            break;
        case GREEN:
            digitalWrite(GREEN_LED_PIN, HIGH);
            break;
    }
}

void detect(cv::Mat& img) 
{
    cv::Mat cimg = img.clone();
    cv::Mat hsv, mask1, mask2, maskg, masky;
    int font = cv::FONT_HERSHEY_SIMPLEX;

    // Convert image to HSV color space
    cv::cvtColor(img, hsv, cv::COLOR_BGR2HSV);

    // Define color ranges
    cv::Scalar lower_red1(0, 100, 100), upper_red1(10, 255, 255);
    cv::Scalar lower_red2(160, 100, 100), upper_red2(180, 255, 255);
    cv::Scalar lower_green(40, 50, 50), upper_green(90, 255, 255);
    cv::Scalar lower_yellow(15, 150, 150), upper_yellow(35, 255, 255);

    cv::inRange(hsv, lower_red1, upper_red1,mask1);
    cv::inRange(hsv, lower_red2, upper_red2,mask2);
    cv::inRange(hsv, lower_green, upper_green,maskg);
    cv::inRange(hsv, lower_yellow, upper_yellow,masky);
    cv::Mat maskr = mask1 + mask2;

    int r = 5;
    double bound = 7.0 / 10.0;
    cv::Size size = img.size();

    // Hough Circle Detection
    std::vector<cv::Vec3f> r_circles, g_circles, y_circles;
    cv::HoughCircles(maskr, r_circles, cv::HOUGH_GRADIENT, 1, 80, 50, 10, 0, 30);
    cv::HoughCircles(maskg, g_circles, cv::HOUGH_GRADIENT, 1, 60, 50, 10, 0, 30);
    cv::HoughCircles(masky, y_circles, cv::HOUGH_GRADIENT, 1, 30, 50, 5, 0, 30);
    
    int detected = 0;

    // Red circle detection
    for (const auto& i : r_circles) 
    {
        if (i[0] > size.width || i[1] > size.height || i[1] > size.height * bound)
            continue;

        double h = 0.0, s = 0.0;
        for (int m = -r; m < r; ++m) 
        {
            for (int n = -r; n < r; ++n) 
            {
                if ((i[1] + m) >= size.height || (i[0] + n) >= size.width)
                    continue;
                h += maskr.at<uchar>(i[1] + m, i[0] + n);
                s += 1;
            }
        }
        if (h / s > 50) 
        {
            cv::circle(cimg, cv::Point(i[0], i[1]), i[2] + 10, cv::Scalar(0, 255, 0), 2);
            cv::putText(cimg, "RED", cv::Point(i[0], i[1]), font, 1, cv::Scalar(255, 0, 0), 2, cv::LINE_AA);
            controlLED(RED);
        }      
    }

    // Green circle detection
    for (const auto& i : g_circles) 
    {
        if (i[0] > size.width || i[1] > size.height || i[1] > size.height * bound)
            continue;

        double h = 0.0, s = 0.0;
        for (int m = -r; m < r; ++m) 
        {
            for (int n = -r; n < r; ++n) 
            {
                if ((i[1] + m) >= size.height || (i[0] + n) >= size.width)
                    continue;
                h += maskg.at<uchar>(i[1] + m, i[0] + n);
                s += 1;
            }
        }
        if (h / s > 100) 
        {
            cv::circle(cimg, cv::Point(i[0], i[1]), i[2] + 10, cv::Scalar(0, 255, 0), 2);
            cv::putText(cimg, "GREEN", cv::Point(i[0], i[1]), font, 1, cv::Scalar(255, 0, 0), 2, cv::LINE_AA);
            controlLED(GREEN);
        }
    }

    // Yellow circle detection
    for (const auto& i : y_circles) 
    {
        if (i[0] > size.width || i[1] > size.height || i[1] > size.height * bound)
            continue;

        double h = 0.0, s = 0.0;
        for (int m = -r; m < r; ++m) 
        {
            for (int n = -r; n < r; ++n) 
            {
                if ((i[1] + m) >= size.height || (i[0] + n) >= size.width)
                    continue;
                h += masky.at<uchar>(i[1] + m, i[0] + n);
                s += 1;
            }
        }
        if (h / s > 50) 
        {
            cv::circle(cimg, cv::Point(i[0], i[1]), i[2] + 10, cv::Scalar(0, 255, 0), 2);
            cv::putText(cimg, "YELLOW", cv::Point(i[0], i[1]), font, 1, cv::Scalar(255, 0, 0), 2, cv::LINE_AA);
            controlLED(YELLOW);
        }
    }
    

    cv::imshow("Detected Results", cimg);
    cv::waitKey(1);
}

int receive_and_process_data(int sockfd)
{
    syslog(LOG_DEBUG, "in receive_and_process_data");
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
            Mat frame(VRES, HRES, CV_8UC3, buffer.data());
            if (frame.empty())
            {
                syslog(LOG_ERR, "Error: Received empty frame");
                continue;
            }
            
            // Convert BGR to RGB for displaying
            Mat display_frame;
            cvtColor(frame, display_frame, COLOR_BGR2RGB);
            
            detect(display_frame);
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
    
    setupGPIO();

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
