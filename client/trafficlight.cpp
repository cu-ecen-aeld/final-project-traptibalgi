/**
 * @file trafficlight.cpp
 * @brief Traffic Light Detection and Control System - Client Side
 *
 * This program receives frames from a server of the traffic light detection system, processes the frames to
 * identify traffic light colors (red, yellow, green), and controls LEDs corresponding to
 * the detected traffic light color. The system uses OpenCV for image processing and socket
 * communication for frame reception.
 *
 * Features:
 * - Traffic light detection.
 * - Real-time LED control based on detected light.
 *
 * @author Trapti Damodar Balgi
 * @date 29th November 2024
 * 
 * References:
 * 1. https://docs.opencv.org/4.x/da/d53/tutorial_py_houghcircles.html
 * 2. https://docs.opencv.org/4.x/df/d9d/tutorial_py_colorspaces.html
 * 3. https://github.com/cu-ecen-aeld/buildroot-assignments-base/wiki/V4L2-Drivers-for-Camera-interfacing
 * 4. https://roboticsbackend.com/introduction-to-wiringpi-for-raspberry-pi/
 * 5. https://github.com/HevLfreis/TrafficLight-Detector
 */

#include "opencv2/opencv.hpp"
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <getopt.h>             /* getopt_long() */
#include <fcntl.h>              /* low-level i/o */
#include <netdb.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <stdbool.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <syslog.h>
#include <signal.h>
#include <pthread.h>
#include <iostream>
#include <wiringPi.h>
#include <thread>
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

using namespace cv;
using namespace std;

#define PORT_NUM (9000)
#define ERROR (-1)
#define HRES (640)
#define VRES (480)
#define FRAME_SIZE (HRES * VRES * 3)
#define NUM_THREADS (2)
#define GREEN_LED_PIN (0) // GPIO17
#define RED_LED_PIN (1) // GPIO18
#define YELLOW_LED_PIN (3) // GPIO22
#define MOVING_AVERAGE_WINDOW (1000)

/* Enumeration for traffic light colors. */
enum TrafficLightColor {
    RED,
    YELLOW,
    GREEN
};

int client_fd;
cv::Mat latest_frame;
struct addrinfo *res;  // will point to the results
pthread_mutex_t frame_mutex;
volatile unsigned caught_signal = 0;
bool timeout_flag;

/* The structure for the receiver thread*/
typedef struct receive_params
{
    pthread_t thread_id;
    int sock_fd;
    struct sockaddr_in sock_addr;
} receive_params_t;

/**
 * @brief Cleans up resources during program termination.
 */
void cleanup() 
{
    if (client_fd != -1) 
    {
        shutdown(client_fd, SHUT_RDWR);
        close(client_fd);
    }
    digitalWrite(YELLOW_LED_PIN, LOW);
    digitalWrite(GREEN_LED_PIN, LOW);
    digitalWrite(RED_LED_PIN, LOW);
    closelog();
}

/**
 * @brief Signal handler for program termination.
 * @param signal_number The signal number received.
 */
static void signal_handler(int signal_number)
{
    caught_signal = signal_number;
    syslog(LOG_INFO, "Signal %d caught", signal_number);
}

/**
 * @brief Initializes GPIO pins for LED control.
 */
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

/**
 * @brief Controls the state of LEDs based on detected traffic light color.
 * @param color Detected traffic light color.
 */
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

/**
 * @brief Detects traffic light color from a given frame.
 * @param img Input frame for traffic light detection.
 * Adapted from: https://github.com/HevLfreis/TrafficLight-Detector
 */
void detect(cv::Mat& img) 
{
    cv::Mat cimg = img.clone();
    cv::Mat hsv, mask1, mask2, maskg, masky;
    int font = cv::FONT_HERSHEY_SIMPLEX;

    /* Convert image to HSV color space */
    cv::cvtColor(img, hsv, cv::COLOR_BGR2HSV);

    /* Define color ranges */
    cv::Scalar lower_red1(0, 100, 100), upper_red1(10, 255, 255);
    cv::Scalar lower_red2(160, 100, 100), upper_red2(180, 255, 255);
    cv::Scalar lower_green(40, 50, 50), upper_green(90, 255, 255);
    cv::Scalar lower_yellow(15, 150, 150), upper_yellow(35, 255, 255);

    /* Create masks */
    cv::inRange(hsv, lower_red1, upper_red1,mask1);
    cv::inRange(hsv, lower_red2, upper_red2,mask2);
    cv::inRange(hsv, lower_green, upper_green,maskg);
    cv::inRange(hsv, lower_yellow, upper_yellow,masky);
    cv::Mat maskr = mask1 + mask2;

    /* 5 pixel radius, bound is 0.7 the height*/
    int r = 5;
    double bound = 7.0 / 10.0;
    cv::Size size = img.size();

    /* Hough Circle Detection */
    /*  i[0]: The x-coordinate of the circle's center.
        i[1]: The y-coordinate of the circle's center.
        i[2]: The radius of the circle. */
    std::vector<cv::Vec3f> r_circles, g_circles, y_circles;
    cv::HoughCircles(maskr, r_circles, cv::HOUGH_GRADIENT, 1, 80, 50, 10, 0, 30);
    cv::HoughCircles(maskg, g_circles, cv::HOUGH_GRADIENT, 1, 60, 50, 10, 0, 30);
    cv::HoughCircles(masky, y_circles, cv::HOUGH_GRADIENT, 1, 30, 50, 5, 0, 30);

    /* Red circle detection */
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

    /* Green circle detection */
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

    /* Yellow circle detection */
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
    
    cv::imshow("Traffic Light Detection", cimg);
    cv::waitKey(1);
}

/**
 * @brief Thread function to process video frames.
 * @param arg Thread arguments (unused).
 * @return void*
 */
void *process_thread(void *arg)
{
    syslog(LOG_DEBUG, "process_thread");

    while ((!caught_signal) && (timeout_flag == 0))
    {
        cv::Mat frame;
        static auto last_time = std::chrono::steady_clock::now();
        double average_fps = 0.0;
        int frame_count = 0;
        
        if (pthread_mutex_lock(&frame_mutex) != 0)
        {
            syslog(LOG_ERR, "process_thread: Failed to lock mutex");
            break;
        }
        
        /* Check if a new frame is available */
        if (!latest_frame.empty()) 
        {
            frame = latest_frame.clone();
            pthread_mutex_unlock(&frame_mutex);
            detect(frame);

            /* Calculate and log FPS */
            auto current_time = std::chrono::steady_clock::now();
            std::chrono::duration<double> elapsed = current_time - last_time;
            double fps = 1.0 / elapsed.count();
            last_time = current_time;

            average_fps = ((average_fps * frame_count) + fps) / (frame_count + 1);
            frame_count = std::min(frame_count + 1, MOVING_AVERAGE_WINDOW);

            syslog(LOG_INFO, "PROCESS: Current FPS: %.2f, Average FPS: %.2f", fps, average_fps);
        } 
        else 
        {
            pthread_mutex_unlock(&frame_mutex);
            usleep(10 * 1000);
        }
    }

    pthread_exit(NULL);
}

/**
 * @brief Thread function to receive video frames over a socket.
 * @param receive_params_struct Pointer to receive thread parameters.
 * @return void*
 */
void *receive_thread(void *receive_params_struct)
{
    syslog(LOG_DEBUG, "In receive_thread");

    receive_params_t *receive_params = (receive_params_t*)receive_params_struct;
    if (receive_params == NULL)
    {
        syslog(LOG_ERR, "receive_thread: Receiver thread struct is NULL");
        pthread_exit(NULL);
    }

    static auto last_time = std::chrono::steady_clock::now();
    struct sockaddr_in client_addr = receive_params->sock_addr;
    socklen_t client_len = sizeof(client_addr);

    double average_fps = 0.0;
    int frame_count = 0;

    while (!caught_signal) 
    {
        int bytes_received;
        std::vector<unsigned char> jpeg_buffer(FRAME_SIZE);

        struct timeval timeout;
        fd_set readfds;
        int retval;

        timeout.tv_sec = 5;  // 5 seconds timeout
        timeout.tv_usec = 0;

        FD_ZERO(&readfds);
        FD_SET(receive_params->sock_fd, &readfds);

        retval = select(receive_params->sock_fd + 1, &readfds, NULL, NULL, &timeout);

        if (retval == -1) 
        {
            syslog( LOG_ERR, "select() error occured");
            pthread_exit(NULL);
        } 
        else if (retval == 0) 
        {
            timeout_flag = true;
            syslog( LOG_ERR, "select() timeout occured on socket");
            pthread_exit(NULL);
        } 
        else 
        {
            /* Receive the JPEG data */
            bytes_received = recvfrom(receive_params->sock_fd, jpeg_buffer.data(), jpeg_buffer.size(), 0, (struct sockaddr *)&(client_addr), &client_len);
        }

        if (bytes_received < 0)
        {
            syslog(LOG_ERR, "recvfrom() failed. Error: %s", strerror(errno));
            continue;
        }
        else if (bytes_received > 0)
        {
            syslog(LOG_DEBUG, "Received %d bytes", bytes_received);

            /* Resize to the actual received size */
            jpeg_buffer.resize(bytes_received);  

            /* Decode the JPEG data */
            cv::Mat image = cv::imdecode(jpeg_buffer, cv::IMREAD_COLOR);
            if (image.empty()) 
            {
                syslog(LOG_ERR, "Failed to decode JPEG image");
                continue;
            }
            
            /* Calculate and log FPS */
            auto current_time = std::chrono::steady_clock::now();
            std::chrono::duration<double> elapsed = current_time - last_time;
            double fps = 1.0 / elapsed.count();
            last_time = current_time;

            average_fps = ((average_fps * frame_count) + fps) / (frame_count + 1);
            frame_count = std::min(frame_count + 1, MOVING_AVERAGE_WINDOW);

            syslog(LOG_INFO, "RECEIVE: Current FPS: %.2f, Average FPS: %.2f", fps, average_fps);

            if (pthread_mutex_lock(&frame_mutex) != 0)
            {
                syslog(LOG_ERR, "receive_thread: Failed to lock mutex");
                break;
            }
            latest_frame = image.clone();
            pthread_mutex_unlock(&frame_mutex);
        }
    }

    pthread_exit(NULL);
}

/**
 * @brief Entry point of the program.
 * @return int Exit status.
 */
int main()
{
    openlog("client", LOG_PID | LOG_CONS, LOG_USER);

    pthread_t process_thread_id;
    receive_params_t *receive_params = NULL;

    int status;
    int optval = 1;
    struct sockaddr_in client_addr;
    struct addrinfo hints;
    
    /* Setup signal handlers*/
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

    /* Create a mutex for synchronising access to the frame*/
    if(pthread_mutex_init(&frame_mutex, NULL) != 0)
    {
        syslog(LOG_ERR, "Creating mutex failed");
        exit(EXIT_FAILURE);
    }
    
    setupGPIO();

    memset(&hints, 0, sizeof hints);    // Make sure the struct is empty
    hints.ai_family = AF_INET;          // IPv4
    hints.ai_socktype = SOCK_DGRAM;    // TCP stream sockets
    hints.ai_flags = AI_PASSIVE;        // Fill in my IP for me

    if ((status = getaddrinfo(NULL, "9000", &hints, &res)) != 0) 
    {
        syslog(LOG_ERR, "getaddrinfo failed");
        goto exit_on_fail;
    }

    /* Create a socket */
    if ((client_fd = socket(res->ai_family, res->ai_socktype, res->ai_protocol)) == -1)
    {
        syslog(LOG_ERR, "Failed to make a socket");
        goto exit_on_fail;
    }

    /* Allow reuse of socket */
    if (setsockopt(client_fd, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof(optval)) == -1) 
    {
        syslog(LOG_ERR, "Error setting socket options (SO_REUSEADDR)");
        goto exit_on_fail;
    }

    memset(&client_addr, 0, sizeof(client_addr));
    client_addr.sin_family = AF_INET;
    client_addr.sin_port = htons(PORT_NUM);
    client_addr.sin_addr.s_addr = INADDR_ANY;

    if (bind(client_fd, (struct sockaddr *)&client_addr, sizeof(client_addr)) == -1) 
	{
		syslog(LOG_ERR, "Error binding socket");
		goto exit_on_fail;
	}

    receive_params = (receive_params_t*)malloc(sizeof(receive_params_t));
    if(receive_params == NULL)
    {
        syslog(LOG_ERR, "Malloc for receiver thread params failed");
        goto exit_on_fail;
    }

    receive_params->sock_fd = client_fd;
    receive_params->sock_addr = client_addr;

    if ((pthread_create(&(receive_params->thread_id), NULL, receive_thread, (void*)receive_params)) != 0)
    {
        syslog(LOG_ERR, "Receiver thread creation failed");
        goto exit_on_fail;
    }

    if ((pthread_create(&(process_thread_id), NULL, process_thread, NULL)) != 0)
    {
        syslog(LOG_ERR, "Receiver thread creation failed");
        goto exit_on_fail;
    }

    syslog(LOG_INFO, "Joining receive_params->thread_id...");
    pthread_join(receive_params->thread_id, NULL);
    syslog(LOG_INFO, "receive_params->thread_id joined successfully.");

    syslog(LOG_INFO, "Joining process_thread_id...");
    pthread_join(process_thread_id, NULL);
    syslog(LOG_INFO, "process_thread_id joined successfully.");

exit_on_fail:
    if (receive_params != NULL)
    {
        free(receive_params);
        receive_params = NULL;
    }
    if (res != NULL) 
    {
        freeaddrinfo(res);
    }
    cleanup();
    exit(EXIT_FAILURE);
}
