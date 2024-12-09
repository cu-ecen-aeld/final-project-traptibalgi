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
#include <wiringPi.h>
#include <thread>

#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

#define PORT_NUM (9000)
#define SERVER_IP "127.0.0.1"//"127.0.0.1" //"10.0.0.127"
#define ERROR (-1)
#define HRES (640)
#define VRES (480)
#define FRAME_SIZE (HRES * VRES * 3)
#define NUM_THREADS (2)
#define GREEN_LED_PIN (0) // GPIO17
#define RED_LED_PIN (1) // GPIO18
#define YELLOW_LED_PIN (3) // GPIO22

using namespace cv;
using namespace std;

enum TrafficLightColor {
    RED,
    YELLOW,
    GREEN
};

pthread_mutex_t frame_mutex;

/* The structure for the receiver thread*/
typedef struct receiver_thread_params
{
    pthread_t thread_id;
    int client_fd;
} receiver_thread_params_t;

/* The structure for the process thread*/
typedef struct process_thread_params
{
    pthread_t thread_id;
} process_thread_params_t;

volatile unsigned caught_signal = 0;

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

std::vector<uchar> buffer(FRAME_SIZE);

void *receiver_thread(void *receiver_thread_params_struct)
{
    syslog(LOG_DEBUG, "receiver_thread");

    receiver_thread_params_t *receiver_thread_params = (receiver_thread_params_t*)receiver_thread_params_struct;
    if (receiver_thread_params == NULL)
    {
        syslog(LOG_ERR, "receiver_thread: Receiver thread struct is NULL");
        return NULL;
    }

    while (!caught_signal) 
    {
        size_t total_received = 0;

        if (pthread_mutex_lock(&frame_mutex) != 0)
        {
            syslog(LOG_ERR, "receiver_thread: Failed to lock mutex");
            continue;
        }

        while (total_received < FRAME_SIZE) 
        {
            ssize_t length = recv(receiver_thread_params->client_fd, buffer.data() + total_received, FRAME_SIZE - total_received, 0);
            if (length <= 0) 
            {
                syslog(LOG_ERR, "recv failed: %s", strerror(errno));
                return NULL;
            }
            total_received += length;
        }

        pthread_mutex_unlock(&frame_mutex);

        syslog(LOG_DEBUG, "Receiver thread added frame to CB");
    }

    pthread_exit(nullptr);
}

void* process_thread(void *arg) {

    syslog(LOG_DEBUG, "process_thread");

    receiver_thread_params_t *data = (receiver_thread_params_t*)arg;
    if (data == NULL)
    {
        syslog(LOG_ERR, "receiver_thread: Receiver thread struct is NULL");
        return NULL;
    }

    while (!caught_signal) {

        syslog(LOG_DEBUG, "process_loop");

        if (pthread_mutex_lock(&frame_mutex) != 0) {
            syslog(LOG_ERR, "process_thread: Failed to lock mutex");
            continue;
        }

        // Process the frame data
        Mat frame(VRES, HRES, CV_8UC3, buffer.data());
        
        pthread_mutex_unlock(&frame_mutex);

        if (frame.data == nullptr) {
            syslog(LOG_ERR, "Process thread: Frame data is null");
            continue;
        }

        if (frame.empty()) {
            syslog(LOG_ERR, "Processor thread [%d]: empty frame", data->thread_id);
            continue;
        }

        Mat display_frame;
        cvtColor(frame, display_frame, COLOR_BGR2RGB);
        detect(display_frame);
    }

    pthread_exit(nullptr);
}

int main()
{
    openlog("client", LOG_PID | LOG_CONS, LOG_USER);
    pthread_t threads[NUM_THREADS];
    receiver_thread_params_t *receiver_thread_params = NULL;
    process_thread_params_t *process_thread_params = NULL;
    int sockfd;
    struct sockaddr_in server_addr;
    struct sigaction new_action;
    memset(&new_action, 0, sizeof(struct sigaction));
    new_action.sa_handler = signal_handler;
    /* Create a mutex for synchronising writes to tmp_file*/
    if(pthread_mutex_init(&frame_mutex, NULL) != 0)
    {
        syslog(LOG_ERR, "Creating mutex failed");
        exit(EXIT_FAILURE);
    }
    
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
        goto exit_on_fail;
    }

    if (connect(sockfd, (struct sockaddr *)&server_addr, sizeof(server_addr)) == -1)
    {
        syslog(LOG_ERR, "Connection to server failed: %s", strerror(errno));
        goto exit_on_fail;
    }

    syslog(LOG_INFO, "Connected to server at %s:%d", SERVER_IP, PORT_NUM);

    receiver_thread_params = (receiver_thread_params_t*)malloc(sizeof(receiver_thread_params_t));
    if(receiver_thread_params == NULL)
    {
        syslog(LOG_ERR, "Malloc for receiver thread params failed");
        goto exit_on_fail;
    }

    receiver_thread_params->client_fd = sockfd;

    if ((pthread_create(&(receiver_thread_params->thread_id), NULL, receiver_thread, (void*)receiver_thread_params)) != 0)
    {
        syslog(LOG_ERR, "Receiver thread creation failed");
        goto exit_on_fail;
    }

    process_thread_params= (process_thread_params_t*)malloc(sizeof(process_thread_params_t));
    if(process_thread_params == NULL)
    {
        syslog(LOG_ERR, "Malloc for process thread params failed");
        goto exit_on_fail;
    }

    if ((pthread_create(&(process_thread_params->thread_id), NULL, process_thread, (void*)process_thread_params)) != 0)
    {
        syslog(LOG_ERR, "Process thread creation failed");
        goto exit_on_fail;
    }

    syslog(LOG_DEBUG, "Size of receiver_thread_params_t: %zu", sizeof(receiver_thread_params_t));
    syslog(LOG_DEBUG, "Size of process_thread_params_t: %zu", sizeof(process_thread_params_t));

    pthread_join(process_thread_params->thread_id, NULL);
    pthread_join(process_thread_params->thread_id, NULL);

exit_on_fail:
    if (receiver_thread_params != NULL)
    {
        free(receiver_thread_params);
        receiver_thread_params = NULL;
    }
    if (process_thread_params != NULL)
    {
        free(process_thread_params);
        process_thread_params = NULL;
    }
    close(sockfd);
    closelog();
    exit(EXIT_FAILURE);
}
