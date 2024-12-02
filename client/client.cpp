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

#define PORT_NUM 9000
#define BUF_SIZE 320 * 240 * 3
#define SERVER_IP "10.0.0.202"
#define ERROR (-1)

using namespace cv;

volatile unsigned caught_signal = 0;

static void signal_handler(int signal_number)
{
    caught_signal = signal_number;
}

int receive_and_process_data(int sockfd)
{
    syslog(LOG_DEBUG, "in receive_and_process_data");
    const int FRAME_SIZE = 320 * 240 * 3;
    std::vector<uchar> buffer(FRAME_SIZE);
    int retval = 0;
    int frame_count = 0;

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

            // Convert BGR to RGB
            cvtColor(frame, frame, COLOR_BGR2RGB);

            imshow("Client Display", frame);
            
            /*char filename[64];
            snprintf(filename, sizeof(filename), "frame_%03d.jpg", frame_count++);
            imwrite(filename, frame);*/

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
