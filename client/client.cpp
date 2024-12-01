#include "/usr/include/opencv4/opencv2/opencv.hpp"
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>

#define PORT_NUM 9000
#define BUF_SIZE 153600

using namespace cv;

int main() 
{
    int sockfd, client_fd;
    struct sockaddr_in server_addr, client_addr;
    char buffer[BUF_SIZE];
    socklen_t addr_len = sizeof(client_addr);

    // Create socket
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) 
    {
        perror("Socket creation failed");
        exit(EXIT_FAILURE);
    }

    // Set up server address
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port = htons(PORT_NUM);

    // Bind socket
    if (bind(sockfd, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) 
    {
        perror("Bind failed");
        close(sockfd);
        exit(EXIT_FAILURE);
    }

    // Listen for incoming connections
    if (listen(sockfd, 5) < 0) 
    {
        perror("Listen failed");
        close(sockfd);
        exit(EXIT_FAILURE);
    }

    printf("Server listening on port %d\n", PORT_NUM);

    // Accept connection
    client_fd = accept(sockfd, (struct sockaddr*)&client_addr, &addr_len);
    if (client_fd < 0) 
    {
        perror("Accept failed");
        close(sockfd);
        exit(EXIT_FAILURE);
    }

    printf("Connection established with %s\n", inet_ntoa(client_addr.sin_addr));

    while (1) {
        // Receive frame data
        ssize_t bytes_received = recv(client_fd, buffer, BUF_SIZE, MSG_WAITALL);
        if (bytes_received <= 0) 
        {
            perror("Receive failed or connection closed");
            break;
        }

        // Process and display the frame
        Mat frame(480, 640, CV_8UC3, buffer);
        if (frame.empty()) 
        {
            fprintf(stderr, "Error: Received empty frame\n");
            continue;
        }

        imshow("Server Frame", frame);
        if (waitKey(1) == 27) 
        {
            printf("Exiting display\n");
            break;
        }
    }

    close(client_fd);
    close(sockfd);
    return 0;
}
