#include "opencv2/opencv.hpp"
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <sys/types.h>
#include <stdbool.h>
#include <netdb.h>
#include <fcntl.h>
#include "queue.h"
#include <pthread.h>
#include <time.h>
#include <sys/select.h>
#include <linux/stat.h>
#include <sys/stat.h>
#include <syslog.h>
#include <vector>
#include <signal.h>
#include <pthread.h>

#define PORT_NUM 9000
#define BUF_SIZE 153600
#define ERROR (-1)
#define BACKLOG (10)
#define BUF_INITIAL_SIZE (153600)

using namespace cv;

int sockfd;
struct addrinfo *res;  // will point to the results
volatile unsigned caught_signal = 0;

typedef struct server_thread_params
{
    pthread_t thread_id;
    volatile bool thread_complete;
    int client_fd;
    char client_ip[INET_ADDRSTRLEN];        /* Size for IPv4 addresses */
    SLIST_ENTRY(server_thread_params) link;
} server_thread_params_t;

typedef SLIST_HEAD(socket_head,server_thread_params) head_t;

void cleanup() 
{
    if (sockfd != -1) 
    {
        shutdown(sockfd, SHUT_RDWR);
        close(sockfd);
    }

    if (res != NULL) 
    {
        freeaddrinfo(res);
    }
}

static void signal_handler (int signal_number)
{
    caught_signal = signal_number;
}

int receive_and_process_data(server_thread_params_t *server_params, char *buf, size_t receive_buf_size)
{
    syslog(LOG_DEBUG, "in receive_and_process_data");
    std::vector<Mat> frames;
    int length;
    size_t total_received = 0;
    char *end_packet = NULL;
    int retval = 0;
    
    do 
    {
        length = recv(server_params->client_fd, buf + total_received, receive_buf_size - total_received, 0);
        if (length <= 0) 
        {
            if (length == 0) 
            {
                syslog(LOG_INFO, "Client disconnected.");
            } 
            else 
            {
                syslog(LOG_ERR, "recv failed: %s", strerror(errno));
            }
        return ERROR;
    }

        total_received += length;

    } while (total_received < receive_buf_size);
    
    printf("Total received %d\n",total_received);
    
    if (total_received == receive_buf_size)
    {
        Mat frame(240, 320, CV_8UC3, buf);
                
        if (frame.empty()) 
        {
            fprintf(stderr, "Error: Received empty frame\n");
            return retval;
        }
        
        imshow("Client Display", frame);
        
        char filename[64];
        snprintf(filename, sizeof(filename), "frame_%03zu.jpg");
        imwrite(filename, frame);
    }
    else
        retval = ERROR;

update_exit:
    return retval;
}

void *threadfn_server(void *server_thread_params_struct)
{
    syslog(LOG_DEBUG, "in thread");
    server_thread_params_t *server_params = (server_thread_params_t*)server_thread_params_struct;
    char buf[BUF_SIZE];
    size_t receive_buf_size = BUF_SIZE;

    if (server_params == NULL)
    {
        syslog(LOG_ERR, "Thread server_thread_params is NULL");
        goto threadfn_server_exit;
    }
    
    while (!caught_signal)
    {
        if(receive_and_process_data(server_params, buf, receive_buf_size) != 0)
        {
            syslog(LOG_ERR, "receive_and_process_data failed");
        }
        
        if(waitKey(1) == 27)
        {
            break;
        }
    }

threadfn_cleanup:
    close(server_params->client_fd);
    syslog(LOG_DEBUG, "Closed connection from %s", server_params->client_ip);
    server_params->thread_complete = true;

threadfn_server_exit:
    return NULL;
}

int main() 
{
    openlog("socket", LOG_PID | LOG_CONS, LOG_USER);

    int status;
    socklen_t addr_size;
    struct addrinfo hints;
    struct sockaddr_storage their_addr;
    struct sockaddr_in server_addr;
    server_thread_params_t *iterator = NULL;
    server_thread_params_t *tmp = NULL;
    server_thread_params_t *server_params = NULL;
    int opt = 1;

    /* Initialize the head */
    head_t head;
    SLIST_INIT(&head); 

    memset(&hints, 0, sizeof hints);    // Make sure the struct is empty
    hints.ai_family = AF_INET;          // IPv4
    hints.ai_socktype = SOCK_STREAM;    // TCP stream sockets
    hints.ai_flags = AI_PASSIVE;        // Fill in my IP for me

    if ((status = getaddrinfo(NULL, "9000", &hints, &res)) != 0) 
    {
        syslog(LOG_ERR, "getaddrinfo failed");
        goto exit_on_fail;
    }

    /* Create a socket */
    if ((sockfd = socket(res->ai_family, res->ai_socktype, res->ai_protocol)) == -1)
    {
        syslog(LOG_ERR, "Failed to make a socket");
        goto exit_on_fail;
    }
    
    /* Allow reuse of socket */
    
    struct timeval timeout;
    timeout.tv_sec = 5;  // 5 seconds timeout
    timeout.tv_usec = 0;
    if(setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, (const char*)&timeout, sizeof(timeout)) != 0)
    {
        syslog(LOG_ERR, "Socket reuse failed");
        goto exit_on_fail;
    }
    
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port=htons(PORT_NUM);

    /* Bind it to the port we passed in to getaddrinfo(): */
    if (bind(sockfd, (struct sockaddr*)&server_addr, sizeof(server_addr)) == -1) 
    {
        syslog(LOG_ERR, "bind failed: %s", strerror(errno));
        goto exit_on_fail;
    }

    if (listen(sockfd, BACKLOG) == -1)
    {
        syslog(LOG_ERR, "Listen failed");
        goto exit_on_fail;
    }

    /* Setup signal handlers*/
    struct sigaction new_action;
    memset(&new_action, 0, sizeof(struct sigaction));
    new_action.sa_handler = signal_handler;

    if (sigaction(SIGTERM, &new_action, NULL) != 0)
    {
        syslog(LOG_ERR, "Sigaction for SIGTERM failed");
    }

    if (sigaction(SIGINT, &new_action, NULL))
    {
        syslog(LOG_ERR, "Sigaction for SIGINT failed");
    }

    /* Now accept incoming connections in a loop while signal not caught*/
    while (!caught_signal)
    {
        int new_fd;
        char client_ip[INET_ADDRSTRLEN];     
        addr_size = sizeof their_addr;
        new_fd = accept(sockfd, (struct sockaddr *)&their_addr, &addr_size);
        if (new_fd == -1)
        {
            syslog(LOG_ERR, "Accept failed: %s", strerror(errno));
            continue;
        }

        inet_ntop(their_addr.ss_family, &(((struct sockaddr_in*)&their_addr)->sin_addr), client_ip, sizeof(client_ip));
        syslog(LOG_DEBUG, "Accepted connection from %s", client_ip);

        server_params = (server_thread_params_t*)malloc(sizeof(server_thread_params_t));
        if(server_params == NULL)
        {
            syslog(LOG_ERR, "Malloc for server thread params failed");
            continue;
        }

        server_params->thread_complete = false;
        server_params->client_fd = new_fd;
        strncpy(server_params->client_ip, client_ip, INET_ADDRSTRLEN);
        
        if ((pthread_create(&(server_params->thread_id), NULL, threadfn_server, (void*)server_params)) != 0)
        {
            syslog(LOG_ERR, "Thread creation failed");
            free(server_params);
            server_params = NULL;
            continue;
        }

        /* Add the node to the SLIST*/
        SLIST_INSERT_HEAD(&head, server_params, link);

        /* Attempt to join threads by checking for the complete_thread flag*/
        server_thread_params_t *iterator = NULL;
        server_thread_params_t *tmp = NULL;

        SLIST_FOREACH_SAFE(iterator, &head, link, tmp) 
	    {
            if (iterator->thread_complete == true) 
	        {
                if(pthread_join(iterator->thread_id, NULL) != 0)
		        {
                    syslog(LOG_ERR, "Thread join failed for %ld", iterator->thread_id);
                }
                syslog(LOG_INFO, "Thread joined %ld", iterator->thread_id);

                /* Remove node from the list and free the memory */
                SLIST_REMOVE(&head, iterator, server_thread_params, link);
                free(iterator);
                iterator = NULL;
            }
        }
    }

    /* Cleanup after caught signal */
    /* Server thread */
    SLIST_FOREACH_SAFE(iterator, &head, link, tmp) 
    {
        if(pthread_join(iterator->thread_id, NULL) != 0)
        {
            syslog(LOG_ERR, "Thread join failed for %ld", iterator->thread_id);
        }
        syslog(LOG_INFO, "Thread joined %ld", iterator->thread_id);

        /* Remove node from the list and free the memory */
        SLIST_REMOVE(&head, iterator, server_thread_params, link);
        free(iterator);
        iterator = NULL;
    }

exit_on_fail:
    cleanup();
    closelog();
    exit(1);
}
