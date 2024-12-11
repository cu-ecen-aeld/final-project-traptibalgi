/*
 *
 *  Adapted by Trapti Damodar Balgi for use with a custom traffic light detection system on Raspberry Pi.
 *
 *  Originally adapted by Sam Siewert for use with UVC web cameras and Bt878 frame
 *  grabber NTSC cameras to acquire digital video from a source,
 *  time-stamp each frame acquired, save to a PGM or PPM file.
 *
 *  The original code adapted was open source from V4L2 API and had the
 *  following use and incorporation policy:
 * 
 *  This program can be used and distributed without restrictions.
 *  
 *      This program is provided with the V4L2 API
 * see http://linuxtv.org/docs.php for more information
 * 
 * References:
 * 1. https://github.com/cu-ecen-aeld/buildroot-assignments-base/wiki/OpenCV-3-Setup-in-Buildroot
 * 2. https://medium.com/@deepeshdeepakdd2/v4l-a-complete-practical-tutorial-c520f097b590
 * 3. https://libjpeg.sourceforge.net/
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <getopt.h>             /* getopt_long() */
#include <fcntl.h>              /* low-level i/o */
#include <unistd.h>
#include <errno.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>
#include <time.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <errno.h>
#include <signal.h>
#include <syslog.h>
#include <netdb.h>
#include <jpeglib.h>

#define CLEAR(x) memset(&(x), 0, sizeof(x))
#define COLOR_CONVERT
#define HRES 640
#define VRES 480
#define PORT_NUM 9000
#define ERROR (-1)
#define BACKLOG (10)
#define CLIENT_ADDRESS ("172.20.10.7")

static int server_fd;
struct addrinfo *res;  // will point to the results
volatile unsigned caught_signal = 0;
struct sockaddr_in server_addr;

// Format is used by a number of functions, so made as a file global
static struct v4l2_format fmt;

enum io_method 
{
        IO_METHOD_READ,
        IO_METHOD_MMAP,
        IO_METHOD_USERPTR,
};

struct buffer 
{
        void   *start;
        size_t  length;
};

static char            *dev_name;
static enum io_method   io = IO_METHOD_MMAP;
static int              fd = -1;
struct buffer          *buffers;
static unsigned int     n_buffers;
static int              out_buf;
static int              force_format=1;

/**
 * signal_handler - Handles signals received by the program.
 * @signal_number: The number of the signal received.
 */
static void signal_handler (int signal_number)
{
    caught_signal = signal_number;
}

/**
 * errno_exit - Handles errors by printing an error message and exiting.
 * @s: A string that describes the source of the error.
 */
static void errno_exit(const char *s)
{
        fprintf(stderr, "%s error %d, %s\n", s, errno, strerror(errno));
        exit(EXIT_FAILURE);
}

/**
 * xioctl - Wrapper function for ioctl calls to handle EINTR interruptions.
 * @fh: File descriptor for the video device.
 * @request: The ioctl request code.
 * @arg: Pointer to the argument passed to ioctl.
 * 
 * Returns: The return value of the ioctl call.
 */
static int xioctl(int fh, int request, void *arg)
{
        int r;

        do 
        {
            r = ioctl(fh, request, arg);

        } while (-1 == r && EINTR == errno);

        return r;
}

// This is probably the most acceptable conversion from camera YUYV to RGB
//
// Wikipedia has a good discussion on the details of various conversions and cites good references:
// http://en.wikipedia.org/wiki/YUV
//
// Also http://www.fourcc.org/yuv.php
//
// What's not clear without knowing more about the camera in question is how often U & V are sampled compared
// to Y.
//
// E.g. YUV444, which is equivalent to RGB, where both require 3 bytes for each pixel
//      YUV422, which we assume here, where there are 2 bytes for each pixel, with two Y samples for one U & V,
//              or as the name implies, 4Y and 2 UV pairs
//      YUV420, where for every 4 Ys, there is a single UV pair, 1.5 bytes for each pixel or 36 bytes for 24 pixels

/**
 * yuv2rgb - Converts a YUYV color format to RGB.
 * @y: The Y component of a pixel.
 * @u: The U component of a pixel.
 * @v: The V component of a pixel.
 * @r: Pointer to the red component of the output pixel.
 * @g: Pointer to the green component of the output pixel.
 * @b: Pointer to the blue component of the output pixel.
 */
void yuv2rgb(int y, int u, int v, unsigned char *r, unsigned char *g, unsigned char *b)
{
   int r1, g1, b1;

   // replaces floating point coefficients
   int c = y-16, d = u - 128, e = v - 128;       

   // Conversion that avoids floating point
   r1 = (298 * c           + 409 * e + 128) >> 8;
   g1 = (298 * c - 100 * d - 208 * e + 128) >> 8;
   b1 = (298 * c + 516 * d           + 128) >> 8;

   // Computed values may need clipping.
   if (r1 > 255) r1 = 255;
   if (g1 > 255) g1 = 255;
   if (b1 > 255) b1 = 255;

   if (r1 < 0) r1 = 0;
   if (g1 < 0) g1 = 0;
   if (b1 < 0) b1 = 0;

   *r = r1 ;
   *g = g1 ;
   *b = b1 ;
}

/**
 * process_image - Processes an image by converting from YUYV to RGB and compressing to JPEG.
 * @p: Pointer to the input YUYV data.
 * @size: The size of the input data.
 */
static void process_image(const void *p, int size) 
{
    syslog(LOG_DEBUG, "in process_image, size is %d", size);

    struct jpeg_compress_struct cinfo;
    struct jpeg_error_mgr jerr;

    unsigned char rgb_buffer[HRES * VRES * 3];
    int i, j;
    unsigned char *jpeg_buffer = NULL;
    size_t jpeg_size = 0;
    unsigned char *yuyv = (unsigned char *)p;
    unsigned char *rgb = rgb_buffer;

    for (i = 0, j = 0; i < HRES * VRES * 2; i += 4, j += 6)
    {
        int y0 = yuyv[i];
        int u = yuyv[i+1];
        int y1 = yuyv[i+2];
        int v = yuyv[i+3];

        yuv2rgb(y0, u, v, &rgb[j], &rgb[j+1], &rgb[j+2]);
        yuv2rgb(y1, u, v, &rgb[j+3], &rgb[j+4], &rgb[j+5]);
    }

    /* Initialize JPEG compression */
    cinfo.err = jpeg_std_error(&jerr);
    jpeg_create_compress(&cinfo);

    /* Set up memory destination */
    jpeg_mem_dest(&cinfo, &jpeg_buffer, &jpeg_size);

    /* Set image parameters */
    cinfo.image_width = HRES;
    cinfo.image_height = VRES;
    cinfo.input_components = 3;
    cinfo.in_color_space = JCS_RGB;

    /* Set default compression parameters */
    jpeg_set_defaults(&cinfo);
    jpeg_set_quality(&cinfo, 75, TRUE); // Quality range: 0-100

    /* Start compression */
    jpeg_start_compress(&cinfo, TRUE);

    /* Write scanlines */
    while (cinfo.next_scanline < cinfo.image_height) 
    {
        unsigned char *row_pointer[1];
        row_pointer[0] = &rgb_buffer[cinfo.next_scanline * HRES * 3];
        jpeg_write_scanlines(&cinfo, row_pointer, 1);
    }

    /* Finish compression */
    jpeg_finish_compress(&cinfo);
    jpeg_destroy_compress(&cinfo);

    syslog(LOG_DEBUG, "going to send");

    socklen_t server_addr_len = sizeof(server_addr);
    if (sendto(server_fd, jpeg_buffer, jpeg_size, 0, (struct sockaddr *)&server_addr, server_addr_len) < 0) 
    {
        syslog(LOG_ERR, "Send failed: %s", strerror(errno));
        exit(EXIT_FAILURE);
    }

    syslog(LOG_DEBUG, "JPEG frame sent: %lu bytes\n", jpeg_size);

    /* Free the JPEG buffer */
    if (jpeg_buffer) 
    {
        free(jpeg_buffer);
    }
}

/**
 * read_frame - Reads a video frame from the device.
 * 
 * Returns: 1 if a frame was successfully read, 0 otherwise.
 */
static int read_frame(void)
{
    syslog(LOG_DEBUG, "in read_frame");
    struct v4l2_buffer buf;
    unsigned int i;

    CLEAR(buf);

    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;

    /* Dequeue a buffer */
    if (-1 == xioctl(fd, VIDIOC_DQBUF, &buf))
    {
        switch (errno)
        {
            case EAGAIN:
                return 0;

            case EIO:
                /* Could ignore EIO, but drivers should only set for serious errors, although some set for
                    non-fatal errors too.
                    */
                return 0;

            default:
                printf("mmap failure\n");
                errno_exit("VIDIOC_DQBUF");
        }
    }

    assert(buf.index < n_buffers);

    process_image(buffers[buf.index].start, buf.bytesused);

    /* Enqueue the buffer again */
    if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))
            errno_exit("VIDIOC_QBUF");

    return 1;
}

/**
 * mainloop - Main loop that handles reading video frames and processing them.
 */
static void mainloop(void)
{
    syslog(LOG_DEBUG, "in mainloop");
    struct timespec read_delay;
    struct timespec time_error;

    read_delay.tv_sec=0;
    read_delay.tv_nsec=60000;

    while (!caught_signal)
    {
        fd_set fds;
        struct timeval tv;
        int r;

        /* Add the file descriptors to be watched */
        FD_ZERO(&fds);
        FD_SET(fd, &fds);

        /* Timeout. */
        tv.tv_sec = 2;
        tv.tv_usec = 0;

        /* Monitor for ready-to-read */
        r = select(fd + 1, &fds, NULL, NULL, &tv);

        if (-1 == r)
        {
            if (EINTR == errno)
                continue;
            errno_exit("select");
        }

        if (0 == r)
        {
            fprintf(stderr, "select timeout\n");
            exit(EXIT_FAILURE);
        }

        /* Read frame from buffer */
        if (read_frame())
        {
            if(nanosleep(&read_delay, &time_error) != 0)
                perror("nanosleep");
            else
                printf("time_error.tv_sec=%ld, time_error.tv_nsec=%ld\n", time_error.tv_sec, time_error.tv_nsec);

            break;
        }

        /* EAGAIN - continue select loop unless count done. */
    }
}

/**
 * stop_capturing - Stops the video capture process.
 */
static void stop_capturing(void)
{
        enum v4l2_buf_type type;

        switch (io) {
        case IO_METHOD_READ:
                /* Nothing to do. */
                break;

        case IO_METHOD_MMAP:
        case IO_METHOD_USERPTR:
                type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                if (-1 == xioctl(fd, VIDIOC_STREAMOFF, &type))
                        errno_exit("VIDIOC_STREAMOFF");
                break;
        }
}

/**
 * start_capturing - Starts the video capture process.
 */
static void start_capturing(void)
{
        unsigned int i;
        enum v4l2_buf_type type;

        for (i = 0; i < n_buffers; ++i) 
        {
                printf("allocated buffer %d\n", i);
                struct v4l2_buffer buf;

                CLEAR(buf);
                buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                buf.memory = V4L2_MEMORY_MMAP;
                buf.index = i;

                /* Queue the buffers */
                if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))
                        errno_exit("VIDIOC_QBUF");
        }

        /* Starting the capture stream */
        type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        if (-1 == xioctl(fd, VIDIOC_STREAMON, &type))
                errno_exit("VIDIOC_STREAMON");
        
}

/**
 * uninit_device - Uninitializes the video capture device and releases allocated resources.
 */
static void uninit_device(void)
{
        unsigned int i;

        for (i = 0; i < n_buffers; ++i)
                if (-1 == munmap(buffers[i].start, buffers[i].length))
                        errno_exit("munmap");

        free(buffers);
}

/**
 * init_read - Initializes memory buffers for read I/O method.
 * @buffer_size: The size of the buffer to allocate.
 */
static void init_read(unsigned int buffer_size)
{
        buffers = (struct buffer*)calloc(1, sizeof(*buffers));

        if (!buffers) 
        {
                fprintf(stderr, "Out of memory\n");
                exit(EXIT_FAILURE);
        }

        buffers[0].length = buffer_size;
        buffers[0].start = malloc(buffer_size);

        if (!buffers[0].start) 
        {
                fprintf(stderr, "Out of memory\n");
                exit(EXIT_FAILURE);
        }
}

/**
 * init_mmap - Initializes memory-mapped buffers for I/O.
 */
static void init_mmap(void)
{
        struct v4l2_requestbuffers req;

        CLEAR(req);

        req.count = 6;
        req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        req.memory = V4L2_MEMORY_MMAP;

        /* Request for 6 buffers */
        if (-1 == xioctl(fd, VIDIOC_REQBUFS, &req)) 
        {
                if (EINVAL == errno) 
                {
                        fprintf(stderr, "%s does not support "
                                 "memory mapping\n", dev_name);
                        exit(EXIT_FAILURE);
                } else 
                {
                        errno_exit("VIDIOC_REQBUFS");
                }
        }

        /* Need minimum of 2 - current frame and the next one */
        if (req.count < 2) 
        {
                fprintf(stderr, "Insufficient buffer memory on %s\n", dev_name);
                exit(EXIT_FAILURE);
        }

        /* Allocate memory for buffers */
        buffers = (struct buffer*)calloc(req.count, sizeof(*buffers));

        if (!buffers) 
        {
                fprintf(stderr, "Out of memory\n");
                exit(EXIT_FAILURE);
        }

        /* Map video capture buffers from kernel space into user space */
        for (n_buffers = 0; n_buffers < req.count; ++n_buffers) {
                struct v4l2_buffer buf;

                CLEAR(buf);

                buf.type        = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                buf.memory      = V4L2_MEMORY_MMAP;
                buf.index       = n_buffers;

                if (-1 == xioctl(fd, VIDIOC_QUERYBUF, &buf))
                        errno_exit("VIDIOC_QUERYBUF");

                buffers[n_buffers].length = buf.length;
                buffers[n_buffers].start =
                        mmap(NULL /* start anywhere */,
                              buf.length,
                              PROT_READ | PROT_WRITE /* required */,
                              MAP_SHARED /* recommended */,
                              fd, buf.m.offset);

                if (MAP_FAILED == buffers[n_buffers].start)
                        errno_exit("mmap");
        }
}

/**
 * init_device - Initializes the video device, configuring its capabilities and format.
 */
static void init_device(void)
{
    struct v4l2_capability cap;
    struct v4l2_cropcap cropcap;
    struct v4l2_crop crop;
    unsigned int min;

    /* Checks if the device is a valid V4L2 device by querying its capabilities */
    if (-1 == xioctl(fd, VIDIOC_QUERYCAP, &cap))
    {
        if (EINVAL == errno) {
            fprintf(stderr, "%s is no V4L2 device\n",
                     dev_name);
            exit(EXIT_FAILURE);
        }
        else
        {
                errno_exit("VIDIOC_QUERYCAP");
        }
    }

    /* Checks if the device supports video capture */
    if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE))
    {
        fprintf(stderr, "%s is no video capture device\n",
                 dev_name);
        exit(EXIT_FAILURE);
    }

    /* Checks if the device supports streaming I/O */
    if (!(cap.capabilities & V4L2_CAP_STREAMING))
    {
        fprintf(stderr, "%s does not support streaming i/o\n",
                    dev_name);
        exit(EXIT_FAILURE);
    }

    /* Select video input, video standard and tune here. */

    CLEAR(cropcap);

    /* Checks if the device has cropping capabilties */
    cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    if (0 == xioctl(fd, VIDIOC_CROPCAP, &cropcap))
    {
        crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        crop.c = cropcap.defrect; /* reset to default */

        if (-1 == xioctl(fd, VIDIOC_S_CROP, &crop))
        {
            switch (errno)
            {
                case EINVAL:
                    /* Cropping not supported. */
                    break;
                default:
                    /* Errors ignored. */
                        break;
            }
        }

    }
    else
    {
        /* Errors ignored. */
    }

    CLEAR(fmt);

    /* Sets or gets the video format */
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    if (force_format)
    {
        printf("FORCING FORMAT\n");
        fmt.fmt.pix.width       = HRES;
        fmt.fmt.pix.height      = VRES;

        // Specify the Pixel Coding Formate here

        // This one work for Logitech C200
        fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;

        //fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_UYVY;
        //fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_VYUY;

        // Would be nice if camera supported
        //fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_GREY;
        //fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_RGB24;

        //fmt.fmt.pix.field       = V4L2_FIELD_INTERLACED;
        fmt.fmt.pix.field       = V4L2_FIELD_NONE;

        if (-1 == xioctl(fd, VIDIOC_S_FMT, &fmt))
                errno_exit("VIDIOC_S_FMT");

        /* Note VIDIOC_S_FMT may change width and height. */
    }
    else
    {
        printf("ASSUMING FORMAT\n");
        /* Preserve original settings as set by v4l2-ctl for example */
        if (-1 == xioctl(fd, VIDIOC_G_FMT, &fmt))
                    errno_exit("VIDIOC_G_FMT");
    }

    /* Buggy driver paranoia. */
    min = fmt.fmt.pix.width * 2;
    if (fmt.fmt.pix.bytesperline < min)
            fmt.fmt.pix.bytesperline = min;
    min = fmt.fmt.pix.bytesperline * fmt.fmt.pix.height;
    if (fmt.fmt.pix.sizeimage < min)
            fmt.fmt.pix.sizeimage = min;
            
    init_mmap();
}

/**
 * @brief Closes the video device and sets the file descriptor to -1.
 * 
 */
static void close_device(void)
{
        if (-1 == close(fd))
                errno_exit("close");

        fd = -1;
}

/**
 * @brief Opens the video device for reading and sets up its properties.
 * 
 * This function checks if the specified device is a valid character
 * device, opens the device for reading and writing in non-blocking mode,
 * and handles errors appropriately.
 */
static void open_device(void)
{
        struct stat st;

        /* Retrieve information about device */
        if (-1 == stat(dev_name, &st)) {
                fprintf(stderr, "Cannot identify '%s': %d, %s\n",
                         dev_name, errno, strerror(errno));
                exit(EXIT_FAILURE);
        }

        /* Check if a character device */
        if (!S_ISCHR(st.st_mode)) {
                fprintf(stderr, "%s is no device\n", dev_name);
                exit(EXIT_FAILURE);
        }

        /* Open device in non-blocking mode */
        fd = open(dev_name, O_RDWR /* required */ | O_NONBLOCK, 0);

        if (-1 == fd) {
                fprintf(stderr, "Cannot open '%s': %d, %s\n",
                         dev_name, errno, strerror(errno));
                exit(EXIT_FAILURE);
        }
}

/**
 * @brief Cleans up resources used by the program.
 * 
 */

void cleanup() 
{
    stop_capturing();
    uninit_device();
    close_device();
    if (server_fd != -1) 
    {
        shutdown(server_fd, SHUT_RDWR);
        close(server_fd);
    }

    if (res != NULL) 
    {
        freeaddrinfo(res);
    }
    closelog();
}

// static void usage(FILE *fp, int argc, char **argv)
// {
//         fprintf(fp,
//                  "Usage: %s [options]\n\n"
//                  "Version 1.3\n"
//                  "Options:\n"
//                  "-d | --device name   Video device name [%s]\n"
//                  "-h | --help          Print this message\n"
//                  "-m | --mmap          Use memory mapped buffers [default]\n"
//                  "-o | --output        Outputs stream to stdout\n"
//                  "-f | --format        Force format to 640x480 GREY\n"
//                  "-c | --count         Number of frames to grab [%i]\n"
//                  "",
//                  argv[0], dev_name, frame_count);
// }

static const char short_options[] = "d:hmruofc:";

static const struct option
long_options[] = {
        { "device", required_argument, NULL, 'd' },
        { "help",   no_argument,       NULL, 'h' },
        { "mmap",   no_argument,       NULL, 'm' },
        { "output", no_argument,       NULL, 'o' },
        { "format", no_argument,       NULL, 'f' },
        { "count",  required_argument, NULL, 'c' },
        { 0, 0, 0, 0 }
};

/**
 * @brief The main entry point for the server application.
 */
int main(int argc, char **argv)
{
    openlog("server", LOG_PID | LOG_CONS, LOG_USER);

    int status;
    struct addrinfo hints;
    int optval = 1;
    int flags;   

    if (argc < 2)
    {
        fprintf(stderr, "Usage: %s <CLIENT_IP>\n", argv[0]);
        exit(EXIT_FAILURE);
    } 

    const char *client_ip = argv[1];

    if (argc > 2)
        dev_name = argv[2];
    else
        dev_name = "/dev/video0";

    for (;;)
    {
        int idx;
        int c;

        c = getopt_long(argc, argv,
                    short_options, long_options, &idx);

        if (-1 == c)
            break;

        switch (c)
        {
            case 0: /* getopt_long() flag */
                break;

            case 'd':
                dev_name = optarg;
                break;

            case 'h':
                //usage(stdout, argc, argv);
                exit(EXIT_SUCCESS);

            case 'm':
                io = IO_METHOD_MMAP;
                break;

            case 'o':
                out_buf++;
                break;

            case 'f':
                force_format++;
                break;

            case 'c':
                errno = 0;
                //frame_count = strtol(optarg, NULL, 0);
                if (errno)
                        errno_exit(optarg);
                break;

            default:
                //usage(stderr, argc, argv);
                exit(EXIT_FAILURE);
        }
    }

    open_device();
    init_device();
    start_capturing();

    memset(&hints, 0, sizeof hints);    // Make sure the struct is empty
    hints.ai_family = AF_INET;          // IPv4
    hints.ai_socktype = SOCK_DGRAM;    // UDP stream sockets
    hints.ai_flags = AI_PASSIVE;        // Fill in my IP for me

    if ((status = getaddrinfo(NULL, "9000", &hints, &res)) != 0) 
    {
        syslog(LOG_ERR, "getaddrinfo failed");
        goto exit_on_fail;
    }

    /* Create a socket */
    if ((server_fd = socket(res->ai_family, res->ai_socktype, res->ai_protocol)) == -1)
    {
        syslog(LOG_ERR, "Failed to make a socket");
        goto exit_on_fail;
    }

    flags = fcntl(server_fd, F_GETFL, 0);
    fcntl(server_fd, F_SETFL, flags & ~O_NONBLOCK);

    /* Allow reuse of socket */
    if(setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, (const char*)&optval, sizeof(optval)) != 0)
    {
        syslog(LOG_ERR, "Socket reuse failed");
        goto exit_on_fail;
    }
    
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port=htons(PORT_NUM);

    inet_pton(res->ai_family, client_ip, &server_addr.sin_addr);

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

    mainloop();

exit_on_fail:
    cleanup();
    closelog();
    exit(1);
}
