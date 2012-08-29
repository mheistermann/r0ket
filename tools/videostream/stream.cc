/*
 * ----------------------------------------------------------------------------
 * "THE BEER-WARE LICENSE" (Revision 42):
 * <mh at sponc dot de> wrote this file. As long as you retain this notice you
 * can do whatever you want with this stuff. If we meet some day, and you think
 * this stuff is worth it, you can buy me a beer in return - Martin Heistermann
 * ----------------------------------------------------------------------------
 */

/* Experimental video streaming via USB, r0ket-side code is in
 * r0ket/firmware/applications/videostream.c
 *
 * We do some preprocessing to make the video recognizable in monochrome
 * and then send it via usb serial using a stupid, yet working protocol
 */

#include <iostream>
#include <cstdio>
#include "cv.h"
#include "highgui.h"

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

using namespace cv;
using namespace std;

#define RESX 96
#define RESY 68
#define RESY_B 9
const char serial_filename[] =  "/dev/ttyACM0";


const Size target_size(RESX, RESY);
#define BUFLEN (RESX * RESY_B)
uint8_t outbuf[BUFLEN];
#define PART_SIZE 60
struct {
    uint8_t magic[3];
    uint8_t part;
    uint8_t data[PART_SIZE];
}  __attribute__((packed)) sendbuf;


int serial_fd = -1;

int initSerial() {
    struct termios tio;

    serial_fd = open(serial_filename, O_RDWR);
    if (serial_fd == -1) {
        perror("open");
        return -1;
    }
    memset(&tio, 0, sizeof(tio));
    cfmakeraw(&tio);
    cfsetspeed(&tio, B115200);
    if (-1 == tcsetattr(serial_fd, TCSANOW, &tio)) {
        perror("tcsetattr");
        return -1;
    }

    memcpy(&sendbuf.magic, "MXN", 3);
}

void sendFrame (Mat &frame) {
    memset(outbuf, 0, sizeof(outbuf));
    for (int y=0; y < RESY; y++) {
        for (int x=0; x < RESX; x++) {
            if (frame.at<uchar>(y, x) == 0xff) {
                int y_byte = (RESY-(y+1)) / 8;
                int y_off = (RESY-(y+1)) % 8;
                outbuf[y_byte*RESX+(RESX-(x+1))] |= (1 << y_off);
            }
        }
    }

    uint8_t part = 0;
    while (part * PART_SIZE < BUFLEN) {
        int offset = part * PART_SIZE;
        sendbuf.part = part;
        int to_send = min(BUFLEN - offset, PART_SIZE);
        memcpy(sendbuf.data, &outbuf[offset], to_send);
        write(serial_fd, (char*)&sendbuf, sizeof(sendbuf));
        part += 1;
        //usleep(10*1000);
    }
    //cout << "frame done" << endl;
}

void processFrame (Mat &frame) {
    Mat small, grey, blurry;
    resize(frame, small, target_size, 1.0/8, 1.0/8, INTER_AREA);
    cvtColor(small, grey, CV_RGB2GRAY);
    GaussianBlur(grey, blurry, Size(5,5), 0, 0);
    adaptiveThreshold(grey, frame,
            255, // maxValue
            //ADAPTIVE_THRESH_MEAN_C, // adaptiveMethod
            ADAPTIVE_THRESH_GAUSSIAN_C, // adaptiveMethod
            THRESH_BINARY,
            5, // blockSize
            -5);
}

int main(int argc, char** argv) {
    Mat frame, roi;
    Rect roi_rect = Rect(0,0,480,480);

    if(0 != initSerial()) {
        cerr << "cannot open serial port" << endl;
        return 1;
    }
    cout << "serial initalized, opening cam" << endl;

    VideoCapture capture(0);
    if(!capture.isOpened()) {
        cerr << "cannot open cam" << endl;
        return 1;
    }
    cout << "ok, stream starting..." << endl;

    namedWindow("stream_in", 1);
    namedWindow("stream_out", 1);
    for(;;) {
        int key;
        capture >> frame;
        imshow("stream_in", frame);
        processFrame(frame);
        imshow("stream_out", frame);
        sendFrame(frame);

        key = waitKey(3);
        if (key == 27) {
            break;
        }
    }
    return 0;
}

