#!/bin/bash
./v4l2-mmal-uvc -v /dev/video0 -u /dev/video1 -n3 -r0
./v4l2-mmal-uvc -v /dev/video0 -u /dev/video1 -zUYVY -b1500000 -n3 -r1 -f1
