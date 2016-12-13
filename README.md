Minimalistic C++ wrapper around V4L2
====================================

Build the demo
--------------

You need the V4L2 development package. On Debian/Ubuntu:
```
$ sudo apt install libv4l2-dev
```

Then:
```
$ g++ -std=c++11 grab.cpp webcam.cpp -ograb -lv4l2
```

Calling `./grab` acquires one image from `/dev/video0` and saves it as
`frame.ppm`.


