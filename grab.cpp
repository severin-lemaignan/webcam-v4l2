#include <iostream>
#include <fstream>

#include "webcam.h"

#define XRES 640
#define YRES 480

using namespace std;

int main(int argc, char** argv)
{

    Webcam webcam("/dev/video0", XRES, YRES);
    auto frame = webcam.frame();

    ofstream image;
    image.open("frame.ppm");
    image << "P6\n" << XRES << " " << YRES << " 255\n";
    image.write((char *) frame.data, frame.size);
    image.close();

    return 0;

}
