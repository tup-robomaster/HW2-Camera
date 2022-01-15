#include <opencv2/opencv.hpp>
#include <string>
#include <iostream>
#include "calibration.h"

using namespace std;
using namespace cv;

int main()
{
    string patternImgPath = "/home/liubiao/LiuBiao/camera_calibration/data/calibration_picture/";
    string calibResultPath = "/home/liubiao/LiuBiao/camera_calibration/data/calibration_result/";
    string srcImgPath = "/home/liubiao/LiuBiao/camera_calibration/data/test/0.png";

    Size boardSize = Size(6, 4);
    Size squre_size = Size(30, 30);
    CCalibration calibration(patternImgPath, calibResultPath, boardSize, squre_size);
    calibration.run();

    return 0;
}
