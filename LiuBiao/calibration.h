#pragma once
#include <opencv2/opencv.hpp>
#include <string>
#include <fstream>
#include <iostream>

using namespace std;
using namespace cv;

class CCalibration
{
public:
    CCalibration(string patternImgPath, string CalibResultPath, Size boardSize, Size squre_size)
    {
        this->patternImgPath = patternImgPath;
        this->calibResultPath = CalibResultPath;
        this->boardSize = boardSize;
        this->squre_size = squre_size;
    }

private:
	vector<Point3f> singlePatternPoints;    //将角点的三维坐标保存到容器
    vector<Mat> patternImgList;             //把读取到的标定图保存到容器
    int imgHeight;
    int imgWidth;
    int imgNum;
    string patternImgPath;       //标定图读取路径
    string calibResultPath;      //标定结果保存路径
    Size boardSize;              //棋盘格每列、每行的角点数
    Size squre_size;             //真实三维世界中每个棋盘格的物理尺寸（mm)
    Mat camK;                    //内参矩阵
    Mat camDiscoeff;             //畸变矩阵
    vector<Vec3d> rotation;      //旋转向量
    vector<Vec3d> translation;   //平移向量

private:
	bool testCorners(vector<Point2f>& corners, int patternWidth, int patternHeight);
	void init3DPoints(Size boardSize, Size squareSize, vector<Point3f> &singlePatternPoint);
    int evaluateCalibrationResult(vector<vector<Point3f>> objectPoints, vector<vector<Point2f>> cornerSquare, vector<Vec3d> _rvec,
								  vector<Vec3d> _tvec, Mat _K, Mat _D, int count);
public:
    bool readPatternImg();
    void calibProcess();
    bool writeParams();
    void run();
    
    ~CCalibration(){}
};
