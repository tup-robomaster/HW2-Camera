#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2\imgproc\types_c.h>
#include <iostream>
#include <vector>
#include <fstream>
#include <string>

using namespace std;
using namespace cv;

class cam_sign
{
public:
    cam_sign(){}
    void angle_point();
    void cam_mark();
    void save();
public:
    vector<string> imgList;//代标定图片列表
    Size image_size;//保存图片大小
    Size pattern_size = Size(4, 6);//标定板上每行、每列的角点数；测试图片中的标定板上内角点数为4*6
    vector<vector<Point2f>> corner_points_of_all_imgs;
    int image_num = 0;
    Mat cameraMatrix = Mat(3, 3, CV_32FC1, Scalar::all(0));//内外参矩阵，H——单应性矩阵
    Mat distCoefficients = Mat(1, 5, CV_32FC1, Scalar::all(0));//摄像机的5个畸变系数：k1,k2,p1,p2,k3
};

ifstream inImgPath("calibdata.txt");    //标定所用图像文件的路径
ofstream fout("caliberation_result.txt");   //保存标定结果的文件

void cam_sign::angle_point()
{
    cout << "开始提取角点......" << endl;
    vector<Point2f> corner_points_buf;//建一个数组缓存检测到的角点，通常采用Point2f形式
    string filename;
    while (image_num < imgList.size())
    {
        filename = imgList[image_num++];
        cout << "image_num = " << image_num << endl;
        cout << filename.c_str() << endl;
        Mat imageInput = imread(filename.c_str());
        if (image_num == 1)
        {
            image_size.width = imageInput.cols;
            image_size.height = imageInput.rows;
            cout << "image_size.width = " << image_size.width << endl;
            cout << "image_size.height = " << image_size.height << endl;
        }

        if (findChessboardCorners(imageInput, pattern_size, corner_points_buf) == 0)
        {
            cout << "can not find chessboard corners!\n";   //找不到角点
            exit(1);
        }
        else
        {
            Mat gray;
            cvtColor(imageInput, gray, CV_RGB2GRAY);
            //亚像素精确化
            find4QuadCornerSubpix(gray, corner_points_buf, Size(5, 5));
            corner_points_of_all_imgs.push_back(corner_points_buf);
            drawChessboardCorners(gray, pattern_size, corner_points_buf, true);//标记角点
            imshow("camera calibration", gray);
            waitKey(100);
        }
    }

    int total = corner_points_of_all_imgs.size();
    cout << "total=" << total << endl;
    int cornerNum = pattern_size.width * pattern_size.height;//每张图片上的总的角点数
    for (int i = 0; i < total; i++)
    {
        cout << "--> 第" << i + 1 << "幅图片的数据 -->:" << endl;
        for (int j = 0; j < cornerNum; j++)
        {
            cout << "-->" << corner_points_of_all_imgs[i][j].x;
            cout << "-->" << corner_points_of_all_imgs[i][j].y;
            if ((j + 1) % 3 == 0)
            {
                cout << endl;
            }
            else
            {
                cout.width(10);
            }
        }
        cout << endl;
    }

    cout << endl << "角点提取完成" << endl;
}

void cam_sign::cam_mark()
{
    //摄像机标定
    cout << "开始标定………………" << endl;
    vector<Mat> tvecsMat;//每幅图像的平移向量，t
    vector<Mat> rvecsMat;//每幅图像的旋转向量（罗德里格旋转向量）
    vector<vector<Point3f>> objectPoints;//保存所有图片的角点的三维坐标
                                             //初始化每一张图片中标定板上角点的三维坐标
    int i, j, k;
    for (k = 0; k < image_num; k++)//遍历每一张图片
    {
        vector<Point3f> tempCornerPoints;//每一幅图片对应的角点数组
        //遍历所有的角点
        for (i = 0; i < pattern_size.height; i++)
        {
            for (j = 0; j < pattern_size.width; j++)
            {
                Point3f singleRealPoint;//一个角点的坐标
                singleRealPoint.x = i * 10;
                singleRealPoint.y = j * 10;
                singleRealPoint.z = 0;//假设z=0
                tempCornerPoints.push_back(singleRealPoint);
            }
        }
        objectPoints.push_back(tempCornerPoints);
    }

    calibrateCamera(objectPoints, corner_points_of_all_imgs, image_size, cameraMatrix, distCoefficients, rvecsMat, tvecsMat, 0);
    cout << "标定完成" << endl;
    //开始保存标定结果
    cout << "开始保存标定结果" << endl;
    cout << endl << "相机相关参数：" << endl;
    fout << "相机相关参数：" << endl;
    cout << "1.内外参数矩阵:" << endl;
    fout << "1.内外参数矩阵:" << endl;
    cout << "大小：" << cameraMatrix.size() << endl;
    fout << "大小：" << cameraMatrix.size() << endl;
    cout << cameraMatrix << endl;
    fout << cameraMatrix << endl;

    cout << "2.畸变系数：" << endl;
    fout << "2.畸变系数：" << endl;
    cout << "大小：" << distCoefficients.size() << endl;
    fout << "大小：" << distCoefficients.size() << endl;
    cout << distCoefficients << endl;
    fout << distCoefficients << endl;

    cout << endl << "图像相关参数：" << endl;
    fout << endl << "图像相关参数：" << endl;
    Mat rotation_Matrix = Mat(3, 3, CV_32FC1, Scalar::all(0));//旋转矩阵
    for (i = 0; i < image_num; i++)
    {
        cout << "第" << i + 1 << "幅图像的旋转向量：" << endl;
        fout << "第" << i + 1 << "幅图像的旋转向量：" << endl;
        cout << rvecsMat[i] << endl;
        fout << rvecsMat[i] << endl;
        cout << "第" << i + 1 << "幅图像的旋转矩阵：" << endl;
        fout << "第" << i + 1 << "幅图像的旋转矩阵：" << endl;
        Rodrigues(rvecsMat[i], rotation_Matrix);//将旋转向量转换为相对应的旋转矩阵
        cout << rotation_Matrix << endl;
        fout << rotation_Matrix << endl;
        cout << "第" << i + 1 << "幅图像的平移向量：" << endl;
        fout << "第" << i + 1 << "幅图像的平移向量：" << endl;
        cout << tvecsMat[i] << endl;
        fout << tvecsMat[i] << endl;
    }
    cout << "结果保存完毕" << endl;
}
void cam_sign::save()
{
    Mat mapx = Mat(image_size, CV_32FC1);
    Mat mapy = Mat(image_size, CV_32FC1);
    Mat R = Mat::eye(3, 3, CV_32F);
    cout << "保存矫正图像" << endl;
    string imageFileName;
    std::stringstream StrStm;
    for (int i = 0; i < image_num; i++)
    {
        cout << "Frame #" << i + 1 << endl;
        initUndistortRectifyMap(cameraMatrix, distCoefficients, R, cameraMatrix, image_size, CV_32FC1, mapx, mapy);
        Mat src_image = imread(imgList[i].c_str(), 1);
        Mat new_image = src_image.clone();
        remap(src_image, new_image, mapx, mapy, INTER_LINEAR);
        imshow("原始图像", src_image);
        imshow("矫正后图像", new_image);

        StrStm.clear();
        imageFileName.clear();
        StrStm << i + 1;
        StrStm >> imageFileName;
        imageFileName += "_d.jpg";
        imwrite(imageFileName, new_image);

        waitKey(200);
    }
    cout << "保存结束" << endl;
}
