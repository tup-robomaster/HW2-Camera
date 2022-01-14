#include <iostream>
#include <opencv2/opencv.hpp>
#include <fstream>

using namespace std;
using namespace cv;
//这个程序还处于半成品的状态主要是Ubuntu下图像的循环输入的问题
//现在使用的方式是将图片的文件名写到一个txt文件里，在从txt中逐行提取文件名作为字符串输入到imread中

int main()
{
    //********************读取图像，获取图像基本信息(图片分辨率相同)

    ifstream fin("/home/poq233/Desktop/chess/chess.txt");//标定所用图像文件的路径
    ofstream fout("/home/poq233/Desktop/chess/result.txt");//保存标定结果
    int image_count = 0;//图像的数量
    Size image_size;//图像的尺寸
    Size board_size = Size(11,9);//标定板上每行、列的内角点数
    vector<Point2f> points_location;//用于存储检测到的内角点图像坐标位置
    vector<vector<Point2f>> points;//保存检测到的所有角点
    string filename;//图像名
    int count = 0;//保存角点数
    while(getline(fin,filename))//从文本文档中依次读取待标定图片名(getline 是按行读取，一行为一个文件名称)
    {
        
        cout<<"图片数目为 = "<<image_count<<endl;

        Mat imageInput = imread(filename);//依次读取当前目录下图片

        if(image_count != 0)//读入第一张图片时获取图像宽高信息
        {
            image_size.width = imageInput.cols;
            image_size.height = imageInput.rows;

            cout<<"图片宽为: "<<image_size.width<<endl;
            cout<<"图片高为: "<<image_size.height<<endl;
        }
        
        //********************提取标定板的内角点
        cout<<"寻找角点中";
        if(0 == findChessboardCorners(imageInput,board_size,points_location))
        {
            cout<<"无法找到角点!\n";
            return 0;
        }//没有找到角点就退出
        else
        {
            Mat gray;
            cvtColor(imageInput,gray,COLOR_RGB2GRAY);//转灰度图
            find4QuadCornerSubpix(gray,points_location,Size(5,5));
            //亚像素精确化
            //亚像素精确化是指用一个像素来表示一个点不能满足精确化的要求时，用一个比像素还小的单位来标点
            count += points_location.size();
            points.push_back(points_location);//保存亚像素角点
            //.push_back 用于给字符串或vector类型在最后加入一个新的数据
            drawChessboardCorners(gray,board_size,points_location,false);
            //opencv内置的棋盘格标定角点函数
            imshow("Camera Calibration",imageInput);//显示图片
            waitKey(500);//暂停0.5S
        }
        cout << "角点数目为:" << count<< endl;//显示角点累加后的值
        image_count++;
    }

    int total = points.size();//图片总数  一份角点一张图
    cout << "图片数目为:" << total << endl;
    int CornerNum = board_size.width*board_size.height;//每张图片上总的内角点数

    for(int ii = 0;ii<total;ii++)
    {
        cout << "第" << ii + 1 << "张图片的数据:" << endl;
        for(int jj = 0;jj<CornerNum;jj++)
        {
            if (0 == jj % 3)
                cout << endl;//每三个角点坐标之后换行
            else
                cout.width(10);//输出格式控制，设置域宽为10个字符

            cout << "（" << points[ii][jj].x << "，" << points[ii][jj].y << ")";
        }
        cout << endl;
    }

    cout<<"角点提取完成!\n";

    //*******************开始标定
    cout<<"开始标定";

    Size square_size = Size(10,10);//设置棋盘格子的实际边长，单位为mm
    vector<vector<Point3f>> object_points;//保存标定板上角点的三维坐标
    Mat cameraMatrix = Mat(3,3,CV_32FC1,Scalar::all(0));//相机内参数矩阵
    vector<int> point_counts;//每幅图像中角点的数量
    Mat distCoeffs = Mat(1,5,CV_32FC1,Scalar::all(0));//摄像机的5个畸变系数：k1,k2,k3,p1,p2
    vector<Mat> tvecsMat;//每幅图像的平移向量
    vector<Mat> rvecsMat;//每幅图像的旋转向量

    //初始化标定板上角点的三维坐标
    int i,j,t;
    for(t = 0;t<image_count;t++)
    {
        vector<Point3f> temPointSet;
        for(i = 0;i<board_size.height;i++)
        {
            for(j = 0;j<board_size.width;j++)
            {
                Point3f realPoint;

                //假设标定板放在世界坐标系中的z = 0平面上
                //需要依据棋盘上单个黑白矩阵的大小，计算出（初始化）每一个内角点的世界坐标
                realPoint.x = i*square_size.width;
                realPoint.y = j*square_size.height;
                realPoint.z = 0;
                temPointSet.push_back(realPoint);

            }
        }
        object_points.push_back(temPointSet);
    }

    //初始化每幅图像中的角点数量，假定每幅图像中都可以看到完整的标定板
    for(i = 0;i<image_count;i++)
    {
        point_counts.push_back(board_size.width*board_size.height);
    }

    calibrateCamera(object_points,points,image_size,cameraMatrix,distCoeffs,rvecsMat,tvecsMat,0);
    /*  object_points 世界坐标系中角点的三维坐标，image_points_seq 每个内角点对应的图像坐标点
        image_size 图像的像素尺寸大小，cameraMatrix 输出，内参数矩阵，distCoeffs 输出，畸变系数
        rvecsMat 输出，旋转向量，tvecsMat 输出，位移向量，0标定时采用的算法
        在使用该函数进行标定运算之前，需要对棋盘上每一个内角点的空间坐标系的位置坐标进行初始化，
        标定的结果是生成相机的内参数矩阵cameraMatrix、相机的5个畸变系数distCoeffs，另外每张图像
        都会生成属于自己的平移向量和旋转向量
    */
    cout<<"标定完成"<<endl;

    cout<<"输出标定结果"<<endl;
    double total_err = 0.0; //所有图像的平均误差的总和，初始化为0.0
    double err = 0.0; //每幅图像的平均误差
    vector<Point2f> image_points2; //保存重新计算得到的投影点

    cout<<"每幅图像的标定误差:\n";
    fout<<"每幅图像的标定误差:\n";

    for(i = 0;i<image_count;i++)
    {
        vector<Point3f> tempPointSet = object_points[i];//取出每幅图像角点的三维空间坐标
        projectPoints(tempPointSet,rvecsMat[i],tvecsMat[i],
            cameraMatrix,distCoeffs,image_points2);
        //通过得到的摄像机内外参数，对空间的三维点进行重新投影计算，得到新的投影点

        vector<Point2f> tempImagePoint = points[i];//原来每幅图像中角点图像坐标
        Mat tempImagePointMat = Mat(1,tempImagePoint.size(),CV_32FC2);
        //用于将原图像坐标点存储成一行多列的Mat，由两个通道浮点型数据构成
        Mat image_points2Mat = Mat(1,image_points2.size(),CV_32FC2);
        //用于将重投影坐标点存储成一行多列的Mat，以便于计算重投影误差

        for(int j = 0;j <tempImagePoint.size(); j++)
        {
            image_points2Mat.at<Vec2f>(0,j) = Vec2f(image_points2[j].x,image_points2[j].y);
            tempImagePointMat.at<Vec2f>(0,j) = Vec2f(tempImagePoint[j].x,tempImagePoint[j].y);
        }//赋值
        //Vec2f表示的是2通道float类型的Vector,mat.at<Vec2f>(y, x)是访问图像的一种方式

        err = norm(image_points2Mat,tempImagePointMat,NORM_L2);
        //计算每张图片重投影坐标和亚像素角点坐标之间的偏差
        total_err += err /= point_counts[i];//累加误差
        cout<<"第"<<i+1<<"幅图像的平均误差:"<<err<<"像素"<<endl;
        fout<<"第"<<i+1<<"幅图像的平均误差:"<<err<<"像素"<<endl;

    }

    cout<<"总体平均误差:"<<total_err/image_count<<"像素"<<endl;
    fout<<"总体平均误差:"<<total_err/image_count<<"像素"<<endl<<endl;
    cout<<"评价完成"<<endl;

    cout<<"保存标定结果"<<endl;
    Mat rotation_matrix = Mat(3,3,CV_32FC1,Scalar::all(0));//保存每幅图像的旋转矩阵
    fout<<"相机内参数矩阵:"<<endl;
    fout<<cameraMatrix<<endl<<endl;
    fout<<"畸变系数:\n";
    fout<<distCoeffs<<endl<<endl<<endl;

    for(i = 0; i<image_count;i++)
    {
        fout<<"第"<<i+1<<"幅图像的旋转向量:"<< endl;
        fout<<rvecsMat[i]<<endl;
        Rodrigues(rvecsMat[i],rotation_matrix);//将旋转向量转换为相应的旋转矩阵
        fout<<"第"<<i+1<<"幅图像的旋转矩阵:"<< endl;
        fout<<rotation_matrix<< endl << endl;
        fout<<"第"<<i+1<<"幅图像的平移向量:"<<  endl;
        fout<<tvecsMat[i]<< endl <<endl;

    }
    cout<<"完成保存"<< endl;
    fout<< endl;


    //********************显示标定结果
    Mat mapx = Mat(image_size,CV_32FC1);//输出的X坐标重映射参数
    Mat mapy = Mat(image_size,CV_32FC1);//输出的Y坐标重映射参数
    Mat R = Mat::eye(3,3,CV_32F);
    cout<<"矫正并保存图像"<<endl;
    /*  计算畸变有三种函数  undistort  initUndistortRectifyMap和remap  undistortPoints
        第一种undistort直接计算失真并在图像上进行更改
        第二种initUndistortRectifyMap先计算原始图像坐标，再用remap将该图像对应像素移至该坐标点
        第三种undistortPoints根据已得到的像素点计算原图上的像素点
    */

    string imageFileName;
    std::stringstream StrStm;
    for(int i = 0;i < image_count;i++)
    {
        cout<<"Frame # "<<i+1<<"....."<<endl;
        initUndistortRectifyMap(cameraMatrix, distCoeffs, R, cameraMatrix,image_size, CV_32FC1, mapx, mapy);
        StrStm.clear();//清除缓存
        imageFileName.clear();
        string filePath = "chess";
        StrStm<<i+1;
        StrStm>>imageFileName;
        filePath += imageFileName;
        filePath += ".bmp";
        //获取图片路径
        Mat imageSource = imread(filePath);//读取图像
        Mat newimage = imageSource.clone();//拷贝图像

        remap(imageSource,newimage,mapx,mapy,INTER_LINEAR);//把求得的映射应用到图像上
        imageFileName += "_d.jpg";//矫正后图片命名
        imwrite(imageFileName,newimage);//保存矫正后的图片
    }
    cout<<"保存已结束"<<endl;

    fin.close();
    fout.close();//关闭文件，不再进行操作
    return 0;
}