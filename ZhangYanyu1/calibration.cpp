#include <opencv2/opencv.hpp>
#include <iostream>
using namespace std;
using namespace cv;

int main(){
    vector<string> filename;
    vector<Mat> images;
    glob("/home/lx/ZhangYanyu1/image/IMG_*.jpg",filename,false);//存入图像文件名
    size_t count = filename.size();
    // cout<<count;
    Size image_size;
    Size board_size = Size(12, 8);//标定板行，列上的角点数
    vector<Point2f> points;
    vector<vector<Point2f>> all_points;
    for(int i = 0; i < count; i++){
        images.push_back(imread(filename[i]));//存入图像
        // imshow("p",images[i]);
        //读取图像宽高
        if(i==0){
            image_size.width = images[i].cols;
            image_size.height = images[i].rows;
        }
        //提取角点
        if(findChessboardCorners(images[i],board_size,points)==0){
            cout<<"could not find chessboard corners./n";
            return -1;
        }
        else{
            Mat gray_image;
            cvtColor(images[i],gray_image,COLOR_BGR2GRAY);
            find4QuadCornerSubpix(gray_image,points,Size(5,5));//亚像素精确化
            all_points.push_back(points);
            drawChessboardCorners(gray_image,board_size,points,false);//标出角点
            imshow("calibration",gray_image);
            waitKey(500);
        }
        //开始标定
        Size square_size = Size(15, 15);
        vector<vector<Point3f>> object_points;

        Mat cameraMatrix = Mat(3,3,CV_32FC1,Scalar::all(0));
        Mat distCoeffs = Mat(1,5,CV_32FC1,Scalar::all(0));
        vector<Mat> tvecs;
        vector<Mat> rvecs;
        //初始化角点三维坐标
        for(int i = 0; i < images.size(); i++){
            vector<Point3f> tempPointSet;
            for(int j = 0; j < board_size.height; j++){
                for(int t = 0; t < board_size.width; t++){
                    Point3f realPoint;
                    realPoint.x = j*square_size.width;
                    realPoint.y = t*square_size.height;
                    realPoint.z = 0;
                    tempPointSet.push_back(realPoint);
                }
            }
            object_points.push_back(tempPointSet);
        }
        //标定
        calibrateCamera(object_points,all_points,image_size,cameraMatrix,distCoeffs,rvecs,tvecs,0);

        
        //显示标定结果  	
        cout<<"标定结果："<<endl;       
        Mat rotation_matrix = Mat(3,3,CV_32FC1, Scalar::all(0)); /* 保存每幅图像的旋转矩阵 */
        cout<<"相机内参数矩阵："<<endl;   
        cout<<cameraMatrix<<endl;   
        cout<<"畸变系数："<<endl;   
        cout<<distCoeffs<<endl;   
        for (int i=0; i<images.size(); i++) 
        { 
            cout<<"第"<<i+1<<"幅图像的旋转向量："<<endl;   
            cout<<tvecs[i]<<endl;    
            /* 将旋转向量转换为相对应的旋转矩阵 */   
            Rodrigues(tvecs[i],rotation_matrix);   
            cout<<"第"<<i+1<<"幅图像的旋转矩阵："<<endl;   
            cout<<rotation_matrix<<endl;   
            cout<<"第"<<i+1<<"幅图像的平移向量："<<endl;   
            cout<<rvecs[i]<<endl<<endl;   
        }   
        waitKey(500);
    }
    

    return 0;
    
}