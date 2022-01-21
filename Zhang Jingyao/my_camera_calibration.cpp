#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include <cstring>
#include <string>

 
using namespace cv;
using namespace std;

// int ChangeNToZERO(string s){
//     int s_len = s.length();
//     int x = 0;
//     for (int i = s_len - 1; i >= 0; i--){
//         if (i == s_len - 1 && s[i] == '\n'){
//             s[i] = '\0';
//             break;
//         }
//         x = 1;
//         break;
//     }
//     if (x == 0)
//         ChangeNToZERO(s);
//     return 1;
// }
int main(){
	int filenum = 14; //在我电脑里在文本中读取文件名会出错 就不以那种形式读取了
	ofstream fout("/home/zjy/chessboard/caliberation_result.txt");  /* 保存标定结果的文件 */	
	//读取每一幅图像，从中提取出角点，然后对角点进行亚像素精确化	
	cout <<"开始提取角点………………";
	int image_count = 0;  /* 图像数量 */
	Size image_size;  /* 图像的尺寸 */
	Size board_size = Size(4,6);    /* 标定板上每行、列的角点数 */
	vector <Point2f> image_points_buf;  /* 缓存每幅图像上检测到的角点 */
	vector <vector <Point2f> > image_points_seq; /* 保存检测到的所有角点 */
	for(int i = 1 ;i <= 14 ;i++){
		image_count++;		
		// 用于观察检验输出
		cout<<"image_count = "<<image_count<<endl;		
		/* 输出检验*/
		Mat imageInput = imread("/home/zjy/chessboard/chess" + to_string(image_count));
        if(imageInput.empty()){
            cout << "没读到" << endl;
        }
		if (image_count == 1){  //读入第一张图片时获取图像宽高信息
			image_size.width = imageInput.cols;
			image_size.height = imageInput.rows;			
			cout<<"image_size.width = "<<image_size.width<<endl;
			cout<<"image_size.height = "<<image_size.height<<endl;
		}
 
		/* 提取角点 */
		if (findChessboardCorners(imageInput,board_size,image_points_buf) == 0){			
			cout << "没有找到角点!" << endl; //找不到角点
			return 0;
		} 
		else {
			Mat view_gray;
			cvtColor(imageInput,view_gray,COLOR_BGR2GRAY);
			/* 亚像素精确化 */
			find4QuadCornerSubpix(view_gray,image_points_buf,Size(5,5)); //对粗提取的角点进行精确化
			image_points_seq.push_back(image_points_buf);  //保存亚像素角点
			/* 在图像上显示角点位置 */
			drawChessboardCorners(view_gray,board_size,image_points_buf,false); //用于在图片中标记角点
			imshow("Camera Calibration",view_gray);//显示图片
			waitKey(500);//暂停0.5S		
		}
	}
	int total = image_points_seq.size();
	cout << "total = " << total << endl;
	int CornerNum = board_size.width * board_size.height;  //每张图片上总的角点数
	for (int i = 0; i < CornerNum; i++){
        cout << "x= "<< image_points_seq[0][i].x << " ";
        cout << "y= "<< image_points_seq[0][i].y << " " <<endl;
    }
	cout << "角点提取完成！" << endl;
 
	//以下是摄像机标定
	cout << "开始标定………………";
	/*棋盘三维信息*/
	Size square_size = Size(10,10);  /* 实际测量得到的标定板上每个棋盘格的大小 */
	vector< vector <Point3f> > object_points; /* 保存标定板上角点的三维坐标 */
	/*内外参数*/
	Mat cameraMatrix = Mat(3,3,CV_32FC1,Scalar::all(0)); /* 摄像机内参数矩阵 */
	vector <int> point_counts;  // 每幅图像中角点的数量
	Mat distCoeffs = Mat(1,5,CV_32FC1,Scalar::all(0)); /* 摄像机的5个畸变系数：k1,k2,p1,p2,k3 */
	vector <Mat> tvecsMat;  /* 每幅图像的平移向量 */
	vector <Mat> rvecsMat; /* 每幅图像的旋转向量 */
	/* 初始化标定板上角点的三维坐标 */
	for (int t = 0; t < image_count; t++) {
		vector <Point3f> tempPointSet;
		for (int i = 0; i < board_size.height ;i++) {
			for (int j = 0;j < board_size.width ;j++) {
				Point3f realPoint;
				/* 假设标定板放在世界坐标系中z=0的平面上 */
				realPoint.x = i * square_size.width;
				realPoint.y = j * square_size.height;
				realPoint.z = 0;
				tempPointSet.push_back(realPoint);
			}
		}
		object_points.push_back(tempPointSet);
	}
	/* 初始化每幅图像中的角点数量，假定每幅图像中都可以看到完整的标定板 */
	for (int i = 0 ;i < image_count ;i++){
		point_counts.push_back(board_size.width*board_size.height);
	}	
	/* 开始标定 */
	calibrateCamera(object_points,image_points_seq,image_size,cameraMatrix,distCoeffs,rvecsMat,tvecsMat,0);
	cout << "标定完成！" << endl;
	//对标定结果进行评价
	cout << "开始评价标定结果………………" << endl;
	//保存定标结果  	
	cout << "开始保存定标结果………………" << endl;       
	Mat rotation_matrix = Mat(3,3,CV_32FC1, Scalar::all(0)); /* 保存每幅图像的旋转矩阵 */
	fout << "相机内参数矩阵：" << endl;
	cout << "相机内参数矩阵：" << endl;   
	fout << cameraMatrix << endl << endl;
	cout << cameraMatrix << endl << endl;    
	fout << "畸变系数：" << endl;
	cout << "畸变系数：" << endl;   
	fout << distCoeffs << endl << endl << endl;
	cout << distCoeffs << endl << endl << endl;   
	for (int i = 0 ;i < image_count ;i++) { 
		fout << "第" << i+1 << "幅图像的旋转向量：" << endl;
		cout << "第" << i+1 << "幅图像的旋转向量：" << endl;
		fout << rvecsMat[i] << endl; 
		cout << rvecsMat[i] << endl;    
		/* 将旋转向量转换为相对应的旋转矩阵 */   
		Rodrigues(rvecsMat[i],rotation_matrix);   
		fout << "第" << i+1 << "幅图像的旋转矩阵：" << endl;
		cout << "第" << i+1 << "幅图像的旋转矩阵：" << endl;  
		fout << rotation_matrix << endl;
		cout << rotation_matrix << endl;   
		fout << "第" << i+1 << "幅图像的平移向量：" << endl;
		cout << "第" << i+1 << "幅图像的平移向量：" << endl;
		fout << tvecsMat[i] << endl << endl;
		cout << tvecsMat[i] << endl << endl; 
	}   
	cout << "完成保存" << endl; 
	fout << endl;
	return 0;
}