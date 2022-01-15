#include "calibration.h"

bool CCalibration::writeParams()
{
    camK.convertTo(camK, CV_32FC1);
    camDiscoeff.convertTo(camDiscoeff, CV_32FC1);
    ofstream out;
    out.open(calibResultPath + "calibResult.txt", ios::out);
    
	//记录内参
	out << "内参矩阵K：" << endl;
    out << "\t" << "[" << camK.at<float>(0, 0) << " " << camK.at<float>(0, 1) << "\t\t" << camK.at<float>(0, 2) << endl;
    out << "\t" << " " << camK.at<float>(1, 0) << "\t" << " " << camK.at<float>(1, 1) << "\t" << camK.at<float>(1, 2) << endl;
    out << "\t" << " " << camK.at<float>(2, 1) << "\t" << " " << camK.at<float>(2, 1) << "\t\t" << camK.at<float>(2, 2) << "\t" << "]" << endl;

    //记录畸变参数
	out << "畸变参数：" << endl;
	out << "\t" << "k1:" << camDiscoeff.at<float>(0, 0) << endl;
	out << "\t" << "k2:" << camDiscoeff.at<float>(0, 1) << endl;
	out << "\t" << "k3:" << camDiscoeff.at<float>(0, 2) << endl;
	out << "\t" << "p1:" << camDiscoeff.at<float>(0, 3) << endl;
	out << "\t" << "p2:" << camDiscoeff.at<float>(0, 4) << endl;

    out.close();
    return true;
}

bool CCalibration::readPatternImg()
{
    int imgNum = 0;
    Mat img;

    while(true)
    {
        stringstream ss;
        ss << imgNum;
        string path = patternImgPath + ss.str()+".JPG";

        img = imread(path, 0);
        if(!img.data)
        {
            break;
        }

		resize(img, img, Size(640, 480));	//原图像尺寸过大

        patternImgList.push_back(img);
        imgNum++;
    }

    if (imgNum == 0)
    {
        cout<< " error! No pattern imgs! " << endl;
        return false;
    }

    this->imgNum = imgNum;				//统计标定图数量
    imgHeight = patternImgList[0].rows; //计算标定图大小
    imgWidth = patternImgList[0].cols;

	cout << "(" << imgHeight << "," << imgWidth << "," << endl;
    return true;
}

bool CCalibration::testCorners(vector<Point2f>& corners, int patternWidth, int patternHeight)
{
	/**
	 * @brief 通过计算三个相邻角点构成的两个向量之间的夹角判断角点连接性，丢弃误差较大的图。
	 * param1 提取到的二维角点
	 * param2 每行的角点数
	 * param3 每列的角点数
	 */

	if (corners.size() != patternWidth * patternHeight)
	{
		return false;
	}

	double dx1, dx2, dy1, dy2;
	double cosVal;

	//列与列之间
	for (int i = 0; i < patternHeight; i++)
	{
		for (int j = 0; j < patternWidth - 2; j++)
		{ 
			dx1 = corners[i * patternWidth + j + 1].x - corners[i * patternWidth + j].x;
			dy1 = corners[i * patternWidth + j + 1].y - corners[i * patternWidth + j].y;
			dx2 = corners[i * patternWidth + j + 2].x - corners[i * patternWidth + j + 1].x;
			dy2 = corners[i * patternWidth + j + 2].y - corners[i * patternWidth + j + 1].y;

			//计算两向量的夹角 误差太大则返回false,抛弃这张标定图
			cosVal = (dx1 * dx2 + dy1 * dy2) / sqrt((dx1 * dx1 + dy1 * dy1) * (dx2 * dx2 + dy2 * dy2));
			if (fabs(cosVal) < 0.999)
			{
				return false;
			}
		}
	}

	//行与行
	for (int i = 0; i < patternHeight - 2; i++)
	{
		for (int j = 0; j < patternWidth; j++)
		{
			dx1 = corners[(i + 1) * patternWidth + j].x - corners[i * patternWidth + j].x;
			dy1 = corners[(i + 1) * patternWidth + j].y - corners[i * patternWidth + j].y;
			dx2 = corners[(i + 2) * patternWidth + j].x - corners[(i + 1) * patternWidth + j].x;
			dy2 = corners[(i + 2) * patternWidth + j].y - corners[(i + 1) * patternWidth + j].y;
			cosVal = (dx1 * dx2 + dy1 * dy2) / sqrt((dx1 * dx1 + dy1 * dy1) * (dx2 * dx2 + dy2 * dy2));
			if (fabs(cosVal) < 0.999)
			{
				return false;
			}
		}
	}
	return true;
}

//初始化角点的三维坐标
void CCalibration::init3DPoints(Size boardSize, Size squareSize, vector<Point3f> &singlePatternPoint)
{
	for (int i = 0; i < boardSize.height; i++)
	{
		for (int j = 0; j < boardSize.width; j++)
		{
			Point3f tempPoint;	//单个角点的三维坐标
			tempPoint.x = float(i * squareSize.width);
			tempPoint.y = float(j * squareSize.height);
			tempPoint.z = 0;
			singlePatternPoint.push_back(tempPoint);
		}
	}
}

void CCalibration::calibProcess()
{
    //相机标定
    double time0 = (double)getTickCount();
    vector<Point2f> corners;				//存储一幅棋盘图中的所有角点二维坐标
    vector<vector<Point2f>> corner2d;		//存储所有棋盘图角点的二维坐标
    vector<Mat> image_Seq;					//存储所有棋盘图
    int successImgNum = 0;
    int count = 0;
    Mat image, scaleImg;

    cout<<"开始提取角点！"<<endl;
    for (int i=0; i<imgNum; i++)
    {
        image = patternImgList[i].clone();

	    //提取角点
	    bool found = findChessboardCorners(image, boardSize, corners, CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_ASYMMETRIC_GRID + CALIB_CB_FILTER_QUADS);
	    if (!found)
	    {
		    cout<<"Can not find chess board corners!\n"<<endl;
		    continue;
	    }
	    else
	    {
			//亚像素精确化
		    cornerSubPix(image, corners, Size(11, 11), Size(-1,-1), TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 30, 0.1));
			//检测角点的连通性（排除质量较差的图片）
			bool good = testCorners(corners, boardSize.width, boardSize.height);
			if (good == false)	continue;

			//绘制检测到的角点并显示
		    Mat cornerImg = image.clone();
            cvtColor(cornerImg, cornerImg, COLOR_GRAY2BGR);
			
			drawChessboardCorners(cornerImg, boardSize, corners, found);
            namedWindow("CirclePattern", WINDOW_AUTOSIZE);
            imshow("CirclePattern", cornerImg);
			waitKey(0);

		    count += (int)corners.size();	//所有棋盘图中的角点个数
		    successImgNum++;				//成功提取角点的棋盘图个数
		    corner2d.push_back(corners);
			image_Seq.push_back(image);
	    }
    }
    cout<<"角点提取完成！"<<endl;
	
    //相机标定
    vector<vector<Point3f>> object_points;	//所有棋盘图像的角点三维坐标
    vector<int> pointCounts;

	//初始化单幅靶标图片的三维点
	init3DPoints(boardSize, squre_size, singlePatternPoints);

    //初始化标定板上的三维坐标
	for (int n = 0; n < successImgNum; n++)
	{
		object_points.push_back(singlePatternPoints);
		pointCounts.push_back(boardSize.width * boardSize.height);
	}

    //开始标定
    cout<<"开始标定!"<<endl;
    Size imgSize = Size(imgWidth, imgHeight);

    int flags=0;
	calibrateCamera(object_points, corner2d, imgSize, camK, camDiscoeff,rotation, translation, flags);

    cout<<"标定完成！"<<endl;

	//计算FPS
    double time1 = getTickCount();
    cout<< "Calibration time: "<< (time1-time0)/getTickFrequency() <<"s"<<endl;
}

void CCalibration::run()
{
    bool readSuccess = readPatternImg();
	if (!readSuccess)
	{
		cout << "no image!" << endl;
		getchar();
	}
	calibProcess();
	writeParams();
}
