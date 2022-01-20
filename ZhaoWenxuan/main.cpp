#include "han.h"

int main()
{
    cam_sign s;
    if (!inImgPath.is_open())
    {
        cout << "没有找到文件" << endl;
    }
    //读取文件中保存的图片文件路径，并存放在数组中
    string temp;
    while (getline(inImgPath, temp))
    {
        s.imgList.push_back(temp);
    }
    s.angle_point();
    s.cam_mark();
    s.save();
    waitKey(0);
    return 0;
}
