#include "ros/ros.h"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/NavSatFix.h>
#include <signal.h>
#include <std_msgs/String.h>
#include <tf/transform_broadcaster.h>
// #include <stl_algobase.h>

#include <webots_ros/set_float.h>
#include <webots_ros/set_int.h>
#include "webots_ros/arm.h"
#include "webots_ros/move.h"
#include "webots_ros/response.h"
#include "webots_ros/RecognitionObject.h"
#include "webots_ros/shelf_info.h"
#include <webots/camera.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>

#define max(a, b) (((a) > (b)) ? (a) : (b))

const char *GoodsList[] = {"can", "cereal box", "cereal box red", "jam jar", "honey jar", "water bottle", "biscuit box", "red can", "beer bottle"};

class findEmptyShelf
{
public:
    int number_of_objects;
    const WbCameraRecognitionObject *objects;
    int GoodsonShelf[4][16]; //货架上的物品ID号 先下后上 先左后右
    int CurrentShelf;        // 当前关注的货架
    int TargetGood;          //当前关注的货物种类
    int TargetIndex;         //当前关注的货架空位

    findEmptyShelf(WbDeviceTag camera, int CurrentShelf_);
    void repareFindEmptyShelf();
    bool toFindEmptyShelf();

    int name2index(char *name);
    const char *index2name(int index);
};

findEmptyShelf::findEmptyShelf(WbDeviceTag camera, int CurrentShelf_)
{
    number_of_objects = wb_camera_recognition_get_number_of_objects(camera);
    objects = wb_camera_recognition_get_objects(camera);
    CurrentShelf = CurrentShelf_;
    TargetIndex = 0;
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 16; j++)
            GoodsonShelf[i][j] = -1;
}

void findEmptyShelf::repareFindEmptyShelf()
{
    for (int i = 0; i < number_of_objects; ++i)
    {
        int Shelfx = max(0, floor((objects[i].position[0] + 0.84) * 4.17 + 0.5)); //左右 平均间隔0.24（架子宽度0.25）右移后对应一个系数 四舍五入
        int Shelfy = (objects[i].position[1] < -0.2) ? 0 : 1;                     //上下层 -0.20  为上下分界

        GoodsonShelf[CurrentShelf][Shelfy * 8 + Shelfx] = name2index(objects[i].model);
        ROS_INFO("物体 %s 对应编号 %d 写入[%d] 写入编号为%d\n", objects[i].model, name2index(objects[i].model), Shelfy * 8 + Shelfx, GoodsonShelf[CurrentShelf][Shelfy * 8 + Shelfx]);
    }
}

bool findEmptyShelf::toFindEmptyShelf()
{
    //检测完毕 判断下一个要取的货物类型
    int Empty_Flag = 0;

    for (int j = 0; j < 16; j++)
        ROS_INFO("GoodsonShelf[%d][%d] = %d\n", CurrentShelf, j, GoodsonShelf[CurrentShelf][j]);
    for (int j = 0; j < 16; j++)
    {
        if (GoodsonShelf[CurrentShelf][j] == -1)
        {
            Empty_Flag = 1;
            TargetIndex = j;
            //寻找邻近货物 判断应该取的货物类型
            //直接覆盖 假装已经放上去了
            int TargetFloor = 0;
            if (j > 7)
                TargetFloor += 8; //层数无关
            if (j % 8 < 4)
                for (int k = 0; k < 8; k++) //从左往右
                {
                    if (GoodsonShelf[CurrentShelf][TargetFloor + k] != -1)
                    {
                        // strcpy(TargetGood, GoodsonShelf[CurrentShelf][TargetFloor + k]);
                        TargetGood = GoodsonShelf[CurrentShelf][TargetFloor + k];
                        break;
                    }
                }
            else
                for (int k = 7; k >= 0; k--) //从右往左
                {
                    if (GoodsonShelf[CurrentShelf][TargetFloor + k] != -1)
                    {
                        // strcpy(TargetGood, GoodsonShelf[CurrentShelf][TargetFloor + k]);
                        TargetGood = GoodsonShelf[CurrentShelf][TargetFloor + k];
                        break;
                    }
                }
            //如果整排都没有可能会出错 下次一定
            ROS_INFO("GoodsonShelf[%d][%d] need %s\n", CurrentShelf, j, index2name(TargetGood));
            break;
        }
    }
    if (Empty_Flag)
        return true;
    else
        return false;
}

//商品名转换
int findEmptyShelf::name2index(char *name)
{
    for (int i = 0; i < sizeof(GoodsList); i++)
    {
        if (strcmp(name, GoodsList[i]) == 0)
            return i;
    }
    return -1;
}

//商品名转换
const char *findEmptyShelf::index2name(int index)
{
    return GoodsList[index];
}

// 服务器回调函数
bool find_empty(webots_ros::shelf_info::Request &req, webots_ros::shelf_info::Response &rep)
{
    findEmptyShelf f(req.camera, req.CurrentShelf);
    rep.isEmpty = f.toFindEmptyShelf();
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "findEmpty_server");
    ros::NodeHandle n;
    ros::ServiceServer service = n.advertiseService("find_empty_shelf", find_empty);
    ROS_INFO("Ready to check shelf.");
    ros::spin();
    return 0;
}