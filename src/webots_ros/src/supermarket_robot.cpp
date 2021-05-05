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

#include <webots/camera.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>

#define TIME_STEP 32
#define NCAMERA 2
#define max(a, b) (((a) > (b)) ? (a) : (b))

ros::NodeHandle *n;
static int controllerCount;
static std::vector<std::string> controllerList;
ros::ServiceClient timeStepClient;
webots_ros::set_int timeStepSrv;

const char *cameraNames[NCAMERA] = {"camera_front", "camera_behind"}; //camera的名字
std::string ROS_NODE_NAME = "supermarketRobot";

bool Init(int argc, char **argv);           //初始化函数
bool InitController(int argc, char **argv); //完成控制器链接工作
bool enableSrv(const char **name, int name_len, std::string srvName);
void findObj();
void ControllerNameCallback(const std_msgs::String::ConstPtr &name); //回调函数获得可供操作的机器人
void Quit(int sig);                                                  //退出程序
void FrontImgCallBack(const sensor_msgs::Image::ConstPtr &msg);
void BehindImgCallBack(const sensor_msgs::Image::ConstPtr &msg);
void ResponseCallBack(const webots_ros::response::ConstPtr &msg); //得到从youbot中返回的消息（zth自己定义）
void recogObjCallBack(const webots_ros::RecognitionObject::ConstPtr &msg);

cv::Mat frontImg;
cv::Mat behindImg;

int main(int argc, char **argv)
{
    if (Init(argc, argv) == true) //初始化
        ROS_INFO("Init succeed!");
    else
    {
        ROS_ERROR("Init failed!");
        return -1;
    }
    //创建有关youbot运动的话题发布者
    ros::Publisher movePub = n->advertise<webots_ros::move>("/youbot/move", 10);
    ros::Publisher armPub = n->advertise<webots_ros::arm>("/youbot/arm", 10);
    //创建图像话题订阅者
    // ros::Subscriber forntImgSub = n->subscribe("/supermarketRobot/camera_front/image", 10, FrontImgCallBack);
    // ros::Subscriber behindImgSub = n->subscribe("/supermarketRobot/camera_behind/image", 10, BehindImgCallBack);
    ros::Subscriber resultSub = n->subscribe("/youbot/response", 10, ResponseCallBack);

    while (ros::ok())
    {
        //发布消息
        // 初始化learning_topic::Person类型的消息
        webots_ros::move moveMsg;
        webots_ros::arm armMsg;
        moveMsg.moveType = "Go forward";
        moveMsg.speed = 1.2;

        armMsg.operateFlag[0] = true;
        armMsg.angle[0] = 1.57;
        armMsg.operateFlag[1] = false;
        armMsg.angle[1] = 0;
        armMsg.operateFlag[2] = false;
        armMsg.angle[2] = 0;
        armMsg.operateFlag[3] = false;
        armMsg.angle[3] = 0;
        armMsg.operateFlag[4] = false;
        armMsg.angle[4] = 0;
        armMsg.grabFlag = false; //张开手爪

        // 发布消息
        //movePub.publish(moveMsg);
        //armPub.publish(armMsg);

        if (!timeStepClient.call(timeStepSrv) || !timeStepSrv.response.success)
        {
            ROS_ERROR("Failed to call service time_step for next step.");
            break;
        }
        ros::spinOnce();
    }
    timeStepSrv.request.value = 0;
    timeStepClient.call(timeStepSrv);
    ros::shutdown();
    return 0;
}

bool Init(int argc, char **argv) //初始化函数
{
    if (InitController(argc, argv) == true) //控制器初始化
        ROS_INFO("Init controller succeed!");
    else
    {
        ROS_ERROR("Init controller failed!");
        return false;
    }

    if (enableSrv(cameraNames, NCAMERA, std::string("/enable")) == true) // camera 初始化
        ROS_INFO("Init camera succeed!");
    else
    {
        ROS_ERROR("Init camera failed!");
        return false;
    }

    if (enableSrv(cameraNames, NCAMERA, std::string("/recognition_enable")) == true) // recognition 初始化
        ROS_INFO("Init camera recognition succeed!");
    else
    {
        ROS_ERROR("Init camera recognition failed!");
        return false;
    }

    return true;
}

bool InitController(int argc, char **argv)
{
    std::string controllerName;
    //创建节点ros"SuperMarketRobot"
    ros::init(argc, argv, ROS_NODE_NAME, ros::init_options::AnonymousName);
    n = new ros::NodeHandle;
    signal(SIGINT, Quit);
    //订阅话题model_name来查看当前可供操作的机器人
    ros::Subscriber nameSub = n->subscribe("model_name", 100, ControllerNameCallback);
    while (controllerCount == 0 || controllerCount < nameSub.getNumPublishers())
        ros::spinOnce();

    ros::spinOnce();
    //设定机器人的仿真步数
    timeStepClient = n->serviceClient<webots_ros::set_int>(ROS_NODE_NAME + "/robot/time_step");
    timeStepSrv.request.value = TIME_STEP;

    //如果多余一个机器人，选择该控制器的控制对象
    if (controllerCount == 1)
        controllerName = controllerList[0];
    else
    {
        int wantedController = 0;
        std::cout << "Choose the # of the controller you want touse:\n";
        std::cin >> wantedController;
        if (1 <= wantedController && wantedController <= controllerCount)
            controllerName = controllerList[wantedController - 1];
        else
        {
            ROS_ERROR("Invalid number for controller choice.");
            return false;
        }
    }
    ROS_INFO("Using controller: '%s'", controllerName.c_str());
    // 关闭该服务
    nameSub.shutdown();
    return true;
}

// call service
bool enableSrv(const char **name, int name_len, std::string srvName)
{
    ros::ServiceClient client;
    webots_ros::set_int srv;
    const char *name_ = NULL;
    name_ = *name++;
    for (int i = 0; i < name_len; i++)
    {
        //请求服务 superMarketRobot/kinect_xxx_color/enable
        client = n->serviceClient<webots_ros::set_int>(ROS_NODE_NAME + std::string("/") + name_ + srvName);
        srv.request.value = 64;
        if (client.call(srv) && srv.response.success)
            ROS_INFO("[%s] succeed...", srvName.c_str());
        else
        {
            ROS_ERROR("[%s] failed...", srvName.c_str());
            return false;
        }
        ROS_INFO("name_ = %s\n", name_);
        name_ = *name;
    }
    return true;
}

// recognition objects
void findObj()
{
    ROS_INFO("To find object.");
    ros::Subscriber recogObj = n->subscribe("/supermarketRobot/camera_front/recognition_objects", 10, recogObjCallBack);
    ros::spin();
}

void ControllerNameCallback(const std_msgs::String::ConstPtr &name)
{
    controllerCount++;
    controllerList.push_back(name->data);
    ROS_INFO("Controller #%d: %s.", controllerCount, controllerList.back().c_str());
}

void Quit(int sig)
{
    ROS_INFO("User stopped the 'ros_test' node.");
    timeStepSrv.request.value = 0;
    timeStepClient.call(timeStepSrv);
    ros::shutdown();
    exit(0);
}

void FrontImgCallBack(const sensor_msgs::Image::ConstPtr &msg)
{
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    frontImg = cv_ptr->image;
    // cv::imshow("frontImage", frontImg);
    // cv::waitKey(1);
}

void BehindImgCallBack(const sensor_msgs::Image::ConstPtr &msg)
{
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    behindImg = cv_ptr->image;
    // cv::imshow("behindImage", behindImg);
    // cv::waitKey(1);
}
void ResponseCallBack(const webots_ros::response::ConstPtr &msg)
{
}

void recogObjCallBack(const webots_ros::RecognitionObject::ConstPtr &msg)
{
    ROS_INFO("(%f, %f, %f)\n", msg->position_on_image.x, msg->position_on_image.y, msg->position_on_image.z);
}
