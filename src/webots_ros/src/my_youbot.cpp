#include "ros/ros.h"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/MagneticField.h>
#include <signal.h>
#include <std_msgs/String.h>
#include<tf/transform_broadcaster.h>

 
#include <webots_ros/set_float.h>
#include <webots_ros/set_int.h>
#include "webots_ros/arm.h"
#include "webots_ros/move.h"
#include "webots_ros/response.h"
 

#define TIME_STEP 32
#define MAX_SPEED 7.0
#define NMOTORS   4
#define NARM      5
#define NFINGER	  2
//移动状态
#define NORMAL    0
#define GOTO      1
//爪子的状态
#define REALSE    0
#define GRAB      1
#define FINGER_MIN_POS 0.0
#define FINGER_MAX_POS 0.025
// stimulus coefficients
#define K1 -3.0
#define K2 10.0
#define K3 10.0
#define DISTANCE_TOLERANCE 0.05
#define ANGLE_TOLERANCE 0.05



///////////math//////////
typedef struct 
{
  double u;
  double v;
} Vector2;
typedef struct 
{
  double u;
  double v;
  double w;
} Vector3;

typedef struct 
{
  Vector3 a;
  Vector3 b;
  Vector3 c;
} Matrix33;
typedef struct 
{
  Vector2 v_target;
  double alpha;
  bool reached;
} goto_struct;
// --- Vector2 functions ---
double vector2_norm(const Vector2 *v);                                 // ||v||
void vector2_minus(Vector2 *v, const Vector2 *v1, const Vector2 *v2);  // v = v1-v2
double vector2_angle(const Vector2 *v1, const Vector2 *v2);            // angle between v1 and v2 -> [0, 2Pi]
// --- Vector3 functions ---
void vector3_set_values(Vector3 *vect, double u, double v, double w);
// --- Matrix33 functions ---
void matrix33_set_values(Matrix33 *m, double au, double av, double aw, double bu, double bv, double bw, double cu, double cv,double cw);
void matrix33_set_identity(Matrix33 *m);
void matrix33_mult_vector3(Vector3 *res, const Matrix33 *m, const Vector3 *v);  // res = m * v
double bound(double v, double a, double b);
///////////math//////////



ros::NodeHandle *n;
static int controllerCount;
static std::vector<std::string> controllerList; 
ros::ServiceClient timeStepClient;
webots_ros::set_int timeStepSrv;
float baseSpeed;//底盘速度
float motorPositions[4];//四个轮子的位置 
float motorSpeeds[4];//四个轮子的速度
float armPositions[5];//五个关节的位置
float armSpeeds[5]; //五个关节的速度
static goto_struct goto_data;//目标位置
sensor_msgs::NavSatFix gpsRawValue;
sensor_msgs::MagneticField compassRawValue;
int grabStatus;
int moveStatus;
static const char *motorNames[NMOTORS] ={"wheel1", "wheel2", "wheel3","wheel4"};//匹配之前定义好的电机name
static const char *armNames[NARM] ={"arm1", "arm2", "arm3","arm4", "arm5"};//匹配之前定义好的电机name
static const char *fingerNames[NFINGER] ={"finger1", "finger2"};//匹配之前定义好的电机name
float preGrabPosition[NARM]={0.0, 0.0, -2.635, 1.0, 0.0};
float grabPosition[NARM]={0.0, 0.0, -2.635, 0.5, 0.0};
float upliftPosition[NARM]={0.0, 1.57, -0.4, -1.78, 0.0};
float down2upPosition[NARM]={0.0, 0.6, -1.8, 0.0, 0.0};
float down2downPosition[NARM]={0.0, -0.8, -2.635, 1.5, 0.0};

std::string ROS_NODE_NAME = "myYoubot";


bool Init(int argc, char **argv);//初始化函数
bool InitController(int argc, char **argv);//完成控制器链接工作
bool InitMotor();//初始化电机
bool InitArm();//初始化机械臂
bool InitGps();//初始化Gps
bool InitCompass();//初始化电子罗盘
void ControllerNameCallback(const std_msgs::String::ConstPtr &name);//回调函数获得可供操作的机器人
void Quit(int sig);//退出程序
void MoveCallback(const webots_ros::move::ConstPtr& msg);
void ArmCallback(const webots_ros::arm::ConstPtr& msg);
void GpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg);
void CompassCallback(const sensor_msgs::MagneticField::ConstPtr& msg);
void Update();//对youbot进行控制


bool SetMotorPosition();
bool SetMotorSpeed();
bool SetArmPosition();
bool SetArmSpeed();
bool ClawRealse();
bool ClawGrab();

void base_reset();
void base_forwards();
void base_backwards();
void base_turn_left();
void base_turn_right();
void base_strafe_left();
void base_strafe_right();

void base_goto_init();
void base_goto_set_target(double x, double z, double a);
void base_goto_run();
bool base_goto_reached();



int main(int argc, char **argv)
{
	if(Init(argc, argv)==true)//初始化
		ROS_INFO("Init succeed!");
	else
	{
		ROS_ERROR("Init failed!");
		return -1;
	}
	//创建话题订阅者
	ros::Subscriber moveSub = n->subscribe("/youbot/move", 10, MoveCallback);
	ros::Subscriber armSub = n->subscribe("/youbot/arm", 10, ArmCallback);
	ros::Subscriber gpsSub = n->subscribe("/supermarketRobot/gps_youbot/values", 10, GpsCallback);
	ros::Subscriber compassSub = n->subscribe("/supermarketRobot/compass_youbot/values", 10, CompassCallback);
	ros::Publisher resultPub = n->advertise<webots_ros::response>("/youbot/response", 10);


	while (ros::ok())
	{   		
		if(base_goto_reached()==true&&moveStatus==GOTO)//到达了指定位置
		{
			ROS_INFO("I get the pose");
			webots_ros::response response;
			response.response=std::string("Goto finished");
			resultPub.publish(response);
		}

		if (!timeStepClient.call(timeStepSrv) || !timeStepSrv.response.success)
		{  
			ROS_ERROR("Failed to call service time_step for next step.");     
			break;   
		}
		Update();   
		ros::spinOnce();
	} 
	timeStepSrv.request.value = 0;
	timeStepClient.call(timeStepSrv);
	ros::shutdown(); 
	return 0;
}


bool Init(int argc, char **argv)//初始化函数
{
	grabStatus=GRAB;
	moveStatus=NORMAL;
	base_goto_init();
	if(InitController(argc, argv)==true)//控制器初始化
		ROS_INFO("InitController succeed!");
	else
	{
		ROS_ERROR("InitController failed!");
		return false;
	}
	if(InitMotor()==true)//电机初始化
		ROS_INFO("InitMotor succeed!");
	else
	{
		ROS_ERROR("InitMotor failed!");
		return false;
	}
	if(InitArm()==true)//电机初始化
		ROS_INFO("InitArm succeed!");
	else
	{
		ROS_ERROR("InitArm failed!");
		return false;
	}

	//传感器初始化
	if(InitGps()==true)//Gps初始化
		ROS_INFO("InitGps succeed!");
	else
	{
		ROS_ERROR("InitGps failed!");
		return false;
	}
	if(InitCompass()==true)//电子罗盘初始化
		ROS_INFO("InitCompass succeed!");
	else
	{
		ROS_ERROR("InitCompass failed!");
		return false;
	}
	base_goto_init(); //导航位置初始化

	return true;
}


bool InitController(int argc, char **argv)
{
	std::string controllerName;
	//创建节点ros"SuperMarketRobot"
	ros::init(argc, argv, ROS_NODE_NAME,ros::init_options::AnonymousName);
	n = new ros::NodeHandle;  
	signal(SIGINT, Quit);
	//订阅话题model_name来查看当前可供操作的机器人
 	ros::Subscriber nameSub = n->subscribe("model_name", 100, ControllerNameCallback);
	 while (controllerCount == 0 || controllerCount <nameSub.getNumPublishers())
		 ros::spinOnce(); 

	ros::spinOnce();
	//设定机器人的仿真步数
	timeStepClient = n->serviceClient<webots_ros::set_int>(ROS_NODE_NAME+"/robot/time_step");
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

bool InitMotor()//初始化电机
{
	for(int i=0;i<NMOTORS;i++)
	{
		motorPositions[i]=INFINITY;
		motorSpeeds[i]=0;
	}
	if(SetMotorPosition()&&SetMotorSpeed())
		return true;
	else
		return false;
}
bool InitArm()
{
	armPositions[0]=0.0;
	armPositions[1]=1.57;
	armPositions[2]=-2.635;
	armPositions[3]=1.78;
	armPositions[4]=0.0;
	for(int i=0;i<NMOTORS;i++)
		motorSpeeds[i]=0;
	if(SetMotorPosition()&&SetMotorSpeed())
		return true;
	else
		return false;
}

bool InitGps()
{
	ros::ServiceClient enableClient;   
	webots_ros::set_int enableSrv;
	//请求服务 supermarketRobot/gps_youbot/enable   
	enableClient = n->serviceClient<webots_ros::set_int>(std::string("/supermarketRobot/gps_youbot") + std::string("/enable"));   
	enableSrv.request.value = 64;
	if (enableClient.call(enableSrv) &&enableSrv.response.success)
			ROS_INFO("succeed enable gps.");
	else
	{     
		ROS_ERROR("Failed to enable gps.");
		return false;
	}
	return true;   
}

bool InitCompass()
{
	ros::ServiceClient enableClient;   
	webots_ros::set_int enableSrv;
	//请求服务 supermarketRobot/compass_youbot/enable   
	enableClient = n->serviceClient<webots_ros::set_int>(std::string("/supermarketRobot/compass_youbot") + std::string("/enable"));   
	enableSrv.request.value = 64;
	if (enableClient.call(enableSrv) &&enableSrv.response.success)
			ROS_INFO("succeed enable compass.");
	else
	{     
		ROS_ERROR("Failed to enable compass.");
		return false;
	}
	return true;   
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


void MoveCallback(const webots_ros::move::ConstPtr& msg)
{
	ROS_INFO("Get Motor Message!");
	motorSpeeds[0] = 0;
	motorSpeeds[1] = 0;
	motorSpeeds[2] = 0;
	motorSpeeds[3] = 0;
	if(msg->speed<MAX_SPEED)	
		baseSpeed=msg->speed;
	else
		baseSpeed=MAX_SPEED;
	if(msg->moveType=="Go forwards")
		base_forwards();
	if(msg->moveType=="Go backwards")
        base_backwards();
	if(msg->moveType=="Strafe left")
        base_strafe_left();
	if(msg->moveType=="Strafe right")
        base_strafe_right();
	if(msg->moveType=="Turn left")
        base_turn_left();
	if(msg->moveType=="Turn right")
        base_turn_right();
	if(msg->moveType=="Reset")
        base_reset();
	if(msg->moveType=="Goto")
	{
        base_goto_set_target(msg->x,msg->z,msg->alphe);
		base_goto_run();
		moveStatus=GOTO;
	}
}
void ArmCallback(const webots_ros::arm::ConstPtr& msg)
{
	ROS_INFO("Get Arm Message!");
	if(msg->grabFlag==true)
		grabStatus=GRAB;
	else
		grabStatus=REALSE;

	if(msg->type=="preGrab")
		for(int i=0;i<NARM;i++)
			armPositions[i]=preGrabPosition[i];

	if(msg->type=="grab")
		for(int i=0;i<NARM;i++)
			armPositions[i]=grabPosition[i];

	if(msg->type=="uplift")
		for(int i=0;i<NARM;i++)
			armPositions[i]=upliftPosition[i];
	
	if(msg->type=="down2up")
		for(int i=0;i<NARM;i++)
			armPositions[i]=down2upPosition[i];
	
	if(msg->type=="down2down")
		for(int i=0;i<NARM;i++)
			armPositions[i]=down2downPosition[i];

	if(msg->type=="normal")
	{
		for(int i=0;i<NARM;i++)
			if(msg->operateFlag[i]==true)
				armPositions[i]=msg->angle[i];
	}

}

void GpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
	gpsRawValue=*msg;
}
void CompassCallback(const sensor_msgs::MagneticField::ConstPtr& msg)
{
	compassRawValue=*msg;
}

void Update()
{
	SetMotorSpeed();//设置电机速度
	SetArmPosition();//设置机械臂各关节位姿
	if(grabStatus==REALSE)
		ClawRealse();
	else
		ClawGrab();
}

////////////////底层函数////////////////



bool SetMotorPosition()
{
	for (int i = 0; i < NMOTORS; ++i) 
	{  
		ros::ServiceClient setPositionClient;   
		webots_ros::set_float setPositionSrv;
		//请求服务 SuperMarketRobot/wheel_xxx/set_position   
		setPositionClient = n->serviceClient<webots_ros::set_float>(ROS_NODE_NAME+std::string("/")
																+ std::string(motorNames[i]) + std::string("/set_position"));   
		setPositionSrv.request.value = motorPositions[i];
		if (setPositionClient.call(setPositionSrv) &&setPositionSrv.response.success)
			ROS_INFO("Position set to %0.2f for motor %s.",setPositionSrv.request.value,motorNames[i]);
		else
		{     
			ROS_ERROR("Failed to call service set_position on motor %s.",motorNames[i]);
			return false;
		}
	}
	return true;   
}


bool SetMotorSpeed()
{
	for (int i = 0; i < NMOTORS; ++i) 
	{  
	 
		ros::ServiceClient setVelocityClient;
		webots_ros::set_float setVelocitySrv;
		//请求服务 SuperMarketRobot/wheel_xxx/set_velocity      
		setVelocityClient =n->serviceClient<webots_ros::set_float>(ROS_NODE_NAME+std::string("/")
																+ std::string(motorNames[i]) + std::string("/set_velocity"));   
		setVelocitySrv.request.value = motorSpeeds[i];   
		if (setVelocityClient.call(setVelocitySrv) && setVelocitySrv.response.success == 1)     
			;//ROS_INFO("Velocity set to %0.2f for motor %s.", setVelocitySrv.request.value,motorNames[i]);   
		else
		{     
			ROS_ERROR("Failed to call service set_velocity on motor %s.",motorNames[i]);
			return false;
		}
	}
	return true;   
}

bool SetArmPosition()
{
	for (int i = 0; i < NARM; ++i) 
	{  
		ros::ServiceClient setPositionClient;   
		webots_ros::set_float setPositionSrv;
		//请求服务 SuperMarketRobot/arm_xxx/set_position   
		setPositionClient = n->serviceClient<webots_ros::set_float>(ROS_NODE_NAME+std::string("/")
																+ std::string(armNames[i]) + std::string("/set_position"));   
		setPositionSrv.request.value = armPositions[i];
		if (setPositionClient.call(setPositionSrv) &&setPositionSrv.response.success)
			;//ROS_INFO("Position set to %0.2f for arm %s.",setPositionSrv.request.value,armNames[i]);
		else
		{     
			ROS_ERROR("Failed to call service set_position on arm %s.",armNames[i]);
			return false;
		}
	}
	return true;   
}


bool SetArmSpeed()
{
	for (int i = 0; i < NARM; ++i) 
	{  
		ros::ServiceClient setVelocityClient;
		webots_ros::set_float setVelocitySrv;
		//请求服务 SuperMarketRobot/arm_xxx/set_velocity      
		setVelocityClient =n->serviceClient<webots_ros::set_float>(ROS_NODE_NAME+std::string("/")
																+ std::string(armNames[i]) + std::string("/set_velocity"));   
		setVelocitySrv.request.value = armSpeeds[i];   
		if (setVelocityClient.call(setVelocitySrv) && setVelocitySrv.response.success == 1)     
			;//ROS_INFO("Velocity set to %0.2f for arm %s.", setVelocitySrv.request.value,armNames[i]);   
		else
		{     
			ROS_ERROR("Failed to call service set_velocity on arm %s.", armNames[i]);
			return false;
		}
	}
	return true;   
}

bool ClawGrab()
{
	for (int i = 0; i < NFINGER; ++i) 
	{  
		ros::ServiceClient setPositionClient;   
		webots_ros::set_float setPositionSrv;
		//请求服务 SuperMarketRobot/finger_xxx/set_position   
		setPositionClient = n->serviceClient<webots_ros::set_float>(ROS_NODE_NAME+std::string("/")
																+ std::string(fingerNames[i]) + std::string("/set_position"));   
		setPositionSrv.request.value = FINGER_MIN_POS;
		if (setPositionClient.call(setPositionSrv) &&setPositionSrv.response.success)
			;//ROS_INFO("Claw Realsed");
		else
		{     
			ROS_ERROR("Failed to call service set_position on %s.",fingerNames[i]);
			return false;
		}
	}
	return true;   
}

bool ClawRealse()
{
	for (int i = 0; i < NFINGER; ++i) 
	{  
		ros::ServiceClient setPositionClient;   
		webots_ros::set_float setPositionSrv;
		//请求服务 SuperMarketRobot/finger_xxx/set_position   
		setPositionClient = n->serviceClient<webots_ros::set_float>(ROS_NODE_NAME+std::string("/")
																+ std::string(fingerNames[i]) + std::string("/set_position"));   
		setPositionSrv.request.value = FINGER_MAX_POS;
		if (setPositionClient.call(setPositionSrv) &&setPositionSrv.response.success)
			;//ROS_INFO("Claw grabbed");
		else
		{     
			ROS_ERROR("Failed to call service set_position on %s.",fingerNames[i]);
			return false;
		}
	}
	return true;   
}



void base_reset() 
{
	motorSpeeds[0]=0.0;motorSpeeds[1]=0.0; motorSpeeds[2]=0.0;motorSpeeds[3]=0.0;
}

void base_forwards() 
{
	motorSpeeds[0]=baseSpeed;motorSpeeds[1]=baseSpeed;motorSpeeds[2]=baseSpeed;motorSpeeds[3]=baseSpeed;
}

void base_backwards() 
{
	motorSpeeds[0]=-baseSpeed;motorSpeeds[1]=-baseSpeed;motorSpeeds[2]=-baseSpeed;motorSpeeds[3]=-baseSpeed;
}

void base_turn_left() 
{
	motorSpeeds[0]=-baseSpeed;motorSpeeds[1]=baseSpeed;motorSpeeds[2]=-baseSpeed;motorSpeeds[3]=baseSpeed;
}

void base_turn_right() 
{
	motorSpeeds[0]=baseSpeed;motorSpeeds[1]=-baseSpeed;motorSpeeds[2]=baseSpeed;motorSpeeds[3]=-baseSpeed;  
}

void base_strafe_left() 
{
	motorSpeeds[0]=baseSpeed;motorSpeeds[1]=-baseSpeed;motorSpeeds[2]=-baseSpeed;motorSpeeds[3]=baseSpeed;
}

void base_strafe_right() 
{
	motorSpeeds[0]=-baseSpeed;motorSpeeds[1]=baseSpeed;motorSpeeds[2]=baseSpeed;motorSpeeds[3]=-baseSpeed;  
}


void base_goto_init() 
{
  goto_data.v_target.u = 0.0;
  goto_data.v_target.v = 0.0;
  goto_data.alpha = 0.0;
  goto_data.reached = false;
}

void base_goto_set_target(double x, double z, double alpha) 
{
  goto_data.v_target.u = x;
  goto_data.v_target.v = z;
  goto_data.alpha = alpha;
  goto_data.reached = false;
}

void base_goto_run() 
{
  ROS_INFO("RUN");
  motorSpeeds[0] = 0;
  motorSpeeds[1] = 0;
  motorSpeeds[2] = 0;
  motorSpeeds[3] = 0;
  // compute 2d vectors
  Vector2 v_gps = {gpsRawValue.latitude, gpsRawValue.longitude};
  Vector2 v_front = {compassRawValue.magnetic_field.x, compassRawValue.magnetic_field.z};
  Vector2 v_right = {-v_front.v, v_front.u};
  Vector2 v_north = {1.0, 0.0};

  // compute distance
  Vector2 v_dir;
  vector2_minus(&v_dir, &goto_data.v_target, &v_gps);
  double distance = vector2_norm(&v_dir);

  // compute absolute angle & delta with the delta with the target angle
  double theta = vector2_angle(&v_front, &v_north);
  double delta_angle = theta - goto_data.alpha;

  // compute the direction vector relatively to the robot coordinates
  // using an a matrix of homogenous coordinates
  Matrix33 transform;
  matrix33_set_identity(&transform);
  transform.a.u = v_front.u;
  transform.a.v = v_right.u;
  transform.b.u = v_front.v;
  transform.b.v = v_right.v;
  transform.c.u = -v_front.u * v_gps.u - v_front.v * v_gps.v;
  transform.c.v = -v_right.u * v_gps.u - v_right.v * v_gps.v;
  Vector3 v_target_tmp = {goto_data.v_target.u, goto_data.v_target.v, 1.0};
  Vector3 v_target_rel;
  matrix33_mult_vector3(&v_target_rel, &transform, &v_target_tmp);

  // compute the speeds
  // -> first stimulus: delta_angle

  motorSpeeds[0] = -delta_angle / M_PI * K1;
  motorSpeeds[1] = delta_angle / M_PI * K1;
  motorSpeeds[2] = -delta_angle / M_PI * K1;
  motorSpeeds[3] = delta_angle / M_PI * K1;

  // -> second stimulus: u coord of the relative target vector
  motorSpeeds[0] += v_target_rel.u * K2;
  motorSpeeds[1] += v_target_rel.u * K2;
  motorSpeeds[2] += v_target_rel.u * K2;
  motorSpeeds[3] += v_target_rel.u * K2;

  // -> third stimulus: v coord of the relative target vector
  motorSpeeds[0] += -v_target_rel.v * K3;
  motorSpeeds[1] += v_target_rel.v * K3;
  motorSpeeds[2] += v_target_rel.v * K3;
  motorSpeeds[3] += -v_target_rel.v * K3;

  // apply the speeds
  int i;
  for (i = 0; i < 4; i++) 
  {
    motorSpeeds[i] /= (K1 + K2 + K2);  // number of stimuli (-1 <= motorSpeeds <= 1)
    motorSpeeds[i] *= MAX_SPEED;           // map to speed (-SPEED <= motorSpeeds <= SPEED)

    // added an arbitrary factor increasing the convergence speed
    motorSpeeds[i] *= 30.0;
    motorSpeeds[i] = bound(motorSpeeds[i], -MAX_SPEED, MAX_SPEED);
  }
  // check if the taget is reached
  ROS_INFO("angle:%0.2f,distance:%0.2f",delta_angle,distance);
  if (distance < DISTANCE_TOLERANCE && delta_angle < ANGLE_TOLERANCE && delta_angle > -ANGLE_TOLERANCE)
	  goto_data.reached = true;
 
}

bool base_goto_reached() 
{
  return goto_data.reached;
}



/////////////math///////////
void vector3_set_values(Vector3 *vect, double u, double v, double w) 
{
  vect->u = u;
  vect->v = v;
  vect->w = w;
}

void matrix33_set_values(Matrix33 *m, double au, double av, double aw, double bu, double bv, double bw, double cu, double cv,double cw) 
{
  vector3_set_values(&(m->a), au, av, aw);
  vector3_set_values(&(m->b), bu, bv, bw);
  vector3_set_values(&(m->c), cu, cv, cw);
}

void matrix33_set_identity(Matrix33 *m) 
{
  matrix33_set_values(m, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
}

void matrix33_mult_vector3(Vector3 *res, const Matrix33 *m, const Vector3 *v) 
{
  res->u = m->a.u * v->u + m->b.u * v->v + m->c.u * v->w;
  res->v = m->a.v * v->u + m->b.v * v->v + m->c.v * v->w;
  res->w = m->a.w * v->u + m->b.w * v->v + m->c.w * v->w;
}

double vector2_norm(const Vector2 *v) 
{
  return sqrt(v->u * v->u + v->v * v->v);
}

void vector2_minus(Vector2 *v, const Vector2 *v1, const Vector2 *v2) 
{
  v->u = v1->u - v2->u;
  v->v = v1->v - v2->v;
}

double vector2_angle(const Vector2 *v1, const Vector2 *v2) 
{
  return atan2(v2->v, v2->u) - atan2(v1->v, v1->u);
}

double bound(double v, double a, double b) 
{
  return (v > b) ? b : (v < a) ? a : v;
}





