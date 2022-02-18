#include "ros/ros.h"
#include "std_msgs/String.h"
#include "dynamixel_sdk/dynamixel_sdk.h"

#include <pthread.h>
#include <unistd.h>
#include <time.h>

#define PI 3.141592

//Dynamixel Control
#define DEVICENAME                      "/dev/ttyUSB0"
#define PROTOCOL_VERSION                2.0

#define DXL1_ID                          7
#define DXL2_ID                          17
#define BAUDRATE                        2000000
#define ADDR_TORQUE_ENABLE              64
#define TORQUE_ENABLE                   1
#define ADDR_GOAL_POSITION              116
#define ADDR_PRESENT_POSITION           132

// Data Byte Length
#define LEN_PRO_GOAL_POSITION            4
#define LEN_PRO_PRESENT_POSITION         4


//time variables
int Control_Cycle = 10; //ms
bool run_status = true;

double t  =  0; //ms
double dt = 10; //ms

double T = 1000; //ms



double present_position = 0;
double target_goal_position = 1000;

/*************dxl_objects declare************/

uint8_t dxl_error = 0;
//int dxl_goal_position[2] = {0,2000};
int dxl_comm_result = COMM_TX_FAIL;
//int index = 0;

bool dxl_addparam_result = false;                // addParam result
bool dxl_getdata_result = false;                 // GetParam result

int32_t dxl_present_position = 0;
int32_t dxl_present_position2 = 0;
uint8_t param_goal_position[4];

dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
// Initialize GroupSyncWrite instance
dynamixel::GroupSyncWrite groupSyncWrite(portHandler, packetHandler, ADDR_GOAL_POSITION, LEN_PRO_GOAL_POSITION);
 // Initialize Groupsyncread instance for Present Position
dynamixel::GroupSyncRead groupSyncRead(portHandler, packetHandler, ADDR_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);

/*********************************************/

//Structures
struct End_point{
  double x;
  double y;
};

struct Joint_angle{
  double TH1;
  double TH2;
};


//functions
void dxl_initialize(void);
void torque_on_dxls(void);
void add_sync_read_param(void);
void dxl_set_goal_pose(int dxl_1_posi, int dxl_2_posi);
struct Joint_angle compute_IK(struct End_point EP);
struct End_point compute_IK(struct Joint_angle joint);
int radian_to_tick(double radian);


//END_POINT
struct End_point e1;
e1.x = 0.05;
e1.y = 0.05;

struct End_point e2;
e2.x = 0.1;
e2.y = 0.05;

struct End_point command_ep;



void process(void)
{
  //ROS_INFO("a");
  if(t<=T){
    command_ep.x = e1.x + (e2.x - e1.x)*0.5*(1.0-cos(PI*(t/T)));
    command_ep.y = e1.y + (e2.y - e1.y)*0.5*(1.0-cos(PI*(t/T)));

    struct Joint_angle j;
    j = compute_IK(command_ep);

    dxl_set_goal_pose(j.TH1, j.TH1);
    dxl_go();

    t = t+dt;
  }
  else if(t>T){
    t = 0;
    present_position = target_goal_position;
  }

}

void *p_function(void * data)
{
  ROS_INFO("thread_created...");
  sleep(1);

  pid_t pid; //process id
  pthread_t tid; // thread id

  pid = getpid();
  tid = pthread_self();

  char* thread_name = (char *)data;
  int i = 0;


  static struct timespec next_time;
  static struct timespec curr_time;

  clock_gettime(CLOCK_MONOTONIC, &next_time);

  while(run_status)
  {
    next_time.tv_sec += (next_time.tv_nsec + Control_Cycle * 1000000) / 1000000000;
    next_time.tv_nsec = (next_time.tv_nsec + Control_Cycle * 1000000) % 1000000000;

    process();

    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, NULL);

  }
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "thread_example_node");
  ros::NodeHandle nh;

  pthread_t pthread_1;
  int thr_id;
  int status;
  char p1[] = "thread_1";

  sleep(1); //1

  thr_id = pthread_create(&pthread_1, NULL, p_function, (void*)p1);

  ros::Publisher chatter_pub = nh.advertise<std_msgs::String>("chatter", 1000);

  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    std_msgs::String msg;
    msg.data = "hello world";

    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
  }

  run_status = false;


  return 0;
}



void dxl_initialize(void){
  if (portHandler->openPort())
  {
    printf("Succeeded to open the port!\n");
  }
  else
  {
    printf("Failed to open the port!\n");
    printf("Press any key to terminate...\n");
    return 0;
  }

  if (portHandler->setBaudRate(BAUDRATE))
    {
      printf("Succeeded to change the baudrate!\n");
    }
    else
    {
      printf("Failed to change the baudrate!\n");
      printf("Press any key to terminate...\n");
      return 0;
    }

}

void torque_on_dxls(void){
  //enable DXL 1,2 torque

  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL1_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL2_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);

}

void add_sync_read_param(void){
  // Add parameter storage for Dynamixel#1 present position value
  dxl_addparam_result = groupSyncRead.addParam(DXL1_ID);
  // Add parameter storage for Dynamixel#2 present position value
  dxl_addparam_result = groupSyncRead.addParam(DXL2_ID);
}

void dxl_set_goal_pose(int dxl_1_posi, int dxl_2_posi){
  param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(dxl_1_posi));
  param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(dxl_1_posi));
  param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(dxl_1_posi));
  param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(dxl_1_posi));

  dxl_addparam_result = groupSyncWrite.addParam(DXL1_ID, param_goal_position);

  param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(dxl_2_posi));
  param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(dxl_2_posi));
  param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(dxl_2_posi));
  param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(dxl_2_posi));

  dxl_addparam_result = groupSyncWrite.addParam(DXL1_ID, param_goal_position);
}

void dxl_go(void){
  groupSyncWrite.txPacket();
}


struct Joint_angle compute_IK(struct End_point EP){
  double X = EP.x;
  double Y = EP.y;

  //your IK solution

  struct Joint_angle J;
  //J.TH1 = ??;
  //J.TH2 = ??;

  return J;

}

struct End_point compute_IK(struct Joint_angle joint){
  double th1 = joint.TH1;
  double th2 = joint.TH2;

  //your FK solution

  struct End_point EP;
  //J.TH1 = ??;
  //J.TH2 = ??;

  return EP;

}

int radian_to_tick(double radian){
  return (int)(radian*(2048/PI));
}

//clock_gettime(CLOCK_MONOTONIC, &curr_time);

//long delta_nsec = (next_time.tv_sec - curr_time.tv_sec) * 1000000000 + (next_time.tv_nsec - curr_time.tv_nsec);
//printf("%f ms\n ",((double)Control_Cycle*1000000-delta_nsec)/(double)1000000);
