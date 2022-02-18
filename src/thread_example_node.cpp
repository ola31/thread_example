#include "ros/ros.h"
#include "std_msgs/String.h"

#include <pthread.h>
#include <unistd.h>
#include <time.h>

#define PI 3.141592

int Control_Cycle = 10; //ms
bool run_status = true;

double t  =  0; //ms
double dt = 10; //ms

double T = 1000; //ms

double present_position = 0;
double target_goal_position = 1000;

void process(void)
{
  //ROS_INFO("a");
  if(t<=T){
    double goal_position = present_position + (target_goal_position - present_position)*0.5*(1.0-cos(PI*(t/T)));
    ROS_INFO("%lf",goal_position);
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



//clock_gettime(CLOCK_MONOTONIC, &curr_time);

//long delta_nsec = (next_time.tv_sec - curr_time.tv_sec) * 1000000000 + (next_time.tv_nsec - curr_time.tv_nsec);
//printf("%f ms\n ",((double)Control_Cycle*1000000-delta_nsec)/(double)1000000);
