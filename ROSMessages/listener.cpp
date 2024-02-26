#include "ros/ros.h"
#include "std_msgs/String.h"
#include <termios.h>
#include <unistd.h>
int getch()
{
    fd_set set;
    struct timeval timeout;
    int rv;
    char buff = 0;
    int len = 1;
    int filedesc = 0;
    FD_ZERO(&set);
    FD_SET(filedesc, &set);

    timeout.tv_sec = 0;
    timeout.tv_usec = 100;

    rv = select(filedesc + 1, &set, NULL, NULL, &timeout);

    struct termios old = {0};
    if (tcgetattr(filedesc, &old) < 0)
        ROS_ERROR("tcsetattr()");
    old.c_lflag &= ~ICANON;
    old.c_lflag &= ~ECHO;
    old.c_cc[VMIN] = 1;
    old.c_cc[VTIME] = 0;
    if (tcsetattr(filedesc, TCSANOW, &old) < 0)
        ROS_ERROR("tcsetattr ICANON");

    if(rv == -1)
        ROS_ERROR("select");
    else if(rv != 0)
        read(filedesc, &buff, len );

    old.c_lflag |= ICANON;
    old.c_lflag |= ECHO;
    if (tcsetattr(filedesc, TCSADRAIN, &old) < 0)
        ROS_ERROR ("tcsetattr ~ICANON");
    return (buff);
}
void CallBack(const std_msgs::String::ConstPtr& msg){
  std::cout <<msg->data.c_str();
  std::cout << std::endl;
 }
int main(int argc, char**argv){
    ros::init(argc,argv,"talker2");
    ros::NodeHandle n;
    ros::Publisher chatter = n.advertise<std_msgs::String>("t2",1000);
    ros::Subscriber sub = n.subscribe("t1",1000,CallBack);
    ros::Rate loop_rate(10);
    while(ros::ok()){
       int k = getch();
       if (k==int('/')){
	       std_msgs::String msg;
	       std::string ss;
	       std::getline(std::cin,ss);
	       if (ss=="exit"){
	            break;
	       }
	       msg.data = ss;
	       chatter.publish(msg);
       }
       ros::spinOnce();
       loop_rate.sleep();
    }
}
/*#include "ros/ros.h"
#include "std_msgs/String.h"
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
using namespace cv;
void CallBack(const std_msgs::String::ConstPtr& msg){
  std::cout << msg->data.c_str();
  std::cout << std::endl;
 }
int main(int argc, char**argv){
    ros::init(argc,argv,"talker2");
    ros::NodeHandle n;
    ros::Publisher chatter = n.advertise<std_msgs::String>("t2",1000);
    ros::Subscriber sub = n.subscribe("t1",1000,CallBack);
    while(ros::ok()){
       Mat a = Mat::zeros(Size(256,256),CV_8UC1);
       imshow("blank", a);
       int k = waitKey(1);
       if (k==int('/')){
               std::cout << "enter: ";
	       std_msgs::String msg;
	       std::string ss;
	       std::getline(std::cin,ss);
	       msg.data = ss;
	       chatter.publish(msg);
       }
       ros::spinOnce();
    }
}*/
