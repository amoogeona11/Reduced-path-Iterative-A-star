#include <ros/ros.h>
#include "gazebo_msgs/SetModelState.h"
#include "gazebo_msgs/ModelState.h"
#include "gazebo_msgs/ModelStates.h"
#include "gazebo_msgs/GetModelState.h"
#include <ctime>
#include <random>
#include <cstdlib>
#include <deque>
#include <eigen3/Eigen/Dense>

std::deque<std::string> obs_name;
std::deque<double> posx;
std::deque<double> posy;
geometry_msgs::Point pr2_position;
gazebo_msgs::ModelState obstacle;
ros::ServiceClient *clientPtr;
std::deque<int> model;
std::deque<double> distx;
std::deque<double> disty;
std::deque<double> dx;
std::deque<double> dy;

int flag_move = 0;
int cnt=0;
int dt = 100; //100hz

auto curTime = std::chrono::system_clock::now();
auto duration = curTime.time_since_epoch();
auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();

auto curTime2 = std::chrono::system_clock::now();
auto duration2 = curTime.time_since_epoch();
auto sec = std::chrono::duration_cast<std::chrono::seconds>(duration2).count();

std::mt19937 mtRand(millis);
std::mt19937 mtRand2(sec);

int getRandomNumber(int min, int max){
    return min+static_cast<int>(mtRand()%(max-min+1));
}
int getRandomNumber2(int min, int max){
    return min+static_cast<int>(mtRand2()%(max-min+1));
}
void callback(const gazebo_msgs::ModelStates::ConstPtr& msg);

int main(int argc, char** argv){
    ros::init(argc, argv, "randomwalking_obstacles");
    ros::NodeHandle nh;
    ros::Subscriber obs_pos;
    obs_pos = nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states",1,callback);
    ros::ServiceClient client = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
    ros::Rate loop_rate(100);
    clientPtr = &client;
    while(ros::ok()){
     
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;
}
void callback(const gazebo_msgs::ModelStates::ConstPtr& msg){
    int num = msg->name.size();
    for(int i=0; i<num; i++){
        obs_name.push_back(msg->name[i]);
        posx.push_back(msg->pose[i].position.x);
        posy.push_back(msg->pose[i].position.y);
    }
    ROS_INFO_STREAM(num);

    if(flag_move == 0){
    model.clear();
    distx.clear();
    disty.clear();
    dx.clear();
    dy.clear();
    int group = getRandomNumber(3,num-3);
    for(int j=0; j<group; j++){
        model.push_back(getRandomNumber(1,num-3));
        distx.push_back(double(getRandomNumber(-2,2)*0.002));
        disty.push_back(double(getRandomNumber(-2,2)*0.002));
        
    }

    dx = distx;
    dy = disty;
    
    flag_move = 1;
    cnt=0;
    }
    else if(flag_move == 1){
        cnt+=1;
        for(int k=0; k<model.size(); k++){
        pr2_position.x = msg->pose[model[k]].position.x;
        pr2_position.y = msg->pose[model[k]].position.y;
        pr2_position.z = 0.0;
        pr2_position.x += dx[k];
        pr2_position.y += dy[k];
        geometry_msgs::Quaternion pr2_orientation;
        pr2_orientation.x = 0.0;
        pr2_orientation.y = 0.0;
        pr2_orientation.z = 0.0;
        pr2_orientation.w = 1.0;

        geometry_msgs::Pose pr2_pose;
        pr2_pose.position = pr2_position;
        pr2_pose.orientation = pr2_orientation;
        obstacle.model_name = msg->name[model[k]];
        obstacle.pose = pr2_pose;
        gazebo_msgs::SetModelState srv;
        srv.request.model_state = obstacle;
        ros::ServiceClient client = (ros::ServiceClient)*clientPtr;
        if(client.call(srv)){
            ROS_INFO("success");
            ROS_INFO_STREAM(dx[1]);

        }
        else{
            ROS_ERROR("Failed to move, Error msg:%s",srv.response.status_message.c_str());
        }
        if(cnt==dt){
            flag_move=0;
        }
    }
    }
}