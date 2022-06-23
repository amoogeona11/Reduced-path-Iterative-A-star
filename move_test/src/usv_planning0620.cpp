#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/UInt32.h"
#include "std_msgs/UInt32MultiArray.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "gazebo_msgs/ModelStates.h"
#include "gazebo_msgs/GetModelState.h"
#include "gazebo_msgs/SetModelState.h"
#include <iostream>
#include <cmath>
#include <list>
#include <algorithm>
#include <deque>
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include "Optimize.h"

Optimize optm;
using namespace Eigen;
using Eigen::MatrixXd;
const int OFFSET = -10;
struct quaternion
{
  double w;
  double x;
  double y;
  double z;
};

/////////////////////////////////////a-star////////////////////////////////////

class smooth_point {
public:
    smooth_point( double a = 0, double b = 0 ) { x = a; y = b; }
    bool operator ==( const smooth_point& p ) { return p.x == x && p.y == y; }
    smooth_point operator +( const smooth_point& p ) { return smooth_point( p.x + x, p.y + y ); }
    double x, y;
};

std::deque<smooth_point> smooth_path;
std::deque<smooth_point> smooth_vel;
std::deque<smooth_point> smooth_acc;

class point {
public:
    point( int a = 0, int b = 0 ) { x = a; y = b; }
    bool operator ==( const point& o ) { return o.x == x && o.y == y; }
    point operator +( const point& o ) { return point( o.x + x, o.y + y ); }
    int x, y;
};

class local_map{
public:
    local_map() {
        char t[20][20];
        // {
        //     {0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0},
        //     {0, 0, 0, 0, 1, 1, 1, 0}, {0, 0, 1, 0, 0, 0, 1, 0},
        //     {0, 0, 1, 0, 0, 0, 1, 0}, {0, 0, 1, 1, 1, 1, 1, 0},
        //     {0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0}
        // };
        w = h = 20;

        for( int r = 0; r < h; r++ )
            for( int s = 0; s < w; s++ )
                t[s][r] = 0;
//        t[0][2]=1; t[1][2]=1; t[2][2]=1; t[3][2]=1; t[4][2]=1; t[5][2]=1; t[15][0]=1;
//        t[15][1]=1; t[15][2]=1; t[15][3]=1; t[15][4]=1; t[15][5]=1; t[15][6]=1; t[15][7]=1;
//        t[15][8]=1; t[14][6]=1; t[13][6]=1; t[12][6]=1; t[11][6]=1; t[10][6]=1; t[9][6]=1;
//        t[8][6]=1; t[7][6]=1; t[6][6]=1;
        for( int r = 0; r < h; r++ )
            for( int s = 0; s < w; s++ )
                m[s][r] = t[r][s];
    }
    int operator() ( int x, int y ) { return m[x][y]; }
    char m[20][20];
    int w, h;
};

class global_map{
public:
    global_map() {
        char t[100][100];
        // {
        //     {0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0},
        //     {0, 0, 0, 0, 1, 1, 1, 0}, {0, 0, 1, 0, 0, 0, 1, 0},
        //     {0, 0, 1, 0, 0, 0, 1, 0}, {0, 0, 1, 1, 1, 1, 1, 0},
        //     {0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0}
        // };
        w = h = 100;

        for( int r = 0; r < h; r++ )
            for( int s = 0; s < w; s++ )
                t[s][r] = 0;
//        t[0][2]=1; t[1][2]=1; t[2][2]=1; t[3][2]=1; t[4][2]=1; t[5][2]=1; t[15][0]=1;
//        t[15][1]=1; t[15][2]=1; t[15][3]=1; t[15][4]=1; t[15][5]=1; t[15][6]=1; t[15][7]=1;
//        t[15][8]=1; t[14][6]=1; t[13][6]=1; t[12][6]=1; t[11][6]=1; t[10][6]=1; t[9][6]=1;
//        t[8][6]=1; t[7][6]=1; t[6][6]=1;
        for( int r = 0; r < h; r++ )
            for( int s = 0; s < w; s++ )
                m[s][r] = t[r][s];
    }
    int operator() ( int x, int y ) { return m[x][y]; }
    char m[100][100];
    int w, h;
};

class node {
public:
    bool operator == (const node& o ) { return pos == o.pos; }
    bool operator == (const point& o ) { return pos == o; }
    bool operator < (const node& o ) { return dist + cost < o.dist + o.cost; }
    point pos, parent;
    int dist, cost;
};


class local_aStar {
public:
    local_aStar() {
        neighbours[0] = point( -1, -1 ); neighbours[1] = point(  1, -1 );
        neighbours[2] = point( -1,  1 ); neighbours[3] = point(  1,  1 );
        neighbours[4] = point(  0, -1 ); neighbours[5] = point( -1,  0 );
        neighbours[6] = point(  0,  1 ); neighbours[7] = point(  1,  0 );
    }

    int calcDist( point& p ){
        // need a better heuristic
        int x = end.x - p.x, y = end.y - p.y;
        return( abs(x) + abs(y) );
    }

    bool isValid( point& p ) {
        return ( p.x >-1 && p.y > -1 && p.x < m.w && p.y < m.h );
    }

    bool existPoint( point& p, int cost ) {
        std::list<node>::iterator i;
        i = std::find( closed.begin(), closed.end(), p );
        if( i != closed.end() ) {
            if( ( *i ).cost + ( *i ).dist < cost ) return true;
            else { closed.erase( i ); return false; }
        }
        i = std::find( open.begin(), open.end(), p );
        if( i != open.end() ) {
            if( ( *i ).cost + ( *i ).dist < cost ) return true;
            else { open.erase( i ); return false; }
        }
        return false;
    }

    bool fillOpen( node& n ) {
        int stepCost, nc, dist;
        point neighbour;

        for( int x = 0; x < 8; x++ ) {
            // one can make diagonals have different cost
            stepCost = x < 4 ? 7 : 5;
            neighbour = n.pos + neighbours[x];
            if( neighbour == end ) return true;

            if( isValid( neighbour ) && m( neighbour.x, neighbour.y ) != 1 ) {
              if( m( n.pos.x, neighbour.y ) != 1 && m( neighbour.x, n.pos.y ) != 1 ){
                nc = stepCost + n.cost;
                dist = calcDist( neighbour );
                if( !existPoint( neighbour, nc + dist ) ) {
                    node m;
                    m.cost = nc; m.dist = dist;
                    m.pos = neighbour;
                    m.parent = n.pos;
                    open.push_back( m );
                }
              }
            }
        }
        return false;
    }

    bool search( point& s, point& e, local_map& mp ) {
        node n; end = e; start = s; m = mp;
        n.cost = 0; n.pos = s; n.parent = 0; n.dist = calcDist( s );
        open.push_back( n );
        while( !open.empty() ) {
            open.sort();
            node n = open.front();
            open.pop_front();
            closed.push_back( n );
            if( fillOpen( n ) ) return true;
        }
        return false;
    }

    int path( std::deque<point>& path ) {
        path.push_front( end );
        int cost = 1 + closed.back().cost;
        path.push_front( closed.back().pos );
        point parent = closed.back().parent;

        for( std::list<node>::reverse_iterator i = closed.rbegin(); i != closed.rend(); i++ ) {
            if( ( *i ).pos == parent && !( ( *i ).pos == start ) ) {
                path.push_front( ( *i ).pos );
                parent = ( *i ).parent;
            }
        }
        path.push_front( start );
        return cost;
    }
    local_map m; point end, start;
    point neighbours[8];
    std::list<node> open;
    std::list<node> closed;
};

class global_aStar {
public:
    global_aStar() {
        neighbours[0] = point( -1, -1 ); neighbours[1] = point(  1, -1 );
        neighbours[2] = point( -1,  1 ); neighbours[3] = point(  1,  1 );
        neighbours[4] = point(  0, -1 ); neighbours[5] = point( -1,  0 );
        neighbours[6] = point(  0,  1 ); neighbours[7] = point(  1,  0 );
    }

    int calcDist( point& p ){
        // need a better heuristic
        int x = end.x - p.x, y = end.y - p.y;
        return( abs(x) + abs(y) );
    }

    bool isValid( point& p ) {
        return ( p.x >-1 && p.y > -1 && p.x < m.w && p.y < m.h );
    }

    bool existPoint( point& p, int cost ) {
        std::list<node>::iterator i;
        i = std::find( closed.begin(), closed.end(), p );
        if( i != closed.end() ) {
            if( ( *i ).cost + ( *i ).dist < cost ) return true;
            else { closed.erase( i ); return false; }
        }
        i = std::find( open.begin(), open.end(), p );
        if( i != open.end() ) {
            if( ( *i ).cost + ( *i ).dist < cost ) return true;
            else { open.erase( i ); return false; }
        }
        return false;
    }

    bool fillOpen( node& n ) {
        int stepCost, nc, dist;
        point neighbour;

        for( int x = 0; x < 8; x++ ) {
            // one can make diagonals have different cost
            stepCost = x < 4 ? 7 : 5;
            neighbour = n.pos + neighbours[x];
            if( neighbour == end ) return true;

            if( isValid( neighbour ) && m( neighbour.x, neighbour.y ) != 1 ) {
              if( m( n.pos.x, neighbour.y ) != 1 && m( neighbour.x, n.pos.y ) != 1 ){
                nc = stepCost + n.cost;
                dist = calcDist( neighbour );
                if( !existPoint( neighbour, nc + dist ) ) {
                    node m;
                    m.cost = nc; m.dist = dist;
                    m.pos = neighbour;
                    m.parent = n.pos;
                    open.push_back( m );
                }
              }
            }
        }
        return false;
    }

    bool search( point& s, point& e, global_map& mp ) {
        node n; end = e; start = s; m = mp;
        n.cost = 0; n.pos = s; n.parent = 0; n.dist = calcDist( s );
        open.push_back( n );
        while( !open.empty() ) {
            open.sort();
            node n = open.front();
            open.pop_front();
            closed.push_back( n );
            if( fillOpen( n ) ) return true;
        }
        return false;
    }

    int path( std::deque<point>& path ) {
        path.push_front( end );
        int cost = 1 + closed.back().cost;
        path.push_front( closed.back().pos );
        point parent = closed.back().parent;

        for( std::list<node>::reverse_iterator i = closed.rbegin(); i != closed.rend(); i++ ) {
            if( ( *i ).pos == parent && !( ( *i ).pos == start ) ) {
                path.push_front( ( *i ).pos );
                parent = ( *i ).parent;
            }
        }
        path.push_front( start );
        return cost;
    }
    global_map m; point end, start;
    point neighbours[8];
    std::list<node> open;
    std::list<node> closed;
};

int global_flag=1;
int local_flag=0;
point l_s, l_e;
point g_s, g_e;
local_aStar l_as;
global_aStar g_as;
int global_goal_x; //임의로 정해준 것
int global_goal_y;
point local_goal;
int global_path_flag=0;
std::deque<point> global_path; // global trajectory
std::deque<point> local_path; // local trajectory
global_map global;
local_map local;
nav_msgs::Path trajectory;
geometry_msgs::PoseStamped traj_pose;
class SubscribeAndPublish
{
public:
  SubscribeAndPublish()
  {
    pub_ = n_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    pub_1 = n_.advertise<std_msgs::Float32>("process_time",10);
    pub_2 = n_.advertise<std_msgs::Float32>("run_time",10);
    // pub_3 = n_.advertise<nav_msgs::Path>("trajectory",10);
    sub_ = n_.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states",10,&SubscribeAndPublish::callback, this);
  }
    void callback(const gazebo_msgs::ModelStates::ConstPtr& msg){
        // Global planning 1회 실행. Command를 주면 global planning 시행
        quaternion robot_quar;
        robot_quar.w = msg->pose.back().orientation.w;
        robot_quar.x = msg->pose.back().orientation.x;
        robot_quar.y = msg->pose.back().orientation.y;
        robot_quar.z = msg->pose.back().orientation.z;
        double robot_rawx=msg->pose.back().position.x;
        double robot_rawy=msg->pose.back().position.y;
        double goal_rawx=msg->pose[msg->name.size()-2].position.x;
        double goal_rawy=msg->pose[msg->name.size()-2].position.y;
        
        int robot_posx = int(std::round(msg->pose.back().position.x))+10; // 로봇 현재 위치
        int robot_posy = int(std::round(msg->pose.back().position.y))+10;

        if(global_flag==1){
            for(int i=0; i<20; i++){
                for(int j=0; j<20; j++){
                    global.m[i][j]=0;
                }
            }
            g_s.x = robot_posx;
            g_s.y = robot_posy;
            global_goal_x = int(std::round(msg->pose[msg->name.size()-2].position.x))+10;
            global_goal_y = int(std::round(msg->pose[msg->name.size()-2].position.y))+10;
            for (int i=1; i<msg->name.size()-2; i++){
                //global.m[int(std::round(msg->pose[i].position.x))+10][int(std::round(msg->pose[i].position.y))+10] = 1;
            }
            g_e.x = global_goal_x;
            g_e.y = global_goal_y;
            global_path.clear();
            g_as.open.clear();
            g_as.closed.clear();
            //// map 정보 업데이트 필요 ////
            // 현 시뮬레이션에서 global에서는 장애물 없다고 가정해도 좋을듯 
            if(g_as.search(g_s,g_e,global)){
                int g_c = g_as.path(global_path);
                global_path.pop_front();
                global_path.push_back(g_e);
                global_flag=0;
                local_flag=1;
            }
        }
        // ROS_INFO_STREAM("global_pose: " << global_goal_x << "y: " << global_goal_y);
        ROS_INFO_STREAM("global_goal: x: " << global_path.back().x << "y: " << global_path.back().y);
        ROS_INFO_STREAM("robot_pose: " << msg->pose.back().position.x << ", " << msg->pose.back().position.y << "," << msg->pose.back().position.z);
        if(sqrt(pow((robot_rawx-goal_rawx),2)+pow((robot_rawy-goal_rawy),2))<0.01){
            robot_msg = move_func(robot_rawx,robot_rawy,robot_quar,goal_rawx,goal_rawy);
            pub_.publish(robot_msg);
            // robot_msg.linear.x = 0;
            // robot_msg.angular.z = 0;
            local_flag=0;
            pub_.publish(robot_msg);
            ROS_INFO_STREAM("arrived");
        }
        else{
        // Local planning callback function이 돌아갈 때마다 시행
            if(local_flag==1){
                for(int i=0;i<20;i++){
                    for(int j=0;j<20;j++){
                    local.m[i][j] = 0;
                    }
                }
                for(int i=1;i<msg->name.size()-2;i++){
                    int a = int(std::round(msg->pose[i].position.x))+10-robot_posx+10;
                    int b = int(std::round(msg->pose[i].position.y))+10-robot_posy+10;
                    if ((0<=a&&a<=19)&&(0<=b&&b<=19)){ //local map 내에 장애물이 감지되면 탐색 진행

                        // local.m[a][b]=1;
                        if ((0<a&&a<19)&&(0<b&&b<19)){ //보수적으로 장애물 위치 지정
                            local.m[a][b]=1;
                            local.m[a][b+1]=1;
                            local.m[a][b-1]=1;
                            local.m[a-1][b-1]=1;
                            local.m[a-1][b]=1;
                            local.m[a-1][b+1]=1;
                            local.m[a+1][b-1]=1;
                            local.m[a+1][b]=1;
                            local.m[a+1][b+1]=1;
                        }
                        else if(a==0&&(0<b&&b<19)){
                            local.m[a][b]=1;
                            local.m[a][b+1]=1;
                            local.m[a][b-1]=1;
                            local.m[a+1][b-1]=1;
                            local.m[a+1][b]=1;
                            local.m[a+1][b+1]=1;
                        }
                        else if((0<a&&a<19)&&b==0){
                            local.m[a][b]=1;
                            local.m[a][b+1]=1;
                            local.m[a-1][b]=1;
                            local.m[a-1][b+1]=1;
                            local.m[a+1][b]=1;
                            local.m[a+1][b+1]=1;
                        }
                        else if(a==0&&b==0){
                            local.m[a][b]=1;
                            local.m[a][b+1]=1;
                            local.m[a+1][b]=1;
                        }
                        else if(a==19&&b==19){
                            local.m[a][b]=1;
                            local.m[a][b-1]=1;
                            local.m[a-1][b]=1;
                        }          
                        else if(a==0&&b==19){
                            local.m[a][b]=1;
                            local.m[a][b-1]=1;
                            local.m[a+1][b]=1;
                        }          
                        else if(a==19&&b==0){
                            local.m[a][b]=1;
                            local.m[a][b+1]=1;
                            local.m[a-1][b]=1;
                        }        
                        }
                }
            local_goal = decide_local_goal(global_path, robot_posx, robot_posy); // 이 함수 어떤 방식으로 할 지 결정 필요
            // local_goal = decide_local_goal(global_path, robot_rawx, robot_rawy);
            local.m[local_goal.x][local_goal.y]=0;
            local.m[10][10]=0;
            l_s.x = 10;
            l_s.y = 10;
            l_e.x = local_goal.x;
            l_e.y = local_goal.y;
            ROS_INFO_STREAM("local start: " << l_s.x << ", " << l_s.y);
            ROS_INFO_STREAM("local goal: " << l_e.x << ", " << l_e.y);
            l_as.open.clear();
            l_as.closed.clear();
            local_path.clear();

            if(l_as.search(l_s,l_e,local)){
                int l_c = l_as.path(local_path);
                ROS_INFO_STREAM("succeeded");
                smooth_path.clear();

                MatrixXd points = MatrixXd::Zero(3,local_path.size());
                points(0,0) = robot_rawx + 10 - robot_posx + 10;
                points(1,0) = robot_rawy + 10 - robot_posy + 10;
                points(2,0) = 0;
                for (int opt=0; opt<local_path.size(); opt++){
                    points(0,opt) = double(local_path[opt].x);
                    points(1,opt) = double(local_path[opt].y);
                    points(2,opt) = 0.0;
                }
                ROS_INFO_STREAM("local path transmitted");
                MatrixXd smooth_traj;
                smooth_traj = optm.trajoptimize(points);
                ROS_INFO_STREAM("smooth_traj cols" << smooth_traj.cols());
                ROS_INFO_STREAM("optimization success");
                for (int spl=0; spl<smooth_traj.cols(); spl++){
                    smooth_point point_tmp;
                    point_tmp.x = smooth_traj(0,spl);
                    point_tmp.y = smooth_traj(1,spl);
                    smooth_path.push_back(point_tmp);
                }
                
                ROS_INFO_STREAM("smooth_path_start: " << smooth_path.begin()->x << ", " << smooth_path.begin()->y);
                // ROS_INFO_STREAM("smooth_path_start: " << smooth_path[1].x << ", " << smooth_path[1].y);
                
                local_path.pop_front();
                if(local_path.begin()->x==10 && local_path.begin()->y==10){
                    local_path.pop_front();
                    smooth_path.pop_front();
                }
                ROS_INFO_STREAM("local_begin: " << local_path.begin()->x << ", " << local_path.begin()->y);
                smooth_path.pop_front();


                ROS_INFO_STREAM("smooth_begin: " << smooth_path.begin()->x << ", " << smooth_path.begin()->y);
                }
            }
            else{
                ROS_INFO_STREAM("FAIL");
            }
            // for(int p=0; p<local_path.size();p++){
            //     traj_pose.header.frame_id="odom";
            //     traj_pose.header.seq=p;
            //     traj_pose.header.stamp = ros::Time::now();
            //     traj_pose.pose.position.x = local_path[p].x;
            //     traj_pose.pose.position.y = local_path[p].y;
            //     traj_pose.pose.position.z = 0;
            //     traj_pose.pose.orientation.w=0;
            //     traj_pose.pose.orientation.x=0;
            //     traj_pose.pose.orientation.y=0;
            //     traj_pose.pose.orientation.z=1;

            //     trajectory.poses.push_back(traj_pose);
            // }
            

            // double dest_x = local_path.begin()->x-10;
            // double dest_y = local_path.begin()->y-10;
            double dest_x = smooth_path.begin()->x-10;
            double dest_y = smooth_path.begin()->y-10;
            double x = msg->pose.back().position.x;
            double y = msg->pose.back().position.y;
            double dist = sqrt(pow((dest_x-x),2)+pow((dest_y-y),2));
            // if(dist<0.01){
            //     local_path.pop_front();
            //     double dest_x = local_path.begin()->x-10-10;
            //     double dest_y = local_path.begin()->y-10-10;
            // }
            
            robot_msg = move_func(x,y,robot_quar,x+dest_x,y+dest_y);
            pub_.publish(robot_msg);
            // pub_3.publish(trajectory);
            }
    }
        // Local planning시 Global path를 기반으로 Goal point 결정 필요
        // Local trajectory 출력
    
    point decide_local_goal(std::deque<point> global_path, int robot_pose_x, int robot_pose_y){
        // Local planning시 goal point를 결정하는 함수
        point dest;
        int idx = 0;
        double min = sqrt(pow(global_path[idx].x-robot_pose_x,2)+pow(global_path[idx].y-robot_pose_y,2));
        int idx_last = global_path.size()-1;
        if(0<=global_path[idx_last].x-robot_pose_x+10&&global_path[idx_last].x-robot_pose_x+10<20 && 0<=global_path[idx_last].y-robot_pose_y+10&&global_path[idx_last].y-robot_pose_y+10<20)
        {
            dest.x = global_path.back().x-robot_pose_x+10;
            dest.y = global_path.back().y-robot_pose_y+10;

            return dest;
        }
        else{
            for(int i=0;i<global_path.size();i++){
                if (min>sqrt(pow(global_path[i].x-robot_pose_x,2)+pow(global_path[i].y-robot_pose_y,2))){
                    min = sqrt(pow(global_path[i].x-robot_pose_x,2)+pow(global_path[i].y-robot_pose_y,2));
                    idx = i;
                }
            }
            
            for(int i=idx;i<global_path.size();i++){
                int dest_tmpx = global_path[i].x-robot_pose_x+10;
                int dest_tmpy = global_path[i].y-robot_pose_y+10;
                if((dest_tmpx == 19) || (dest_tmpx == 0) || (dest_tmpy ==19) || (dest_tmpy ==0)){
                    dest.x=dest_tmpx;
                    dest.y=dest_tmpy;
                    if(dest.x>19){
                        dest.x=19;
                    }
                    if(dest.y>19){
                        dest.y=19;
                    }
                    return dest;
                }


                }
            }
    }
    geometry_msgs::Twist move_func(double x, double y, quaternion quar, double dest_x, double dest_y){

        quaternion q = quar;
        double angle;
        double vel;
        double t;
        double w;

        t = 1;
        angle = std::atan2(2*(q.w*q.z+q.x*q.y),1-(2*(pow(q.y,2)+pow(q.z,2))));
        double dest_angle = std::atan2(dest_y-y,dest_x-x);
        //double robot_direction[2] = {x+cos(angle), y+sin(angle)};
        //double robot_location[2] = {x,y};
        //double dest_direction[2] = {dest_x, dest_y};
        double cross_product = cos(angle)*(dest_y-y)-sin(angle)*(dest_x-x);
        double dot_product = cos(angle)*(dest_x-x)+sin(angle)*(dest_y-y);
        double dist = sqrt(pow((dest_x-x),2)+pow((dest_y-y),2));

        //vel = 0.2/(dist+1);
        w = 2*std::acos(dot_product/std::sqrt(pow(dest_x-x,2)+pow(dest_y-y,2)));
        //vel = 0.5;
        vel = 0.8/(abs(w)+1);
        
        // ROS_INFO_STREAM("dest_angle:" << dest_angle);
        // ROS_INFO_STREAM("robot_angle:" << angle);
        ROS_INFO_STREAM("velocity: " << w << ", " << vel);
        ROS_INFO_STREAM("dest_point:" << dest_x << "," << dest_y);

        if (dist < 0.01) {
            robot_msg.linear.x = 0;
            robot_msg.angular.z = 0;
            // local_path.pop_front();
            // global_path.pop_front();
        }
        
        else{
            robot_msg.linear.x = vel;

            if(cross_product<0){
            robot_msg.angular.z = w;
            }
            else{
            robot_msg.angular.z = -w;
            }
        }

        return robot_msg;
        }
    // std::deque<double> spline(std::deque<int> trajectory){
    //     // Local Planning 으로 얻은 trajectory를 최적화하여 반환
    // }


private:
  ros::NodeHandle n_;
  ros::Publisher pub_;
  ros::Publisher pub_1;
  ros::Publisher pub_2;
//   ros::Publisher pub_3;
  ros::Subscriber sub_;

  geometry_msgs::Twist robot_msg;
};
int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_pub");

  ros::Time::init();
  ros::Rate loop_rate(10);
  ROS_INFO_STREAM("initiate");
//   double robot_posx = 0;
//   double robot_posy = 0;
  SubscribeAndPublish SAPObject;


    while (ros::ok())
    {
  //    msg.linear.x = 0;
  //    msg.angular.z = 0;
  //    chatter_pub.publish(msg);

      ros::spinOnce();

      loop_rate.sleep();
    }

  return 0;

}

// updated