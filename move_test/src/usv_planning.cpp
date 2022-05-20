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

const int OFFSET = -10;
struct quaternion
{
  double w;
  double x;
  double y;
  double z;
};

/////////////////////////////////////a-star////////////////////////////////////

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


class SubscribeAndPublish
{
public:
  SubscribeAndPublish()
  {
    pub_ = n_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    pub_1 = n_.advertise<std_msgs::Float32>("process_time",10);
    pub_2 = n_.advertise<std_msgs::Float32>("run_time",10);
    sub_ = n_.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states",10,&SubscribeAndPublish::callback, this);
  }
    void callback(const gazebo_msgs::ModelStates::ConstPtr& msg){
        // Global planning 1회 실행. Command를 주면 global planning 시행
        // Local planning callback function이 돌아갈 때마다 시행
        // Local planning시 Global path를 기반으로 Goal point 결정 필요
        // Local trajectory 출력
    }
    point decide_local_goal(){
        // Local planning시 goal point를 결정하는 함수
    }
    geometry_msgs::Twist move_func(std::deque<point> local_trajectory, quaternion quar, double x, double y){
    // x,y : current robot position
    // quar : current robot orientation
    
    quaternion q = quar;
    double angle;
    double vel;
    double t;
    double w;
    double dest_x = local_trajectory[0].x;
    double dest_y = local_trajectory[0].y;
    local_trajectory.pop_front();
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
    vel = 0.8/(w+1);
    ROS_INFO_STREAM("dest_angle:" << dest_angle);
    ROS_INFO_STREAM("robot_angle:" << angle);
    ROS_INFO_STREAM("dest_point:" << dest_x << "," << dest_y);
    if (dist < 0.1) {
        robot_msg.linear.x = 0;
        robot_msg.angular.z = 0;
        local_trajectory.pop_front();
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
    std::deque<double> spline(std::deque<int> trajectory){
        // Local Planning 으로 얻은 trajectory를 최적화하여 반환
    }


private:
  ros::NodeHandle n_;
  ros::Publisher pub_;
  ros::Publisher pub_1;
  ros::Publisher pub_2;
  ros::Subscriber sub_;

  geometry_msgs::Twist robot_msg;
};
