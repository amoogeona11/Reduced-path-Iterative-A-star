#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "gazebo_msgs/ModelStates.h"
#include "gazebo_msgs/GetModelState.h"
#include <iostream>
#include <cmath>
#include <list>
#include <algorithm>
#include <deque>

const int OFFSET = -10;
struct quarternion
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

class map{
public:
    map() {
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


class node {
public:
    bool operator == (const node& o ) { return pos == o.pos; }
    bool operator == (const point& o ) { return pos == o; }
    bool operator < (const node& o ) { return dist + cost < o.dist + o.cost; }
    point pos, parent;
    int dist, cost;
};


class aStar {
public:
    aStar() {
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

    bool search( point& s, point& e, map& mp ) {
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

    map m; point end, start;
    point neighbours[8];
    std::list<node> open;
    std::list<node> closed;
};
/////////////////////////////////////////////////////////////////////////////////////////

//int main( int argc, char* argv[] ) {
//    map m;
//    int offset = -10;
//    point s, e( 7, 7 );
//    aStar as;

//    if( as.search( s, e, m ) ) {
//        std::list<point> path;
//        int c = as.path( path );
//        for( int y = -1; y < 9; y++ ) {
//            for( int x = -1; x < 9; x++ ) {
//                if( x < 0 || y < 0 || x > 7 || y > 7 || m( x, y ) == 1 )
//                    std::cout << "1";
//                else {
//                    if( std::find( path.begin(), path.end(), point( x, y ) )!= path.end() )
//                        std::cout << "x";
//                    else std::cout << "0";
//                }
//            }
//            std::cout << "\n";
//        }

//        std::cout << "\nPath cost " << c << ": ";
//        for( std::list<point>::iterator i = path.begin(); i != path.end(); i++ ) {
//            std::cout<< "(" << ( *i ).x << ", " << ( *i ).y << ") ";
//        }
//    }
//    std::cout << "\n\n";
//    return 0;
//}

geometry_msgs::Twist robot_msg;

geometry_msgs::Twist move_func(double x, double y, quarternion quar, double dest_x, double dest_y);
point s, e;
aStar as;

map mapp;
int flag_obs=1;
int flag_search=0;
int flag_dest=1;
int dx, dy, rx, ry;
int len;
int cnt = 1;
std::deque<point> path;

class SubscribeAndPublish
{
public:
  SubscribeAndPublish()
  {
    pub_ = n_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    sub_ = n_.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states",10,&SubscribeAndPublish::callback, this);
  }
  void callback(const gazebo_msgs::ModelStates::ConstPtr& msg){
    if(flag_obs==1){
      as.closed.clear();
      as.open.clear();
      for(int i=0;i<20;i++){
        for(int j=0;j<20;j++){
          mapp.m[i][j] = 0;
        }
      }
      int i=1;
      while(msg->name[i] != "sjtu_drone"){
        int a,b;
        a = std::round(msg->pose[i].position.x)+10;
        b = std::round(msg->pose[i].position.y)+10;
        mapp.m[a][b]=1;
        i++;
      }
      len = i+2;
      ROS_INFO_STREAM("obstacles detected");
      flag_obs=0;
      flag_search=1;
    }
    if(flag_dest==1){
      dx = int(std::round(msg->pose[len-2].position.x))+10;
      dy = int(std::round(msg->pose[len-2].position.y))+10;
      rx = int(std::round(msg->pose.back().position.x))+10;
      ry = int(std::round(msg->pose.back().position.y))+10;
      s.x = rx;
      s.y = ry;
      e.x = dx;
      e.y = dy;
      flag_dest=0;
      flag_search=1;

    }
    if((dx != int(std::round(msg->pose[len-2].position.x))+10) || (dy != int(std::round(msg->pose[len-2].position.y))+10)){
      robot_msg.linear.x=0;
      robot_msg.angular.z=0;
      pub_.publish(robot_msg);
      flag_dest=1;
      flag_search=0;
      flag_obs=1;
    }
    if(flag_obs==0 && flag_search==1){
      path.clear();
      if(as.search(s,e,mapp)){
        int c = as.path( path );
        ROS_INFO_STREAM("success");
        if(cnt != 1){
          path.pop_front();
        }
        cnt += 1;
        flag_search=0;
      }
    }

    else if(flag_obs==0 && flag_search==0){
      ROS_INFO_STREAM("robot name:" << msg->name.back());
      ROS_INFO_STREAM("Position:" << msg->pose.back().position.x << "," << msg->pose.back().position.y << "," << msg->pose.back().position.z);
      ROS_INFO_STREAM("destination:" << std::round(msg->pose[len-2].position.x) << "," << std::round(msg->pose[len-2].position.y));
     // ROS_INFO_STREAM("(" <<path[2].x << "," << path[2].y << ")");

      quarternion robot_quar;
      robot_quar.w = msg->pose.back().orientation.w;
      robot_quar.x = msg->pose.back().orientation.x;
      robot_quar.y = msg->pose.back().orientation.y;
      robot_quar.z = msg->pose.back().orientation.z;
      double dest_x = path.begin()->x+OFFSET;
      double dest_y = path.begin()->y+OFFSET;
      double x = msg->pose.back().position.x;
      double y = msg->pose.back().position.y;
      double dist = sqrt(pow((dest_x-x),2)+pow((dest_y-y),2));

      //int c = as.path( path );
      if(path.empty()){
        robot_msg.linear.x=0;
        robot_msg.angular.z=0;
        pub_.publish(robot_msg);
        //ros::Duration(0.5).sleep();
      }
      else{
        robot_msg = move_func(x, y, robot_quar, dest_x, dest_y);
        pub_.publish(robot_msg);

      }
    }

  }


private:
  ros::NodeHandle n_;
  ros::Publisher pub_;
  ros::Subscriber sub_;
};

//void callback(const gazebo_msgs::ModelStates::ConstPtr& msg);

//ros::NodeHandle nh;
//ros::Publisher chatter_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
//ros::Subscriber chatter_sub = nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states",5,callback);



geometry_msgs::Twist move_func(double x, double y, quarternion quar, double dest_x, double dest_y){

  quarternion q = quar;
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
  vel = 0.8/(w+1);
  ROS_INFO_STREAM("dest_angle:" << dest_angle);
  ROS_INFO_STREAM("robot_angle:" << angle);
  ROS_INFO_STREAM("dest_point:" << dest_x << "," << dest_y);
  if (dist < 0.1) {
    robot_msg.linear.x = 0;
    robot_msg.angular.z = 0;
    path.pop_front();
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

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_pub");

  ros::Time::init();
  ros::Rate loop_rate(10);
  double robot_posx = 0;
  double robot_posy = 0;
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

