#include <iostream>
#include "Optimize.h"
#include <deque>
#include <eigen3/Eigen/Dense>

using Eigen::MatrixXd;

Optimize optm;

class point {
public:
    point( int a = 0, int b = 0 ) { x = a; y = b; }
    bool operator ==( const point& o ) { return o.x == x && o.y == y; }
    point operator +( const point& o ) { return point( o.x + x, o.y + y ); }
    int x, y;
};
class smooth_point {
public:
    smooth_point( double a = 0, double b = 0 ) { x = a; y = b; }
    bool operator ==( const smooth_point& p ) { return p.x == x && p.y == y; }
    smooth_point operator +( const smooth_point& p ) { return smooth_point( p.x + x, p.y + y ); }
    double x, y;
};
std::deque<point> path;
std::deque<smooth_point> smooth_path;
MatrixXd orig_path = MatrixXd(3,8);

int main(){

    point path_tmp;

    path_tmp.x = 10;
    path_tmp.y = 10;
    path.push_back(path_tmp);

    path_tmp.x = 11;
    path_tmp.y = 10;
    path.push_back(path_tmp);

    path_tmp.x = 12;
    path_tmp.y = 10;
    path.push_back(path_tmp);

    path_tmp.x = 13;
    path_tmp.y = 10;
    path.push_back(path_tmp);

    path_tmp.x = 14;
    path_tmp.y = 10;
    path.push_back(path_tmp);

    path_tmp.x = 14;
    path_tmp.y = 11;
    path.push_back(path_tmp);

    path_tmp.x = 14;
    path_tmp.y = 12;
    path.push_back(path_tmp);

    path_tmp.x = 14;
    path_tmp.y = 13;
    path.push_back(path_tmp);

    for (int i=0; i<path.size(); i++){
        orig_path(0,i) = path[i].x;
        orig_path(1,i) = path[i].y;
        orig_path(2,i) = 0;
    }
    
    MatrixXd smooth = optm.trajoptimize(orig_path);
    for (int spl=0; spl<smooth.cols(); spl++){
        smooth_point point_tmp;
        point_tmp.x = smooth(0,spl);
        point_tmp.y = smooth(1,spl);
        smooth_path.push_back(point_tmp);
    }
    for (int i=0; i<smooth_path.size(); i++)
    std::cout << smooth_path[i].x << ", " << smooth_path[i].y << "\n";


    return 0;
}